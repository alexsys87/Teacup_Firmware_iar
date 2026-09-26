#include	"gcode_process.h"

/** \file
	\brief Work out what to do with received G-Code commands
*/

#include	<string.h>

#include	"gcode_parse.h"

#include "cpu.h"
#include	"dda.h"
#include	"dda_queue.h"
#include	"watchdog.h"
#include	"delay.h"
#include	"serial.h"
#include	"temp.h"
#include	"heater.h"
#include	"timer.h"
#include	"sersendf.h"
#include	"pinio.h"
#include	"debug.h"
#include	"clock.h"
#include	"config_wrapper.h"
#include	"home.h"
#include "sd.h"
#include "bed_leveling.h"
#include "kill.h"
#include "gcode_queue.h"
#include "status.h"
#include "beeper.h"
#include "dda_maths.h"
#include "settings.h"
#include "flash_store.h"
#include "pid_autotune.h"
#include "emergency_parser.h"
#include "babystep.h"
#include "filament.h"
#include "probe.h"
#include "servo.h"
#include <math.h>
#include <stdlib.h>

/// the current tool
uint8_t tool;

/// the tool to be changed when we get an M6
uint8_t next_tool;

/**
  Commands which use axis letters for values (M92, M201, M203, M205, M303)
  must not leave them in the move target, else the next move without that
  axis would go there. Restore what the target was before this line.
*/
static void restore_axis_word(enum axis_e a) {
  if (next_target.option_all_relative ||
      (a == E && next_target.option_e_relative))
    next_target.target.axis[a] = 0;
  else
    next_target.target.axis[a] = startpoint.axis[a];
}

/// Seen flags of X, Y, Z, E as an array.
static void seen_axes(uint8_t seen[4]) {
  seen[X] = next_target.seen_X;
  seen[Y] = next_target.seen_Y;
  seen[Z] = next_target.seen_Z;
  seen[E] = next_target.seen_E;
}

/**
  After homing / probing the nozzle is somewhere else than the G-code
  target: take the position of the end of the queue as the target, so the
  next move without X, Y or Z stays there.
*/
static void sync_target_to_startpoint(void) {
  enum axis_e a;

  for (a = X; a <= Z; a++)
    next_target.target.axis[a] =
      next_target.option_all_relative ? 0 : startpoint.axis[a];
}

/// Home offsets changed by M501/M502: shift coordinates like M206 does.
static void reapply_home_offsets(const int32_t old[3]) {
  enum axis_e a;

  for (a = X; a <= Z; a++) {
    int32_t new_offset = home_offset[a];

    home_offset[a] = old[a];
    home_set_offset(a, new_offset);
  }
}

/// Soft limits (X_MIN .. Z_MAX), shifted by the home offsets (M206).
static void apply_soft_limits(axes_int32_t axis) {
	#ifdef	X_MIN
    if (axis[X] < (int32_t)(X_MIN * 1000.) + home_offset[X])
      axis[X] = (int32_t)(X_MIN * 1000.) + home_offset[X];
	#endif
	#ifdef	X_MAX
    if (axis[X] > (int32_t)(X_MAX * 1000.) + home_offset[X])
      axis[X] = (int32_t)(X_MAX * 1000.) + home_offset[X];
	#endif
	#ifdef	Y_MIN
    if (axis[Y] < (int32_t)(Y_MIN * 1000.) + home_offset[Y])
      axis[Y] = (int32_t)(Y_MIN * 1000.) + home_offset[Y];
	#endif
	#ifdef	Y_MAX
    if (axis[Y] > (int32_t)(Y_MAX * 1000.) + home_offset[Y])
      axis[Y] = (int32_t)(Y_MAX * 1000.) + home_offset[Y];
	#endif
	#ifdef	Z_MIN
    if (axis[Z] < (int32_t)(Z_MIN * 1000.) + home_offset[Z])
      axis[Z] = (int32_t)(Z_MIN * 1000.) + home_offset[Z];
	#endif
	#ifdef	Z_MAX
    if (axis[Z] > (int32_t)(Z_MAX * 1000.) + home_offset[Z])
      axis[Z] = (int32_t)(Z_MAX * 1000.) + home_offset[Z];
	#endif
}

#ifndef NO_ARC_SUPPORT
/**
  G2 / G3: arc in the XY plane, split into straight segments.

  Center either by I, J (offset from the start point, always relative) or
  by R (radius; negative R means the arc longer than 180 degrees). Z moves
  linearly along the arc (helix), E is distributed by arc length. P adds
  full turns. The end point is the programmed one exactly, rounding never
  accumulates. Like Marlin: start == end with I/J is a full circle, bad
  parameters give a message and no move.

  \return 0 on bad parameters.
*/
static uint8_t arc_move(uint8_t clockwise) {
  const float two_pi = 6.28318530718f;
  float scale = next_target.option_inches ? 25.4f : 1.0f;
  float sx = (float)startpoint.axis[X], sy = (float)startpoint.axis[Y];
  float ex = (float)next_target.target.axis[X];
  float ey = (float)next_target.target.axis[Y];
  float cx, cy, r, a0, travel, seg_len;
  float rsx, rsy, rex, rey;
  int32_t z0 = startpoint.axis[Z], dz;
  int32_t e0 = 0, de;
  uint8_t start_is_end = startpoint.axis[X] == next_target.target.axis[X] &&
                         startpoint.axis[Y] == next_target.target.axis[Y];
  uint8_t qs = emergency_quickstop_count();
  uint32_t n, k;
  TARGET seg;

  if (next_target.seen_I || next_target.seen_J) {
    cx = sx + (next_target.seen_I ? (float)next_target.I_milli * scale : 0.0f);
    cy = sy + (next_target.seen_J ? (float)next_target.J_milli * scale : 0.0f);
    r = sqrtf((sx - cx) * (sx - cx) + (sy - cy) * (sy - cy));
  }
  else if (next_target.seen_R) {
    float dx = ex - sx, dy = ey - sy;
    float d = sqrtf(dx * dx + dy * dy);
    float h2, h, e;

    r = (float)next_target.R;
    if (start_is_end || d < 1.0f)
      return 0;                           // R can't describe a full circle.
    // Slightly too small radii (rounding in the G-code) give a half circle.
    h2 = (r - 0.5f * d) * (r + 0.5f * d);
    h = (h2 > 0.0f) ? sqrtf(h2) : 0.0f;
    // Center left of the chord for G3, right for G2; R < 0 swaps.
    e = ((clockwise != 0) ^ (r < 0.0f)) ? -1.0f : 1.0f;
    cx = sx + 0.5f * dx - e * h * dy / d;
    cy = sy + 0.5f * dy + e * h * dx / d;
    r = fabsf(r);
  }
  else {
    return 0;
  }
  if (r < 1.0f)                           // Below 1 um: nonsense.
    return 0;

  // Angle from start to end, counterclockwise positive.
  rsx = sx - cx; rsy = sy - cy;
  rex = ex - cx; rey = ey - cy;
  travel = atan2f(rsx * rey - rsy * rex, rsx * rex + rsy * rey);
  if (clockwise) {
    if (travel > 0.0f)
      travel -= two_pi;
  }
  else {
    if (travel < 0.0f)
      travel += two_pi;
  }
  if (start_is_end && fabsf(travel) < 1e-4f)
    travel = clockwise ? -two_pi : two_pi;
  if (next_target.seen_P && next_target.P)
    travel += (clockwise ? -two_pi : two_pi) * (float)next_target.P;

  // Segment length for the chord tolerance: sagitta = len^2 / (8 r).
  seg_len = sqrtf(8.0f * r * (float)(ARC_TOLERANCE * 1000.));
  if (seg_len < (float)(ARC_SEGMENT_MIN * 1000.))
    seg_len = (float)(ARC_SEGMENT_MIN * 1000.);
  if (seg_len > (float)(ARC_SEGMENT_MAX * 1000.))
    seg_len = (float)(ARC_SEGMENT_MAX * 1000.);
  n = (uint32_t)ceilf(fabsf(travel) * r / seg_len);
  if (n == 0)
    n = 1;

  dz = next_target.target.axis[Z] - z0;
  if (next_target.target.e_relative) {
    de = next_target.target.axis[E];
  }
  else {
    e0 = startpoint.axis[E];
    de = next_target.target.axis[E] - e0;
  }

  temp_wait();
  a0 = atan2f(rsy, rsx);
  seg = next_target.target;

  for (k = 1; k <= n; k++) {
    if (k < n) {
      float a = a0 + travel * (float)k / (float)n;

      seg.axis[X] = (int32_t)floorf(cx + r * cosf(a) + 0.5f);
      seg.axis[Y] = (int32_t)floorf(cy + r * sinf(a) + 0.5f);
      seg.axis[Z] = z0 + (int32_t)((int64_t)dz * k / n);
      apply_soft_limits(seg.axis);
    }
    else {
      seg.axis[X] = next_target.target.axis[X];
      seg.axis[Y] = next_target.target.axis[Y];
      seg.axis[Z] = next_target.target.axis[Z];
    }

    if (seg.e_relative)
      seg.axis[E] = (int32_t)((int64_t)de * k / n -
                              (int64_t)de * (k - 1) / n);
    else
      seg.axis[E] = e0 + (int32_t)((int64_t)de * k / n);

    // Wait here rather than in enqueue(): an M410 while waiting ends the
    // arc, no segment gets queued after it.
    while (queue_full())
      clock_poll();
    if (emergency_quickstop_count() != qs)
      break;
    // Split at the mesh grid lines, if leveling is on.
    bed_level_enqueue(&seg);
  }
  return 1;
}
#endif /* NO_ARC_SUPPORT */

/************************************************************************//**

  \brief Processes command stored in global \ref next_target.
  This is where we work out what to actually do with each command we
    receive. All data has already been scaled to integers in gcode_process.
    If you want to add support for a new G or M code, this is the place.


*//*************************************************************************/

void process_gcode_command(void) {
	uint32_t	backup_f;
	// Axis words as parsed, before relative conversion and soft limits.
	// Needed by commands which use X/Y/Z as plain values (M206).
	axes_int32_t raw_axis;

	memcpy(raw_axis, next_target.target.axis, sizeof(raw_axis));

	// convert relative to absolute
	if (next_target.option_all_relative) {
    next_target.target.axis[X] += startpoint.axis[X];
    next_target.target.axis[Y] += startpoint.axis[Y];
    next_target.target.axis[Z] += startpoint.axis[Z];
	}

	// E relative movement.
	// Matches Sprinter's behaviour as of March 2012.
	if (next_target.option_e_relative)
		next_target.target.e_relative = 1;
	else
		next_target.target.e_relative = 0;

	if (next_target.option_all_relative && !next_target.option_e_relative)
		next_target.target.axis[E] += startpoint.axis[E];

	// implement axis limits (shifted by home offsets, M206)
	apply_soft_limits(next_target.target.axis);

	// The GCode documentation was taken from http://reprap.org/wiki/Gcode .

	// T alone selects the tool; M104 T0, M204 T1000 etc. use T otherwise.
	if (next_target.seen_T && ( ! next_target.seen_M || next_target.M == 6)) {
	    //? --- T: Select Tool ---
	    //?
	    //? Example: T1
	    //?
	    //? Select extruder number 1 to build with.  Extruder numbering starts at 0.

	    next_tool = next_target.T;
	}

	if (next_target.seen_G) {
		uint8_t axisSelected = 0;

		switch (next_target.G) {
			case 0:
				//? G0: Rapid Linear Motion
				//?
				//? Example: G0 X12
				//?
				//? In this case move rapidly to X = 12 mm.  In fact, the RepRap firmware uses exactly the same code for rapid as it uses for controlled moves (see G1 below), as - for the RepRap machine - this is just as efficient as not doing so.  (The distinction comes from some old machine tools that used to move faster if the axes were not driven in a straight line.  For them G0 allowed any movement in space to get to the destination as fast as possible.)
				//?
        temp_wait();
				backup_f = next_target.target.F;
				next_target.target.F = settings.max_feedrate[X] * 2L;
				bed_level_enqueue(&next_target.target);
				next_target.target.F = backup_f;
				break;

			case 1:
				//? --- G1: Linear Motion at Feed Rate ---
				//?
				//? Example: G1 X90.6 Y13.8 E22.4
				//?
				//? Go in a straight line from the current (X, Y) point to the point (90.6, 13.8), extruding material as the move happens from the current extruded length to a length of 22.4 mm.
				//?
        temp_wait();
				bed_level_enqueue(&next_target.target);
				break;

#ifndef NO_ARC_SUPPORT
			case 2:
			case 3:
        //? --- G2, G3: Arc clockwise / counterclockwise ---
        //?
        //? Example: G2 X20 Y10 I5 J0 E1.2
        //? Example: G3 X20 Y10 R5 F1800
        //?
        //? Arc in the XY plane from the current position to X, Y. Center by
        //? I, J (offset from the start, always relative) or radius R
        //? (negative: the arc longer than 180 degrees). Z makes a helix,
        //? P adds full turns. Start = end with I/J is a full circle.
        //? PrusaSlicer's and Cura's arc fitting (ArcWelder) produce these.
        //?
        if ( ! arc_move(next_target.G == 2))
          serial_writestr_P(("echo:G2/G3 bad parameters\n"));
        break;
#endif

			case 4:
				//? --- G4: Dwell ---
				//?
				//? Example: G4 P200
				//? Example: G4 S2
				//?
				//? In this case sit still doing nothing for 200 milliseconds (P)
				//? or 2 seconds (S). During delays the state of the machine (for
				//? example the temperatures of its extruders) will still be
				//? preserved and controlled.
				//?
				queue_wait();
				{
					uint32_t ms = next_target.seen_P ? next_target.P : 0;

					if (next_target.seen_S && next_target.S > 0)
						ms += (uint32_t)next_target.S * 1000UL;
					for ( ; ms > 0; ms--) {
						clock_poll();
						delay_ms(1);
					}
				}
				break;

			case 20:
				//? --- G20: Set Units to Inches ---
				//?
				//? Example: G20
				//?
				//? Units from now on are in inches.
				//?
				next_target.option_inches = 1;
				break;

			case 21:
				//? --- G21: Set Units to Millimeters ---
				//?
				//? Example: G21
				//?
				//? Units from now on are in millimeters.  (This is the RepRap default.)
				//?
				next_target.option_inches = 0;
				break;

			case 28:
				//? --- G28: Home ---
				//?
				//? Example: G28
				//?
        //? This causes the RepRap machine to search for its X, Y and Z
        //? endstops. It does so at high speed, so as to get there fast. When
        //? it arrives it backs off slowly until the endstop is released again.
        //? Backing off slowly ensures more accurate positioning.
				//?
        //? If you add axis characters, then just the axes specified will be
        //? seached. Thus
				//?
        //?   G28 X Y72.3
				//?
        //? will zero the X and Y axes, but not Z. Coordinate values are
        //? ignored.
				//?

				queue_wait();

				if (next_target.seen_X) {
					#if defined	X_MIN_PIN
						home_x_negative();
					#elif defined X_MAX_PIN
						home_x_positive();
					#endif
					axisSelected = 1;
				}
				if (next_target.seen_Y) {
					#if defined	Y_MIN_PIN
						home_y_negative();
					#elif defined Y_MAX_PIN
						home_y_positive();
					#endif
					axisSelected = 1;
				}
				if (next_target.seen_Z) {
          #if defined Z_MIN_PIN
            home_z_negative();
          #elif defined Z_MAX_PIN
            home_z_positive();
					#endif
					axisSelected = 1;
				}
				// there's no point in moving E, as E has no endstops

				if (!axisSelected) {
					home();
				}
				sync_target_to_startpoint();
				break;

#ifdef Z_PROBE
      case 29:
        //? --- G29: Bed leveling, probe the mesh ---
        //?
        //? Example: G29
        //?
        //? Probes GRID_POINTS_X x GRID_POINTS_Y points (BLTouch) over the
        //? reachable bed area minus MESH_INSET, stores the mesh and switches
        //? leveling on. Needs X, Y, Z homed (G28). M500 stores the mesh,
        //? M420 V shows it, M420 S0/S1 switches leveling off/on.
        //?
        queue_wait();
        probe_mesh();
        sync_target_to_startpoint();
        break;

      case 30:
        //? --- G30: Probe the bed at one point ---
        //?
        //? Example: G30           at the current probe position
        //? Example: G30 X100 Y80  with the probe at X100 Y80
        //?
        //? Answers "Bed X: 100.000 Y: 80.000 Z: 0.052": bed height there.
        //?
        {
          int32_t px = next_target.seen_X ? raw_axis[X]
                                          : startpoint.axis[X] + probe_offset[X];
          int32_t py = next_target.seen_Y ? raw_axis[Y]
                                          : startpoint.axis[Y] + probe_offset[Y];
          int32_t bed_z;

          queue_wait();
          probe_single(px, py, &bed_z);
          sync_target_to_startpoint();
        }
        break;
#endif /* Z_PROBE */

			case 90:
				//? --- G90: Set to Absolute Positioning ---
				//?
				//? Example: G90
				//?
				//? All coordinates from now on are absolute relative to the origin
				//? of the machine. This is the RepRap default.
				//?
				//? If you ever want to switch back and forth between relative and
				//? absolute movement keep in mind, X, Y and Z follow the machine's
				//? coordinate system while E doesn't change it's position in the
				//? coordinate system on relative movements.
				//?

				// No wait_queue() needed.
				next_target.option_all_relative = 0;
				break;

			case 91:
				//? --- G91: Set to Relative Positioning ---
				//?
				//? Example: G91
				//?
				//? All coordinates from now on are relative to the last position.
				//?

				// No wait_queue() needed.
				next_target.option_all_relative = 1;
				break;

			case 92:
				//? --- G92: Set Position ---
				//?
				//? Example: G92 X10 E90
				//?
				//? Allows programming of absolute zero point, by reseting the current position to the values specified.  This would set the machine's X coordinate to 10, and the extrude coordinate to 90. No physical motion will occur.
				//?

				queue_wait();

				if (next_target.seen_X) {
          startpoint.axis[X] = next_target.target.axis[X];
					axisSelected = 1;
				}
				if (next_target.seen_Y) {
          startpoint.axis[Y] = next_target.target.axis[Y];
					axisSelected = 1;
				}
				if (next_target.seen_Z) {
          startpoint.axis[Z] = next_target.target.axis[Z];
					axisSelected = 1;
				}
				if (next_target.seen_E) {
          startpoint.axis[E] = next_target.target.axis[E];
					axisSelected = 1;
				}

				if (axisSelected == 0) {
          startpoint.axis[X] = next_target.target.axis[X] =
          startpoint.axis[Y] = next_target.target.axis[Y] =
          startpoint.axis[Z] = next_target.target.axis[Z] =
          startpoint.axis[E] = next_target.target.axis[E] = 0;
				}

				dda_new_startpoint();
				break;

			case 161:
				//? --- G161: Home negative ---
				//?
				//? Find the minimum limit of the specified axes by searching for the limit switch.
				//?
        #if defined X_MIN_PIN
          if (next_target.seen_X)
            home_x_negative();
        #endif
        #if defined Y_MIN_PIN
          if (next_target.seen_Y)
            home_y_negative();
        #endif
        #if defined Z_MIN_PIN
          if (next_target.seen_Z)
            home_z_negative();
        #endif
				break;

			case 162:
				//? --- G162: Home positive ---
				//?
				//? Find the maximum limit of the specified axes by searching for the limit switch.
				//?
        #if defined X_MAX_PIN
          if (next_target.seen_X)
            home_x_positive();
        #endif
        #if defined Y_MAX_PIN
          if (next_target.seen_Y)
            home_y_positive();
        #endif
        #if defined Z_MAX_PIN
          if (next_target.seen_Z)
            home_z_positive();
        #endif
				break;

				// unknown gcode: spit an error
			default:
				sersendf_P(("echo:Unknown command: \"G%d\"\n"), next_target.G);
				return;
		}
	}
	else if (next_target.seen_M) {
		uint8_t i;

		switch (next_target.M) {
			case 0:
				//? --- M0: machine stop ---
				//?
				//? Example: M0
				//?
				//? http://linuxcnc.org/handbook/RS274NGC_3/RS274NGC_33a.html#1002379
				//? Unimplemented, especially the restart after the stop. Fall trough to M2.
				//?

			case 2:
				//? --- M2: program end ---
				//?
				//? Example: M2
				//?
				//? http://linuxcnc.org/handbook/RS274NGC_3/RS274NGC_33a.html#1002379
				//?
				queue_wait();
				for (i = 0; i < NUM_TEMP_SENSORS; i++)
					temp_set((temp_sensor_t)i, 0);
				power_off();
        serial_writestr_P(("\nstop\n"));
				break;

      case 80:
        //? --- M80: power supply on ---
        //?
        //? Example: M80
        //?
        //? Switches the power supply on (PS_ON_PIN, PS_MOSFET_PIN) and waits
        //? until it's up. Heating and moving do this by themselves, M80 is
        //? for hosts which want the supply on early (OctoPrint PSU Control).
        power_on();
        break;

      case 81:
        //? --- M81: power supply off ---
        //?
        //? Example: M81
        //?
        //? Waits for the queued moves, switches all heaters and fans off,
        //? disables the steppers and switches the power supply off.
        queue_wait();
        for (i = 0; i < NUM_TEMP_SENSORS; i++)
          temp_set((temp_sensor_t)i, 0);
        heater_all_off();
        power_off();
        break;

      case 17:
        //? --- M17: Enable steppers ---
        //?
        //? Example: M17
        //?
        //? Enables all stepper drivers (and the power supply).
        //?
        steppers_enable_all();
        break;

      case 18:
      case 84:
        //? --- M18, M84: Disable steppers / set idle timeout ---
        //?
        //? Example: M84       disable all steppers after moves completed
        //? Example: M84 S120  disable steppers after 120 s idle, S0 = never
        //?
        //? Heaters are not affected (Marlin behaviour; Teacup's old M84
        //? was an alias for M2).
        //?
        if (next_target.seen_S) {
          steppers_set_idle_timeout((uint16_t)next_target.S);
        }
        else {
          queue_wait();
          steppers_disable_all();
        }
        break;

			case 6:
				//? --- M6: tool change ---
				//?
				//? Undocumented.
				tool = next_tool;
				break;

      #ifdef SD
      case 20:
        //? --- M20: list SD card. ---
        sd_list("/");
        break;

      case 21:
        //? --- M21: initialise SD card. ---
        //?
        //? Has to be done before doing any other operation, including M20.
        sd_mount();
        break;

      case 22:
        //? --- M22: release SD card. ---
        //?
        //? Not mandatory. Just removing the card is fine, but results in
        //? odd behaviour when trying to read from the card anyways. M22
        //? makes also sure SD card printing is disabled, even with the card
        //? inserted.
        sd_unmount();
        break;

      #ifdef SD_FLASH
      case 28:
        //? --- M28: Start writing a file (SPI flash) ---
        //?
        //? Example: M28 PART.GCO
        //?
        //? The following lines are stored in the file instead of being
        //? executed, until M29. Pronterface: "Upload to SD".
        //?
        sd_start_write(gcode_str_buf);
        break;

      case 29:
        //? --- M29: Stop writing the file ---
        sd_end_write();
        break;

      case 30:
        //? --- M30: Delete a file ---
        //?
        //? Example: M30 PART.GCO
        //?
        sd_delete(gcode_str_buf);
        break;

      case 9002:
        //? --- M9002: Delete all files in SPI flash (Teacup specific) ---
        //?
        sd_format();
        break;
      #endif /* SD_FLASH */

      case 23:
        //? --- M23: select file. ---
        //?
        //? This opens a file for reading. This file is valid up to M22 or up
        //? to the next M23.
        sd_open(gcode_str_buf);
        break;

      case 24:
        //? --- M24: start/resume SD print. ---
        //?
        //? This makes the SD card available as a G-code source. File is the
        //? one selected with M23.
        gcode_sources |= GCODE_SOURCE_SD;
        break;

      case 25:
        //? --- M25: pause SD print. ---
        //?
        //? This removes the SD card from the bitfield of available G-code
        //? sources. The file is kept open. The position inside the file
        //? is kept as well, to allow resuming.
        gcode_sources &= (uint8_t)~GCODE_SOURCE_SD;
        break;
      #endif /* SD */

			case 82:
				//? --- M82 - Set E codes absolute ---
				//?
				//? This is the default and overrides G90/G91.
				//? M82/M83 is not documented in the RepRap wiki, behaviour
				//? was taken from Sprinter as of March 2012.
				//?
				//? While E does relative movements, it doesn't change its
				//? position in the coordinate system. See also comment on G90.
				//?

				// No wait_queue() needed.
				next_target.option_e_relative = 0;
				break;

			case 83:
				//? --- M83 - Set E codes relative ---
				//?
				//? Counterpart to M82.
				//?

				// No wait_queue() needed.
				next_target.option_e_relative = 1;
				break;

			// M3/M101- extruder on
			case 3:
			case 101:
				//? --- M101: extruder on ---
				//?
				//? Undocumented.
        temp_wait();
				#ifdef DC_EXTRUDER
					heater_set(DC_EXTRUDER, DC_EXTRUDER_PWM);
				#endif
				break;

			// M5/M103- extruder off
			case 5:
			case 103:
				//? --- M103: extruder off ---
				//?
				//? Undocumented.
				#ifdef DC_EXTRUDER
					heater_set(DC_EXTRUDER, 0);
				#endif
				break;

			case 104:
				//? --- M104: Set Extruder Temperature (Fast) ---
				//?
				//? Example: M104 S190
				//?
        //? Set the temperature of the current extruder to 190<sup>o</sup>C
        //? and return control to the host immediately (''i.e.'' before that
        //? temperature has been reached by the extruder). For waiting, see M116.
        //?
        //? Teacup supports an optional P parameter as a zero-based temperature
        //? sensor index to address (e.g. M104 P1 S100 will set the temperature
        //? of the heater connected to the second temperature sensor rather
        //? than the extruder temperature).
        //?
				if ( ! next_target.seen_S)
					break;
        if ( ! next_target.seen_P)
          #ifdef HEATER_EXTRUDER
            next_target.P = TEMP_SENSOR_extruder;
          #else
            next_target.P = 0;
          #endif
				temp_set((temp_sensor_t)next_target.P, next_target.S);
				break;

			case 109:
				//? --- M109: Set extruder temperature and wait ---
				//?
				//? Example: M109 S200  heat, wait only if heating is needed
				//? Example: M109 R150  set and wait for heating or cooling
				//?
				//? Waits until the temperature stayed within TEMP_HYSTERESIS for
				//? TEMP_RESIDENCY_TIME. M108 cancels the wait. Optional P selects
				//? the temperature sensor.
				//?
				#ifdef HEATER_EXTRUDER
					if ( ! next_target.seen_P)
						next_target.P = TEMP_SENSOR_extruder;
					if (next_target.seen_S || next_target.seen_R) {
						temp_set((temp_sensor_t)next_target.P,
						         next_target.seen_S ? next_target.S : next_target.R);
						temp_wait_sensor((temp_sensor_t)next_target.P,
						                 next_target.seen_S ? 1 : 0);
					}
				#endif
				break;

			case 190:
				//? --- M190: Set bed temperature and wait ---
				//?
				//? Example: M190 S60  heat, wait only if heating is needed
				//? Example: M190 R40  set and wait for heating or cooling
				//?
				#ifdef HEATER_BED
					if (next_target.seen_S || next_target.seen_R) {
						temp_set(TEMP_SENSOR_bed,
						         next_target.seen_S ? next_target.S : next_target.R);
						temp_wait_sensor(TEMP_SENSOR_bed, next_target.seen_S ? 1 : 0);
					}
				#endif
				break;

			case 105:
        //? --- M105: Get Temperature(s) ---
				//?
				//? Example: M105
				//?
        //? Request the temperature of the current extruder and the build base
        //? in degrees Celsius. For example, the line sent to the host in
        //? response to this command looks like
				//?
				//? <tt>ok T:201 B:117</tt>
				//?
        //? Teacup supports an optional P parameter as a zero-based temperature
        //? sensor index to address.
				//?
				#ifdef ENFORCE_ORDER
					queue_wait();
				#endif
				if ( ! next_target.seen_P)
					next_target.P = TEMP_SENSOR_none;
				// Marlin style: the answer goes into the "ok" line.
				if (gcode_active & GCODE_SOURCE_SERIAL) {
					serial_writestr("ok ");
					gcode_ok_sent = 1;
				}
				temp_print((temp_sensor_t)next_target.P);
				break;

			case 7:
			case 106:
				//? --- M106: Set Fan Speed / Set Device Power ---
				//?
				//? Example: M106 S120
				//?
				//? Control the cooling fan (if any).
				//?
        //? Teacup supports an optional P parameter as a zero-based heater
        //? index to address. The heater index can differ from the temperature
        //? sensor index, see config.h.

				#ifdef ENFORCE_ORDER
					// wait for all moves to complete
					queue_wait();
				#endif
        if ( ! next_target.seen_P)
          #ifdef HEATER_FAN
            next_target.P = HEATER_FAN;
          #else
            next_target.P = 0;
          #endif
				if ( ! next_target.seen_S)
					break;
        heater_set((heater_t)next_target.P, next_target.S);
				break;

			case 107:
				//? --- M107: Fan off ---
				//?
				//? Example: M107
				//?
				//? Same as M106 S0. Optional P selects the heater (device).
				//?
				if ( ! next_target.seen_P)
					#ifdef HEATER_FAN
						next_target.P = HEATER_FAN;
					#else
						next_target.P = 0;
					#endif
				heater_set((heater_t)next_target.P, 0);
				break;

			case 110:
				//? --- M110: Set Current Line Number ---
				//?
				//? Example: N123 M110
				//?
				//? Set the current line number to 123.  Thus the expected next line after this command will be 124.
				//? "N5 M110 N123" sets it to 123, too (parameter wins).
				//? Handled when the line is received, see gcode_queue.c.
				//?
				break;

			case 113:
				//? --- M113: Host keepalive ---
				//?
				//? Example: M113 S2
				//?
				//? While a command takes long, send "busy: processing" every S
				//? seconds. S0 disables it. Without S, report the interval.
				//?
				if (next_target.seen_S)
					gcode_queue_set_keepalive((uint8_t)next_target.S);
				else
					sersendf_P(("echo:M113 S%su\n"), gcode_queue_get_keepalive());
				break;

      #ifdef DEBUG
			case 111:
				//? --- M111: Set Debug Level ---
				//?
				//? Example: M111 S6
				//?
				//? Set the level of debugging information transmitted back to the host to level 6.  The level is the OR of three bits:
				//?
				//? <Pre>
				//? #define         DEBUG_PID       1
				//? #define         DEBUG_DDA       2
				//? #define         DEBUG_POSITION  4
				//? </pre>
				//?
				//? This command is only available in DEBUG builds of Teacup.

				if ( ! next_target.seen_S)
					break;
				debug_flags = next_target.S;
				break;
      #endif /* DEBUG */

      case 112:
        //? --- M112: Emergency Stop ---
        //?
        //? Example: M112
        //?
        //? Any moves in progress are immediately terminated, then the printer
        //? shuts down. All motors and heaters are turned off. Restart with
        //? M999 or the reset button.
        //?
        //? Received over serial, M112 is executed by the emergency parser
        //? already (src/emergency_parser.c). This covers other sources,
        //? e.g. an SD card file.
        //?
        printer_kill("Emergency stop (M112)", KILL_NO_ID);
        // printer_kill() doesn't return, no 'break' (IAR: unreachable code).

      case 108:
        //? --- M108: Cancel heating ---
        //?
        //? Stops waiting for temperatures (M116 and moves after it).
        //? Executed by the emergency parser when it arrives, so nothing to
        //? do here. From an SD card it acts here.
        temp_cancel_wait();
        break;

      case 410:
        //? --- M410: Quickstop ---
        //?
        //? Stops all moves immediately and drops the move queue. The current
        //? position is kept (steps are counted), but the abrupt stop may make
        //? steppers skip. Executed by the emergency parser when received over
        //? serial; from other sources it waits for the queue.
        break;

      case 999:
        //? --- M999: Restart after a halt ---
        //?
        //? In the halted state after an error (see kill.c) M999 reboots the
        //? controller. Otherwise nothing to do.
        break;

			case 114:
				//? --- M114: Get Current Position ---
				//?
				//? Example: M114
				//?
				//? This causes the RepRap machine to report its current X, Y, Z and E coordinates to the host.
				//?
				//? For example, the machine returns a string such as:
				//?
				//? <tt>X:10.000 Y:0.000 Z:0.000 E:0.000 Count X:400 Y:0 Z:0</tt>
				//?
				#ifdef ENFORCE_ORDER
					// wait for all moves to complete
					queue_wait();
				#endif
				update_current_position();
				// Marlin format, parsed by OctoPrint & co. Count = motor steps.
				sersendf_P(("X:%lq Y:%lq Z:%lq E:%lq Count X:%ld Y:%ld Z:%ld\n"),
                        current_position.axis[X], current_position.axis[Y],
                        current_position.axis[Z], current_position.axis[E],
                        um_to_steps(current_position.axis[X], X),
                        um_to_steps(current_position.axis[Y], Y),
                        um_to_steps(current_position.axis[Z], Z));

        if (mb_tail_dda != NULL) {
          if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
            DDA *dda = mb_tail_dda;
            // No preprocessor directives inside macro arguments (not portable).
            #ifdef ACCELERATION_REPRAP
              uint32_t c = dda->end_c;
            #else
              uint32_t c = dda->c;
            #endif

            sersendf_P(("Endpoint: X:%ld,Y:%ld,Z:%ld,E:%ld,F:%lu,c:%lu}\n"),
                       dda->endpoint.axis[X], dda->endpoint.axis[Y],
                       dda->endpoint.axis[Z], dda->endpoint.axis[E],
                       dda->endpoint.F, c);
          }
          print_queue();
        }

				break;

			case 115:
				//? --- M115: Get Firmware Version and Capabilities ---
				//?
				//? Example: M115
				//?
				//? Request the Firmware Version and Capabilities of the current microcontroller
				//? The details are returned to the host computer as key:value pairs separated by spaces and terminated with a linefeed.
				//?
				//? sample data from firmware:
				//?  FIRMWARE_NAME:Teacup FIRMWARE_URL:http://github.com/traumflug/Teacup_Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1 TEMP_SENSOR_COUNT:1 HEATER_COUNT:1
				//?

        sersendf_P(("FIRMWARE_NAME:Teacup "
                        "FIRMWARE_URL:http://github.com/traumflug/Teacup_Firmware/ "
                        "PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:%d "
                        "TEMP_SENSOR_COUNT:%d HEATER_COUNT:%d\n"
                        "Cap:AUTOREPORT_TEMP:1\n"
                        "Cap:EMERGENCY_PARSER:1\n"
                        "Cap:BUSY_PROTOCOL:1\n"
                        "Cap:HOST_ACTION_COMMANDS:1\n"
                        "Cap:PROMPT_SUPPORT:1\n"),
                        1, NUM_TEMP_SENSORS, NUM_HEATERS);
        #ifdef Z_PROBE
          serial_writestr("Cap:Z_PROBE:1\n");
        #endif
        #ifdef BED_LEVELING
          serial_writestr("Cap:LEVELING_DATA:1\n");
        #endif
        #ifdef BABYSTEPPING
          serial_writestr("Cap:BABYSTEPPING:1\n");
        #endif
        #ifdef FILAMENT_RUNOUT_PIN
          serial_writestr("Cap:FILAMENT_RUNOUT:1\n");
        #endif
				break;

			case 116:
				//? --- M116: Wait ---
				//?
				//? Example: M116
				//?
				//? Wait for temperatures and other slowly-changing variables to arrive at their set values.
        temp_set_wait();
				break;

      case 117:
        //? --- M117: Set status message ---
        //?
        //? Example: M117 Printing part 1
        //?
        //? Shown on the display, if any. Up to 31 characters.
        //?
        status_set_message(gcode_msg_buf);
        break;

      case 73:
        //? --- M73: Set/get print progress ---
        //?
        //? Example: M73 P42 R17   42 %, 17 minutes remaining
        //? Example: M73          report
        //?
        if (next_target.seen_P || next_target.seen_R) {
          status_set_progress(next_target.seen_P ? (uint8_t)next_target.P
                                                 : status_get_progress(),
                              next_target.seen_R ? next_target.R
                                                 : status_get_remaining());
        }
        else if ( ! next_target.seen_S) {
          // "M73 Q17 S12" (Prusa silent mode progress) is ignored.
          sersendf_P(("echo:Progress: %su%%, remaining %ld min\n"),
                     status_get_progress(), status_get_remaining());
        }
        break;

      case 400:
        //? --- M400: Wait for moves to finish ---
        //?
        //? Example: M400
        //?
        queue_wait();
        break;

      case 206:
        //? --- M206: Set home offsets ---
        //?
        //? Example: M206 X10 Z-0.2
        //?
        //? The offset shifts the coordinate system: after homing, the axis
        //? reports its home position plus the offset. Soft limits (X_MIN
        //? etc.) move along. Without parameters: report the offsets.
        //? Not stored permanently yet.
        //?
        if (next_target.seen_X || next_target.seen_Y || next_target.seen_Z) {
          enum axis_e ax;
          uint8_t seen[3];

          seen[X] = next_target.seen_X;
          seen[Y] = next_target.seen_Y;
          seen[Z] = next_target.seen_Z;

          queue_wait();
          for (ax = X; ax <= Z; ax++) {
            if ( ! seen[ax])
              continue;
            // The parser stored the offset as a target coordinate. Undo
            // that, so the next move without this axis doesn't go there.
            next_target.target.axis[ax] =
              next_target.option_all_relative ? 0 : startpoint.axis[ax];
            home_set_offset(ax, raw_axis[ax]);
          }
        }
        else {
          sersendf_P(("echo:M206 X%lq Y%lq Z%lq\n"),
                     home_offset[X], home_offset[Y], home_offset[Z]);
        }
        break;

      case 428:
        //? --- M428: Set home offsets from current position ---
        //?
        //? Example: M428
        //?
        //? Makes the current position the home position (zero, or X_MIN etc.
        //? if defined) by adjusting the home offsets of X, Y and Z.
        //?
        {
          enum axis_e ax;

          queue_wait();
          for (ax = X; ax <= Z; ax++) {
            int32_t base = 0;

            #ifdef X_MIN
              if (ax == X) base = (int32_t)(X_MIN * 1000.);
            #endif
            #ifdef Y_MIN
              if (ax == Y) base = (int32_t)(Y_MIN * 1000.);
            #endif
            #ifdef Z_MIN
              if (ax == Z) base = (int32_t)(Z_MIN * 1000.);
            #endif
            home_set_offset(ax, home_offset[ax] + base - startpoint.axis[ax]);
          }
          sersendf_P(("echo:M206 X%lq Y%lq Z%lq\n"),
                     home_offset[X], home_offset[Y], home_offset[Z]);
        }
        break;

      case 300:
        //? --- M300: Beep ---
        //?
        //? Example: M300 S1000 P200   1 kHz for 200 ms
        //?
        //? Needs BEEPER_PIN, otherwise only waits. Max. 5 s.
        //?
        beeper_tone(next_target.seen_S ? (uint16_t)next_target.S : 260,
                    next_target.seen_P ? next_target.P : 1000);
        break;

      #ifdef SD
      case 27:
        //? --- M27: Report SD print status ---
        //?
        //? Example: M27
        //?
        //? Answers "SD printing byte 1234/56789" or "Not SD printing".
        //?
        sd_report_status();
        break;

      case 26:
        //? --- M26: Set SD position ---
        //?
        //? Example: M26 S1234
        //?
        //? Continue reading the open file at byte 1234.
        //?
        if (next_target.seen_S)
          sd_seek((uint32_t)next_target.S);
        break;
      #endif /* SD */

      case 92:
      case 201:
      case 203:
      case 205:
        //? --- M92: Set steps per unit ---
        //? Example: M92 X160 Y160 Z8000 E1672
        //? --- M201: Set max. acceleration per axis, mm/s^2 ---
        //? Example: M201 X1000 Y1000 Z50 E10000
        //? --- M203: Set max. feedrate per axis, mm/s ---
        //? Example: M203 X150 Y150 Z4 E25
        //? --- M205: Set jerk per axis, mm/s ---
        //? Example: M205 X20 Y20 Z0.4 E5
        //?
        //? Marlin units. Without parameters: report (see M503). Changes are
        //? lost at reset unless stored with M500. Inch mode is not supported
        //? for these commands.
        //?
        {
          uint8_t seen[4], any = 0;
          enum axis_e a;

          seen_axes(seen);
          if (next_target.M == 92 && (seen[X] || seen[Y] || seen[Z] || seen[E]))
            queue_wait();             // Position is kept in steps.

          for (a = X; a < AXIS_COUNT; a++) {
            int32_t v = raw_axis[a];  // Value * 1000.

            if ( ! seen[a])
              continue;
            restore_axis_word(a);
            any = 1;
            if (next_target.M == 92) {
              // steps/mm * 1000 = steps/m.
              if (v < 20 || v > 40960000L) {
                serial_writestr("echo:M92 value out of range\n");
                continue;
              }
              settings.steps_per_m[a] = (uint32_t)v;
            }
            else if (next_target.M == 201) {
              if (v >= 1000)
                settings.max_accel[a] = (uint32_t)(v / 1000);
            }
            else if (next_target.M == 203) {
              if (v > 0)
                settings.max_feedrate[a] = (uint32_t)v * 3 / 50;  // mm/s -> mm/min
            }
            else {  // 205
              if (v >= 0)
                settings.max_jerk[a] = (uint32_t)v * 3 / 50;
            }
          }
          if (any) {
            settings_apply();
            if (next_target.M == 92)
              dda_new_startpoint();   // Same position, new step count.
          }
          else {
            settings_report();
          }
        }
        break;

      case 204:
        //? --- M204: Set acceleration ---
        //?
        //? Example: M204 S1000   (also P, for Marlin compatibility)
        //?
        //? Teacup has one acceleration for printing, retracting and travel.
        //? The acceleration of a move is the lower of this and the M201
        //? value of its fastest axis.
        //?
        if (next_target.seen_S || next_target.seen_P) {
          uint32_t acc = next_target.seen_S ? (uint32_t)next_target.S
                                            : (uint32_t)next_target.P;
          if (acc >= 1) {
            settings.acceleration = acc;
            settings_apply();
          }
        }
        else {
          settings_report();
        }
        break;

      case 301:
      case 304:
        //? --- M301: Set hotend PID, M304: Set bed PID ---
        //?
        //? Example: M301 P22.2 I1.08 D114
        //?
        //? Marlin units (Kp, Ki and Kd per second). Converted to Teacup's
        //? internal units: P * 256, I * 64, D * 128, I limit for the full
        //? output range. Without parameters: report.
        //?
        {
          heater_t h = (heater_t)NUM_HEATERS;

          #ifdef HEATER_EXTRUDER
            if (next_target.M == 301)
              h = HEATER_EXTRUDER;
          #endif
          #ifdef HEATER_BED
            if (next_target.M == 304)
              h = HEATER_BED;
          #endif
          if (h >= NUM_HEATERS)
            break;

          if (next_target.seen_P)
            pid_set_p(h, (int32_t)(((int64_t)next_target.P_milli * 256) / 1000));
          if (next_target.seen_I) {
            int32_t i = (int32_t)(((int64_t)next_target.I_milli * 64) / 1000);

            pid_set_i(h, i);
            pid_set_i_limit(h, i ? 255L * 1024 / i : 32767);
          }
          if (next_target.seen_D)
            pid_set_d(h, (int32_t)(((int64_t)next_target.D_milli * 128) / 1000));
          if ( ! next_target.seen_P && ! next_target.seen_I && ! next_target.seen_D)
            settings_report();
        }
        break;

      case 303:
        //? --- M303: PID autotune ---
        //?
        //? Example: M303 E0 S200 C8 U1
        //?
        //? E0 hotend, E-1 bed. S target temperature (C), C cycles (3..20,
        //? default 5), U1 applies the result (store with M500). M108
        //? aborts. The printer should be idle and at room temperature.
        //?
        {
          int32_t e = next_target.seen_E ? raw_axis[E] / 1000 : 0;
          heater_t h = (heater_t)NUM_HEATERS;
          temp_sensor_t sensor = TEMP_SENSOR_none;

          if (next_target.seen_E)
            restore_axis_word(E);
          #ifdef HEATER_EXTRUDER
            if (e == 0) {
              h = HEATER_EXTRUDER;
              sensor = TEMP_SENSOR_extruder;
            }
          #endif
          #ifdef HEATER_BED
            if (e == -1) {
              h = HEATER_BED;
              sensor = TEMP_SENSOR_bed;
            }
          #endif
          if (h >= NUM_HEATERS) {
            serial_writestr("echo:M303: no such heater\n");
            break;
          }
          queue_wait();
          pid_autotune(h, sensor,
                       next_target.seen_S ? (uint16_t)next_target.S : 150,
                       next_target.seen_C ? (uint8_t)next_target.C : 5,
                       next_target.seen_U ? next_target.U : 0);
        }
        break;

      case 500:
        //? --- M500: Store settings in Flash ---
        //?
        //? Internal Flash: waits for moves to finish, erasing stalls the CPU
        //? for up to 2 s, so don't use it while printing.
        //? SPI flash (SPI_FLASH_SETTINGS): no stall, works while printing.
        //?
        if ( ! flash_store_on_spi())
          queue_wait();
        if ( ! settings_save())
          serial_writestr("echo:Error storing settings\n");
        break;

      case 501:
      case 502:
        //? --- M501: Load settings from Flash, M502: Default settings ---
        //?
        {
          int32_t old[3];
          uint8_t i;

          queue_wait();
          for (i = 0; i < 3; i++)
            old[i] = home_offset[i];
          if (next_target.M == 501) {
            if ( ! settings_load())
              serial_writestr("echo:No stored settings\n");
          }
          else {
            settings_defaults();
            serial_writestr("echo:Hardcoded Default Settings Loaded\n");
          }
          settings_apply();
          dda_new_startpoint();
          reapply_home_offsets(old);
        }
        break;

      case 503:
        //? --- M503: Report settings ---
        //?
        settings_report();
        break;

      case 600:
        //? --- M600: Filament change ---
        //?
        //? Example: M600
        //? Example: M600 E2 Z20 X10 Y170 U60 L40
        //?
        //? Retract E, lift Z (relative), park at X/Y (only if homed), unload
        //? U mm, then wait for the user: insert the new filament and send
        //? M108 (Pronterface) or press "Continue" (OctoPrint, M876 S0).
        //? Loads and purges L mm, goes back and continues. Defaults from
        //? FILAMENT_CHANGE_* in the printer config. The hotend heater goes
        //? off after FILAMENT_CHANGE_NOZZLE_TIMEOUT s of waiting and is
        //? heated up again before loading. Also run by the runout sensor.
        //?
        {
          filament_change_t fc;

          filament_change_defaults(&fc);
          if (next_target.seen_E)
            fc.retract = labs(raw_axis[E]);
          if (next_target.seen_Z)
            fc.lift = labs(raw_axis[Z]);
          if (next_target.seen_X)
            fc.park_x = raw_axis[X];
          if (next_target.seen_Y)
            fc.park_y = raw_axis[Y];
          if (next_target.seen_U)
            fc.unload = labs(next_target.U_milli);
          if (next_target.seen_L)
            fc.load = labs(next_target.L_milli);
          filament_change(&fc);
        }
        break;

      case 876:
        //? --- M876: Host prompt response ---
        //?
        //? Example: M876 S0
        //?
        //? "Continue" in OctoPrint's prompt of M600. Acts in the emergency
        //? parser already; M876 P1 (host announces prompt support) does
        //? nothing.
        //?
        break;

      #ifdef FILAMENT_RUNOUT_PIN
      case 412:
        //? --- M412: Filament runout sensor ---
        //?
        //? Example: M412         report
        //? Example: M412 S0      off, S1 on
        //? Example: M412 D25     pause after 25 mm more filament
        //? Example: M412 R       forget a detected runout
        //?
        //? Stored with M500.
        //?
        if (next_target.seen_S || next_target.seen_D)
          filament_runout_set(next_target.seen_S ? (int8_t)(next_target.S ? 1 : 0) : -1,
                              next_target.seen_D ? labs(next_target.D_milli) : -1);
        if (next_target.seen_R)
          filament_runout_reset();
        if ( ! next_target.seen_S && ! next_target.seen_D && ! next_target.seen_R)
          filament_runout_report();
        break;
      #endif

      #ifdef BABYSTEPPING
      case 290:
        //? --- M290: Babystep ---
        //?
        //? Example: M290 Z0.02   nozzle 0.02 mm up, now
        //? Example: M290 Z-0.05  0.05 mm down
        //? Example: M290         report the Z offset
        //?
        //? Moves Z right away, also in the middle of a print (first layer
        //? adjustment). The logical coordinates stay, the total is a Z
        //? offset (+-BABYSTEP_LIMIT), stored with M500.
        //?
        if (next_target.seen_Z) {
          int32_t want = raw_axis[Z];
          int32_t done = babystep_add(want);

          if (done != want)
            serial_writestr("echo:Z offset limited\n");
        }
        sersendf_P(("echo:Z offset %lq\n"), babystep_offset());
        break;
      #endif

      #ifdef BED_LEVELING
      case 420:
        //? --- M420: Bed leveling state ---
        //?
        //? Example: M420 S1      leveling on (needs a mesh, G29 or M421)
        //? Example: M420 S0      off
        //? Example: M420 Z10     fade out over 10 mm (Z0: no fade)
        //? Example: M420         report state and mesh (also M420 V)
        //?
        //? The motors don't move, the logical Z changes instead (like
        //? Marlin). Stored with M500.
        //?
        if (next_target.seen_Z)
          bed_level_set_fade(raw_axis[Z]);
        if (next_target.seen_S) {
          if ( ! bed_level_set_active((uint8_t)(next_target.S ? 1 : 0)) &&
              next_target.S)
            serial_writestr("echo:No mesh, leveling stays off\n");
        }
        if ( ! next_target.seen_S && ! next_target.seen_Z)
          bed_level_report();
        sync_target_to_startpoint();
        break;

      case 421:
        //? --- M421: Set a mesh point ---
        //?
        //? Example: M421 I1 J2 Z0.05
        //?
        //? Point I (X index), J (Y index) of the existing mesh gets bed
        //? height Z. Without a mesh (no G29 yet), a GRID_POINTS_X x
        //? GRID_POINTS_Y mesh over the bed minus MESH_INSET is created
        //? first, all points 0. So the mesh can be measured by hand.
        //?
        if ( ! next_target.seen_I || ! next_target.seen_J || ! next_target.seen_Z) {
          serial_writestr("echo:M421 needs I, J and Z\n");
          break;
        }
        if (mesh.nx == 0) {
          int32_t inset = (int32_t)(MESH_INSET * 1000.);
          int32_t x0 = (int32_t)(X_MIN * 1000.) + home_offset[X] + inset;
          int32_t x1 = (int32_t)(X_MAX * 1000.) + home_offset[X] - inset;
          int32_t y0 = (int32_t)(Y_MIN * 1000.) + home_offset[Y] + inset;
          int32_t y1 = (int32_t)(Y_MAX * 1000.) + home_offset[Y] - inset;

          bed_level_new(x0, y0, (x1 - x0) / (GRID_POINTS_X - 1),
                        (y1 - y0) / (GRID_POINTS_Y - 1),
                        GRID_POINTS_X, GRID_POINTS_Y);
        }
        if (next_target.I_milli < 0 || next_target.J_milli < 0 ||
            ! bed_level_set_point((uint8_t)(next_target.I_milli / 1000),
                                  (uint8_t)(next_target.J_milli / 1000),
                                  raw_axis[Z]))
          serial_writestr("echo:M421: no such mesh point\n");
        sync_target_to_startpoint();
        break;
      #endif /* BED_LEVELING */

      #ifdef Z_PROBE
      case 851:
        //? --- M851: Z probe offset ---
        //?
        //? Example: M851 X-35 Y-5 Z-1.52
        //?
        //? Position of the probe relative to the nozzle, mm. Z: negative,
        //? how far the nozzle is above the bed when the probe triggers.
        //? Takes effect with the next G28 Z / G29 / G30. Stored with M500.
        //?
        if (next_target.seen_X)
          probe_offset[X] = raw_axis[X];
        if (next_target.seen_Y)
          probe_offset[Y] = raw_axis[Y];
        if (next_target.seen_Z)
          probe_offset[Z] = raw_axis[Z];
        sersendf_P(("echo:Probe Offset X%lq Y%lq Z%lq\n"),
                   probe_offset[X], probe_offset[Y], probe_offset[Z]);
        break;

      case 401:
        //? --- M401: Deploy the probe ---
        probe_deploy();
        break;

      case 402:
        //? --- M402: Stow the probe ---
        probe_stow();
        break;
      #endif /* Z_PROBE */

      #ifdef BLTOUCH_SERVO_PIN
      case 280:
        //? --- M280: Servo position ---
        //?
        //? Example: M280 P0 S10   BLTouch: deploy
        //? Example: M280 P0 S90   stow
        //? Example: M280 P0 S160  alarm release
        //? Example: M280 P0 S120  self test
        //?
        //? S angle 0..180, or a pulse width in us (544..2500). Only servo 0.
        //?
        if (next_target.seen_P && next_target.P != 0) {
          serial_writestr("echo:Servo 0 only\n");
          break;
        }
        if (next_target.seen_S && next_target.S >= 0)
          servo_write_angle((uint16_t)next_target.S);
        else
          sersendf_P(("echo: Servo 0: %u\n"), servo_read_angle());
        break;
      #endif

      case 9001:
        //? --- M9001: Step interrupt statistics (Teacup specific) ---
        //?
        //? Example: M9001     report
        //? Example: M9001 R   reset
        //?
        //? Duration of the step interrupt in CPU cycles (min/avg/max), max.
        //? latency from compare match to handler entry, share of CPU time
        //? since the last reset, number of steps scheduled too late. Use it
        //? to measure optimisations on the real hardware.
        //?
        if (next_target.seen_R) {
          step_stats_reset();
        }
        else {
          // Copy first: one volatile access per statement (IAR Pa082).
          uint32_t n = step_stats.count;
          uint32_t smin = step_stats.min;
          uint32_t smax = step_stats.max;
          uint32_t slat = step_stats.max_latency;
          uint32_t slate = step_stats.late;
          uint32_t spulses = step_stats.pulses;
          uint64_t ssum = step_stats.sum;
          // Elapsed CPU cycles; clock_millis() instead of the cycle counter,
          // which wraps every 51 s at 84 MHz.
          uint64_t elapsed = (uint64_t)(clock_millis() - step_stats.since) *
                             (F_CPU / 1000UL);
          uint32_t avg = n ? (uint32_t)(ssum / n) : 0;
          uint32_t load = elapsed ? (uint32_t)((ssum * 10000ULL) / elapsed) : 0;

          sersendf_P(("echo:Step IRQ: n %lu, cycles min %lu avg %lu max %lu, "
                      "latency max %lu, late %lu, pulses %lu, load %lu.%su%%, "
                      "F_CPU %lu\n"),
                     n, n ? smin : 0, avg, smax, slat, slate, spulses,
                     load / 100, (uint8_t)(load % 100),
                     (uint32_t)F_CPU);
        }
        break;

      case 119:
        //? --- M119: report endstop status ---
        //? Report the current status of the endstops configured in the
        //? firmware to the host.
        power_on();
        endstops_on();
        delay_ms(10); // allow the signal to stabilize
        {
          #if ! (defined(X_MIN_PIN) || defined(X_MAX_PIN) || \
                 defined(Y_MIN_PIN) || defined(Y_MAX_PIN) || \
                 defined(Z_MIN_PIN) || defined(Z_MAX_PIN))
            serial_writestr_P(("No endstops defined."));
          #else
            const char* const open = ("open ");
            const char* const triggered = ("triggered ");
          #endif

          #if defined(X_MIN_PIN)
            serial_writestr_P(("x_min:"));
            x_min() ? serial_writestr_P(triggered) : serial_writestr_P(open);
          #endif
          #if defined(X_MAX_PIN)
            serial_writestr_P(("x_max:"));
            x_max() ? serial_writestr_P(triggered) : serial_writestr_P(open);
          #endif
          #if defined(Y_MIN_PIN)
            serial_writestr_P(("y_min:"));
            y_min() ? serial_writestr_P(triggered) : serial_writestr_P(open);
          #endif
          #if defined(Y_MAX_PIN)
            serial_writestr_P(("y_max:"));
            y_max() ? serial_writestr_P(triggered) : serial_writestr_P(open);
          #endif
          #if defined(Z_MIN_PIN)
            serial_writestr_P(("z_min:"));
            z_min() ? serial_writestr_P(triggered) : serial_writestr_P(open);
          #endif
          #if defined(Z_MAX_PIN)
            serial_writestr_P(("z_max:"));
            z_max() ? serial_writestr_P(triggered) : serial_writestr_P(open);
          #endif
          #ifdef FILAMENT_RUNOUT_PIN
            // Marlin: "filament: TRIGGERED" when there's no filament.
            serial_writestr_P(("filament:"));
            (READ(FILAMENT_RUNOUT_PIN) == FILAMENT_RUNOUT_STATE) ?
              serial_writestr_P(triggered) : serial_writestr_P(open);
          #endif
        }
        endstops_off();
        serial_writechar('\n');
        break;

			case 130:
				//? --- M130: heater P factor ---
				//? Undocumented.
			  	//  P factor in counts per degreeC of error
        if ( ! next_target.seen_P)
          #ifdef HEATER_EXTRUDER
            next_target.P = HEATER_EXTRUDER;
          #else
            next_target.P = 0;
          #endif
				if (next_target.seen_S)
					pid_set_p((heater_t)next_target.P, next_target.S);
				break;

			case 131:
				//? --- M131: heater I factor ---
				//? Undocumented.
			  	// I factor in counts per C*s of integrated error
        if ( ! next_target.seen_P)
          #ifdef HEATER_EXTRUDER
            next_target.P = HEATER_EXTRUDER;
          #else
            next_target.P = 0;
          #endif
				if (next_target.seen_S)
					pid_set_i((heater_t)next_target.P, next_target.S);
				break;

			case 132:
				//? --- M132: heater D factor ---
				//? Undocumented.
			  	// D factor in counts per degreesC/second
        if ( ! next_target.seen_P)
          #ifdef HEATER_EXTRUDER
            next_target.P = HEATER_EXTRUDER;
          #else
            next_target.P = 0;
          #endif
				if (next_target.seen_S)
					pid_set_d((heater_t)next_target.P, next_target.S);
				break;

			case 133:
				//? --- M133: heater I limit ---
				//? Undocumented.
        if ( ! next_target.seen_P)
          #ifdef HEATER_EXTRUDER
            next_target.P = HEATER_EXTRUDER;
          #else
            next_target.P = 0;
          #endif
				if (next_target.seen_S)
					pid_set_i_limit((heater_t)next_target.P, next_target.S);
				break;

      // M134 (save PID settings to EEPROM) isn't available, no EEPROM.

      #ifdef DEBUG
			case 136:
				//? --- M136: PRINT PID settings to host ---
				//? Undocumented.
				//? This comand is only available in DEBUG builds.
        if ( ! next_target.seen_P)
          #ifdef HEATER_EXTRUDER
            next_target.P = HEATER_EXTRUDER;
          #else
            next_target.P = 0;
          #endif
				heater_print(next_target.P);
				break;
      #endif /* DEBUG */

			case 140:
				//? --- M140: Set heated bed temperature ---
				//? Undocumented.
				#ifdef	HEATER_BED
					if ( ! next_target.seen_S)
						break;
					temp_set(TEMP_SENSOR_bed, next_target.S);
				#endif
				break;

      case 155:
        //? --- M155: Report Temperature(s) Periodically ---
        //?
        //? Example: M155 Sn
        //?
        //? turns on periodic reporting of the temperatures of the current
        //? extruder and the build base in degrees Celsius. The reporting
        //? interval is given in seconds as the S parameter. Use S0 to disable
        //? periodic temperature reporting. The reporting format is the same
        //? as for M105, except there is no "ok" at the start of each report.
        //? For example, the line sent to the host periodically looks like
        //?
        //? <tt>T:201 B:117</tt>
        //?
        //? Teacup supports an optional P parameter as a zero-based temperature
        //? sensor index to address.
        //?

        // S<period-seconds> is required
        if ( ! next_target.seen_S)
          break;
        #ifdef ENFORCE_ORDER
          queue_wait();
        #endif
        if ( ! next_target.seen_P)
          next_target.P = TEMP_SENSOR_none;
        temp_periodic_config(next_target.S, (temp_sensor_t)next_target.P);
        break;

      case 220:
        //? --- M220: Set speed factor override percentage ---
        if ( ! next_target.seen_S)
          break;
        // Scale 100% = 256
        next_target.target.f_multiplier = (next_target.S * 64 + 12) / 25;
        break;

      case 221:
        //? --- M221: Control the extruders flow ---
        if ( ! next_target.seen_S)
          break;
        // Scale 100% = 256
        next_target.target.e_multiplier = (next_target.S * 64 + 12) / 25;
        break;

      #ifdef DEBUG
			case 240:
				//? --- M240: echo off ---
				//? Disable echo.
				//? This command is only available in DEBUG builds.
				debug_flags &= ~DEBUG_ECHO;
				serial_writestr_P(("Echo off\n"));
				break;

			case 241:
				//? --- M241: echo on ---
				//? Enable echo.
				//? This command is only available in DEBUG builds.
				debug_flags |= DEBUG_ECHO;
				serial_writestr_P(("Echo on\n"));
				break;
      #endif /* DEBUG */

				// unknown mcode: spit an error
			default:
				sersendf_P(("echo:Unknown command: \"M%d\"\n"), next_target.M);
		} // switch (next_target.M)

		// M-commands never set a move target. Axis letters used as values
		// (M84 X Y E, M92 X160, M303 E0, ...) must not end up as the target
		// of the next move without that axis.
		{
			uint8_t seen[4];
			enum axis_e a;

			seen_axes(seen);
			for (a = X; a < AXIS_COUNT; a++)
				if (seen[a])
					restore_axis_word(a);
		}
	} // else if (next_target.seen_M)
} // process_gcode_command()
