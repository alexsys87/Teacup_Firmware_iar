/** \file
  \brief Homing routines
*/

#include "home.h"
#include "bed_leveling.h"
#include "settings.h"

#include <math.h>
#include "dda.h"
#include "dda_queue.h"
#include "pinio.h"
#include "gcode_parse.h"
#include "probe.h"

// Check configuration.
#if defined X_MIN_PIN || defined X_MAX_PIN
  #ifndef SEARCH_FEEDRATE_X
    #error SEARCH_FEEDRATE_X undefined. It should be defined in config.h.
  #endif
  #ifndef ENDSTOP_CLEARANCE_X
    #error ENDSTOP_CLEARANCE_X undefined. It should be defined in config.h.
  #endif
#endif
#if defined Y_MIN_PIN || defined Y_MAX_PIN
  #ifndef SEARCH_FEEDRATE_Y
    #error SEARCH_FEEDRATE_Y undefined. It should be defined in config.h.
  #endif
  #ifndef ENDSTOP_CLEARANCE_Y
    #error ENDSTOP_CLEARANCE_Y undefined. It should be defined in config.h.
  #endif
#endif
#if defined Z_MIN_PIN || defined Z_MAX_PIN
  #ifndef SEARCH_FEEDRATE_Z
    #error SEARCH_FEEDRATE_Z undefined. It should be defined in config.h.
  #endif
  #ifndef ENDSTOP_CLEARANCE_Z
    #error ENDSTOP_CLEARANCE_Z undefined. It should be defined in config.h.
  #endif
#endif

// Calculate feedrates according to clearance and deceleration.
// For a description, see #define ENDSTOP_CLEARANCE_{XYZ} in config.h.
//   s = 1/2 * a * t^2; t = v / a  <==> v = sqrt(2 * a * s))
//   units: / 1000 for um -> mm; * 60 for mm/s -> mm/min
// Acceleration is a runtime setting (M201/M204), so this is calculated
// before each homing.
static uint32_t fast_feedrate_P[3];
static uint32_t search_feedrate_P[3];

static void init_home_feedrates(void) {
  static const uint32_t clearance[3] = {
    ENDSTOP_CLEARANCE_X, ENDSTOP_CLEARANCE_Y, ENDSTOP_CLEARANCE_Z
  };
  static const uint32_t search[3] = {
    SEARCH_FEEDRATE_X, SEARCH_FEEDRATE_Y, SEARCH_FEEDRATE_Z
  };
  enum axis_e a;

  for (a = X; a <= Z; a++) {
    uint32_t fast = (uint32_t)(60. * sqrt(2. * (double)settings_axis_accel(a) *
                                          (double)clearance[a] / 1000.));

    fast_feedrate_P[a] = (fast > search[a]) ? fast : search[a];
    search_feedrate_P[a] = (fast > search[a]) ? search[a] : 0;
  }
}

static void home_axis(enum axis_e n, int8_t dir, enum axis_endstop_e endstop_check);
static void set_axis_home_position(enum axis_e n, int8_t dir);

uint8_t axes_homed = 0;

/**
  Home all axes (G28 without axis letters): in the order of DEFINE_HOMING
  in the printer config, X, Y, Z if there is none. Every axis goes to the
  endstop it has, MIN preferred.
*/
void home(void) {
  uint8_t configured = 0;

  #ifdef DEFINE_HOMING_ACTUAL
    #undef DEFINE_HOMING_ACTUAL
      #define DEFINE_HOMING_ACTUAL(first, second, third, fourth) \
        { \
          home_##first(); \
          home_##second(); \
          home_##third(); \
          home_##fourth(); \
          configured = 1; \
        };
      #include "config_wrapper.h"
    #undef DEFINE_HOMING_ACTUAL
  #endif

  if ( ! configured) {
    #if defined X_MIN_PIN
      home_x_negative();
    #elif defined X_MAX_PIN
      home_x_positive();
    #endif
    #if defined Y_MIN_PIN
      home_y_negative();
    #elif defined Y_MAX_PIN
      home_y_positive();
    #endif
    #if defined Z_MIN_PIN
      home_z_negative();
    #elif defined Z_MAX_PIN
      home_z_positive();
    #endif
  }
}

void home_none(void) {
}

void home_lift_z(void) {
  #if defined Z_HOMING_HEIGHT && (defined Z_MIN_PIN || defined Z_MAX_PIN)
    const int32_t lift = (int32_t)(Z_HOMING_HEIGHT * 1000.);
    TARGET t;

    // Like Marlin: up to the height, never down. With Z not homed the
    // current Z is assumed right (0 after power on), so repeated homing of
    // X or Y doesn't lift again and again.
    queue_wait();
    if (lift <= 0 || startpoint.axis[Z] >= lift)
      return;
    t = startpoint;
    t.axis[Z] = lift;
    t.F = settings.max_feedrate[Z];
    t.f_multiplier = 256;
    enqueue(&t);
    queue_wait();
  #endif
}

/// find X MIN endstop
void home_x_negative(void) {
  #if defined X_MIN_PIN
    home_axis(X, -1, X_MIN_ENDSTOP);
  #endif
}

/// find X_MAX endstop
void home_x_positive(void) {
  #if defined X_MAX_PIN && ! defined X_MAX
    #warning X_MAX_PIN defined, but not X_MAX. home_x_positive() disabled.
  #endif
  #if defined X_MAX_PIN && defined X_MAX
    home_axis(X, 1, X_MAX_ENDSTOP);
  #endif
}

/// fund Y MIN endstop
void home_y_negative(void) {
  #if defined Y_MIN_PIN
    home_axis(Y, -1, Y_MIN_ENDSTOP);
  #endif
}

/// find Y MAX endstop
void home_y_positive(void) {
  #if defined Y_MAX_PIN && ! defined Y_MAX
    #warning Y_MAX_PIN defined, but not Y_MAX. home_y_positive() disabled.
  #endif
  #if defined Y_MAX_PIN && defined Y_MAX
    home_axis(Y, 1, Y_MAX_ENDSTOP);
  #endif
}

/// find Z MIN endstop
void home_z_negative(void) {
  #if defined Z_PROBE
    // Z_MIN is the probe: home with it at a safe XY position.
    probe_home_z();
  #elif defined Z_MIN_PIN
    home_axis(Z, -1, Z_MIN_ENDSTOP);
  #endif
}

/// find Z MAX endstop
void home_z_positive(void) {
  #if defined Z_MAX_PIN && ! defined Z_MAX
    #warning Z_MAX_PIN defined, but not Z_MAX. home_z_positive() disabled.
  #endif
  #if defined Z_MAX_PIN && defined Z_MAX
    home_axis(Z, 1, Z_MAX_ENDSTOP);
  #endif
}

static void home_axis(enum axis_e n, int8_t dir, enum axis_endstop_e endstop_check) {
  TARGET t;
  uint32_t search_feedrate;
  int32_t z_steps;

  // Home in motor coordinates: the Z correction (mesh, Z offset) would
  // otherwise move Z along with X/Y and end up in the Z home position.
  queue_wait();
  zcorr_suspend();
  z_steps = startpoint_steps.axis[Z];
  t = startpoint;

  startpoint.axis[n] = 0;

  init_home_feedrates();

  t.axis[n] = dir * MAX_DELTA_UM;
  t.F = fast_feedrate_P[n];
  enqueue_home(&t, endstop_check, 1);

  search_feedrate = (search_feedrate_P[n]);
  if (search_feedrate) {
    // back off slowly
    t.axis[n] = 0;
    t.F = search_feedrate;
    enqueue_home(&t, endstop_check, 0);
  }

  queue_wait();
  endstops_off();
  set_axis_home_position(n, dir);
  dda_new_startpoint();
  if (n != Z) {
    // Z didn't move: keep its exact motor position (micrometers can't
    // hold every step, the conversion may round).
    startpoint_steps.axis[Z] = z_steps;
  }
  axes_homed |= (uint8_t)(1U << n);
  // Correction back on: logical Z = motor Z - correction at this place.
  zcorr_resume();
}

int32_t home_offset[3];

void home_set_offset(enum axis_e n, int32_t offset_um) {
  int32_t delta;

  if (n > Z)
    return;

  queue_wait();
  delta = offset_um - home_offset[n];
  home_offset[n] = offset_um;

  // Same physical position, new coordinates (like G92).
  startpoint.axis[n] += delta;
  if ( ! next_target.option_all_relative)
    next_target.target.axis[n] += delta;
  dda_new_startpoint();
}

static void set_axis_home_position(enum axis_e n, int8_t dir) {
  int32_t home_position = 0;
  if (dir < 0) {
    if (n == X) {
      #ifdef X_MIN
      home_position = (int32_t)(X_MIN * 1000L);
      #endif
    }
    else if (n == Y) {
      #ifdef Y_MIN
      home_position = (int32_t)(Y_MIN * 1000L);
      #endif
    }
    else if (n == Z) {
      #ifdef Z_MIN
      home_position = (int32_t)(Z_MIN * 1000L);
      #endif
    }
  }
  else {
    if (n == X) {
      #ifdef X_MAX
      home_position = (int32_t)(X_MAX * 1000L);
      #endif
    }
    else if (n == Y) {
      #ifdef Y_MAX
      home_position = (int32_t)(Y_MAX * 1000L);
      #endif
    }
    else if (n == Z) {
      #ifdef Z_MAX
      home_position = (int32_t)(Z_MAX * 1000L);
      #endif
    }
  }
  home_position += home_offset[n];
  startpoint.axis[n] = next_target.target.axis[n] = home_position;
}
