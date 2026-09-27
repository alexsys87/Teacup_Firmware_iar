/** \file

  \brief User interface: status screen and menu, see ui.h.

  Screens are text (display_text_*()), laid out for the size of the
  display: 2 lines (16x2 / 20x2 LCD), 4 lines (20x4 LCD, 128x32 OLED) or
  8 lines (128x64 OLED). display_update() sends what changed.

  The menu (DISPLAY_MENU) runs in ui_tick(), every 10 ms, also while a
  long command (homing, heating) blocks the main loop. It never executes
  commands itself: it queues G-code lines, which the main loop executes
  between commands (ui_execute()), like commands from the host. So values
  set in the menu go the same way as M104, M92 etc. from a host, and
  "Store settings" is M500 (SPI flash or internal Flash).

  Menu structure:
    Main: Status | Resume/discard (power loss) | Continue (filament
          change) | Pause/Resume print, Stop print, Tune (printing) |
          Print from SD, Motion (not printing) | Temperature | Settings
    Tune: speed, flow, hotend, bed, fan, babystep Z, change filament
    Temperature: hotend, bed, fan, preheat PLA / PETG, cooldown
    Motion: home, auto level, align Z, move X/Y/Z (homed axes), steppers
            off
    Settings: steps/mm, feedrates, accelerations, jerk, linear advance,
              probe offset, retract; store, load, defaults

  Buttons: up / down move the selection or change a value (held: repeat,
  after 2 s in steps of ten), OK selects / sets, back (optional, each
  menu has a "Back" item as well) returns / cancels.
*/

#include "ui.h"

#ifdef DISPLAY

#include <string.h>
#include "config_wrapper.h"
#include "clock.h"
#include "temp.h"
#include "heater.h"
#include "dda.h"
#include "gcode_parse.h"
#include "status.h"
#include "sd.h"
#include "print_stats.h"
#include "power_loss.h"
#include "filament.h"
#include "settings.h"
#include "buttons.h"
#include "babystep.h"
#include "probe.h"
#include "dda_queue.h"
#include "emergency_parser.h"
#include "serial.h"
#include "home.h"

#if defined DISPLAY_MENU && ! defined BUTTONS
  #error DISPLAY_MENU needs buttons (BUTTON_UP_BIT, BUTTON_DOWN_BIT, BUTTON_OK_BIT, PCF8574_ADDRESS).
#endif

#ifndef PREHEAT_PLA_HOTEND
  #define PREHEAT_PLA_HOTEND    200
#endif
#ifndef PREHEAT_PLA_BED
  #define PREHEAT_PLA_BED       60
#endif
#ifndef PREHEAT_PETG_HOTEND
  #define PREHEAT_PETG_HOTEND   240
#endif
#ifndef PREHEAT_PETG_BED
  #define PREHEAT_PETG_BED      80
#endif
#define STR_(x) #x
#define STR(x)  STR_(x)
#ifndef X_MAX
  #define X_MAX 200.0
#endif
#ifndef Y_MAX
  #define Y_MAX 200.0
#endif
#ifndef Z_MAX
  #define Z_MAX 200.0
#endif

/// Back to the status screen after this long without a button, s.
#ifndef MENU_TIMEOUT
  #define MENU_TIMEOUT          60
#endif

/* ---- Text helpers ------------------------------------------------------ */

/// Line being built, one character more for the terminating zero.
static char lb[DISPLAY_COLS + 1];
static uint8_t lp;

static void lb_start(void) {
  lp = 0;
  lb[0] = '\0';
}

static void lb_char(char c) {
  if (lp < DISPLAY_COLS) {
    lb[lp++] = c;
    lb[lp] = '\0';
  }
}

static void lb_str(const char *s) {
  while (*s)
    lb_char(*s++);
}

/// Number with 'decimals' digits after the point (v in 10^-decimals).
static void lb_fixed(int32_t v, uint8_t decimals) {
  char d[12];
  uint8_t n = 0;
  uint32_t u;

  if (v < 0) {
    lb_char('-');
    u = (uint32_t)(-v);
  }
  else {
    u = (uint32_t)v;
  }
  do {
    d[n++] = (char)('0' + u % 10);
    u /= 10;
  } while (u || n <= decimals);
  while (n) {
    if (n == decimals)
      lb_char('.');
    lb_char(d[--n]);
  }
}

static void lb_uint(uint32_t v) {
  lb_fixed((int32_t)v, 0);
}

#if DISPLAY_LINES >= 8 || defined DISPLAY_MENU
/// Pad with spaces up to column 'col'.
static void lb_pad(uint8_t col) {
  while (lp < col && lp < DISPLAY_COLS)
    lb_char(' ');
}

/// Right-align 'value' at the end of the line.
static void lb_right(const char *value) {
  uint8_t n = (uint8_t)strlen(value);

  if (n < DISPLAY_COLS)
    lb_pad(DISPLAY_COLS - n);
  lb_str(value);
}
#endif

/// Temperature in whole degrees, from quarter degrees.
static uint16_t deg(uint16_t q) {
  return (uint16_t)((q + 2) >> 2);
}

/* ---- Status screen ----------------------------------------------------- */

/// Whether a file print is going on (printing or paused).
static uint8_t printing(void) {
  #ifdef SD
    return (gcode_sources & GCODE_SOURCE_SD) || job_paused();
  #else
    return 0;
  #endif
}

#ifdef HEATER_EXTRUDER
static void lb_temps(uint8_t wide) {
  lb_str(wide ? "Hotend " : "E");
  lb_uint(deg(temp_get(TEMP_SENSOR_extruder)));
  lb_char('/');
  lb_uint(deg(temp_get_target(TEMP_SENSOR_extruder)));
}
#endif

#ifdef HEATER_BED
static void lb_bed(uint8_t wide) {
  lb_str(wide ? "Bed    " : "B");
  lb_uint(deg(temp_get(TEMP_SENSOR_bed)));
  lb_char('/');
  lb_uint(deg(temp_get_target(TEMP_SENSOR_bed)));
}
#endif

/// Print progress: percent and time, "" when not printing.
static void lb_progress(void) {
  #ifdef SD
    uint32_t size = sd_file_size(), s = job_elapsed_s();
    uint8_t pct = status_get_progress();

    if ( ! printing())
      return;
    if (pct > 100 && size)
      pct = (uint8_t)((uint64_t)sd_position() * 100 / size);
    lb_str(job_paused() ? "Paused " : "SD ");
    if (pct <= 100) {
      lb_uint(pct);
      lb_str("% ");
    }
    lb_uint(s / 3600);
    lb_char(':');
    lb_char((char)('0' + s / 600 % 6));
    lb_char((char)('0' + s / 60 % 10));
  #endif
}

/// Message line: M117 text, else what the printer does.
static void lb_message(void) {
  const char *msg = status_get_message();

  if (*msg)
    lb_str(msg);
  #ifdef POWER_LOSS_RECOVERY
  else if (plr_pending())
    lb_str("Power loss: resume?");
  #endif
  else if (filament_change_active())
    lb_str("Filament change");
  else if (temp_waiting())
    lb_str("Heating...");
  #ifdef SD
  else if (printing())
    lb_str(sd_file_name());
  #endif
  else
    lb_str("Ready");
}

#if DISPLAY_LINES >= 4
static void lb_position(uint8_t z_only) {
  update_current_position();
  if ( ! z_only) {
    lb_char('X');
    lb_fixed(current_position.axis[X] / 100, 1);
    lb_str(" Y");
    lb_fixed(current_position.axis[Y] / 100, 1);
    lb_char(' ');
  }
  lb_char('Z');
  lb_fixed(current_position.axis[Z] / 10, 2);
}

static void lb_fan_speed(void) {
  #ifdef HEATER_FAN
    lb_str("Fan ");
    lb_uint((uint32_t)heaters_runtime[HEATER_FAN].heater_output * 100 / 255);
    lb_str("% ");
  #endif
  lb_str("F");
  lb_uint(((uint32_t)next_target.target.f_multiplier * 100 + 128) / 256);
  lb_char('%');
}
#endif /* DISPLAY_LINES >= 4 */

static void status_screen(void) {
  uint8_t l = 0;

  #if DISPLAY_LINES >= 8
    lb_start(); lb_str("Teacup");
    lb_right(printing() ? (job_paused() ? "Paused" : "Printing") : "");
    display_text_line(l++, lb);
    #ifdef HEATER_EXTRUDER
      lb_start(); lb_temps(1); display_text_line(l++, lb);
    #endif
    #ifdef HEATER_BED
      lb_start(); lb_bed(1); display_text_line(l++, lb);
    #endif
    lb_start(); lb_fan_speed(); display_text_line(l++, lb);
    lb_start(); lb_position(0); display_text_line(l++, lb);
    while (l < DISPLAY_LINES - 2)
      display_text_line(l++, "");
    lb_start(); lb_progress(); display_text_line(l++, lb);
    lb_start(); lb_message(); display_text_line(l++, lb);
  #elif DISPLAY_LINES >= 4
    lb_start();
    #ifdef HEATER_EXTRUDER
      lb_temps(0); lb_char(' ');
    #endif
    #ifdef HEATER_BED
      lb_bed(0);
    #endif
    display_text_line(l++, lb);
    lb_start(); lb_position(0); display_text_line(l++, lb);
    lb_start();
    if (printing())
      lb_progress();
    else
      lb_fan_speed();
    display_text_line(l++, lb);
    while (l < DISPLAY_LINES - 1)
      display_text_line(l++, "");
    lb_start(); lb_message(); display_text_line(l++, lb);
  #else
    lb_start();
    #ifdef HEATER_EXTRUDER
      lb_temps(0); lb_char(' ');
    #endif
    #ifdef HEATER_BED
      lb_bed(0);
    #endif
    display_text_line(l++, lb);
    lb_start();
    if (printing() && ! *status_get_message())
      lb_progress();
    else
      lb_message();
    display_text_line(l++, lb);
  #endif
}

/* ---- Menu -------------------------------------------------------------- */

#ifdef DISPLAY_MENU

/// Commands queued for the main loop.
#define CMDS     4
#define CMD_LEN  64
static char cmds[CMDS][CMD_LEN];
static uint8_t cmd_head, cmd_tail;
/// An action to run in the main loop (stop print), besides the commands.
static void (*pending_action)(void);

/// Queue G-code lines ('\n' separated) for the main loop.
static uint8_t ui_command(const char *gcode) {
  uint8_t next = (cmd_head + 1) % CMDS;

  if (next == cmd_tail)
    return 0;                           // Full, the user pressed too fast.
  strncpy(cmds[cmd_head], gcode, CMD_LEN - 1);
  cmds[cmd_head][CMD_LEN - 1] = '\0';
  cmd_head = next;
  return 1;
}

typedef void (*screen_t)(void);

/// A value to edit.
typedef struct {
  const char *label;
  /// Current value in 10^-decimals, arg: e.g. the axis.
  int32_t (*get)(uint8_t arg);
  uint8_t arg;
  uint8_t decimals;
  int32_t min, max, step;
  /// G-code, the value is appended, e.g. "M104 S".
  const char *gcode;
  /// Instead of gcode: build the command from the value.
  void (*apply)(int32_t v);
  /// Send each step right away as a relative change (babystepping).
  uint8_t live;
} param_t;

#define DEPTH 6
static screen_t screen;                 ///< NULL: status screen.
static screen_t stack[DEPTH];
static uint8_t sel_stack[DEPTH], top_stack[DEPTH], depth;
static uint8_t sel, top, count;         ///< Selected item, first shown, items.
static uint8_t key;                     ///< Event handled by this frame.
static uint8_t item_n;                  ///< Items so far in this frame.
static uint16_t idle_ticks;             ///< 10 ms ticks without a button.
static uint8_t redraw;

/// Edit screen state.
static const param_t *edit_p;
static int32_t edit_v;
/// Live edit (babystep): change not sent yet, the queue was full.
static int32_t live_pending;

/// Confirmation screen state.
static const char *confirm_text;
static const char *confirm_gcode;
static void (*confirm_action)(void);
static uint8_t confirm_close;           ///< Yes: back to the status screen.

#define ROWS (DISPLAY_LINES - 1)        ///< Item lines below the title.

static void go(screen_t s) {
  if (depth < DEPTH) {
    stack[depth] = screen;
    sel_stack[depth] = sel;
    top_stack[depth] = top;
    depth++;
  }
  screen = s;
  sel = top = 0;
  redraw = 1;
}

static void back(void) {
  if (depth) {
    depth--;
    screen = stack[depth];
    sel = sel_stack[depth];
    top = top_stack[depth];
  }
  else {
    screen = NULL;
  }
  redraw = 1;
}

static void title(const char *t) {
  lb_start();
  lb_str(t);
  display_text_line(0, lb);
}

/**
  One menu item. Draws it if it is in the shown window. Returns 1 if it
  is the selected item and OK was pressed (the event is used up then).
*/
static uint8_t item(const char *label, const char *value) {
  uint8_t n = item_n++;

  if (n >= top && n < top + ROWS) {
    lb_start();
    lb_char(n == sel ? '>' : ' ');
    lb_str(label);
    if (value)
      lb_right(value);
    display_text_line((uint8_t)(1 + n - top), lb);
  }
  if (n == sel && key == BUTTON_OK) {
    key = BUTTON_NONE;
    return 1;
  }
  return 0;
}

static uint8_t item_back(void) {
  if (item(depth ? "< Back" : "< Status", NULL)) {
    back();
    return 1;
  }
  return 0;
}

static uint8_t item_sub(const char *label, screen_t s) {
  if (item(label, ">")) {
    go(s);
    return 1;
  }
  return 0;
}

static void item_cmd(const char *label, const char *gcode) {
  if (item(label, NULL))
    ui_command(gcode);
}

static void screen_edit(void);
static void live_flush(void);

/// Value as text into buf (for the right side of an item).
static void fmt_value(char *buf, const param_t *p, int32_t v) {
  uint8_t save = lp;
  char save_lb[DISPLAY_COLS + 1];

  memcpy(save_lb, lb, sizeof(lb));
  lb_start();
  lb_fixed(v, p->decimals);
  strcpy(buf, lb);
  memcpy(lb, save_lb, sizeof(lb));
  lp = save;
}

static void item_edit(const param_t *p) {
  char v[DISPLAY_COLS + 1];

  fmt_value(v, p, p->get(p->arg));
  if (item(p->label, v)) {
    live_flush();
    if (live_pending)
      return;                           // Last live edit not sent yet.
    edit_p = p;
    edit_v = p->get(p->arg);
    go(screen_edit);
  }
}

static void screen_confirm(void);

static void item_confirm(const char *label, const char *text,
                         const char *gcode, void (*action)(void),
                         uint8_t close) {
  if (item(label, NULL)) {
    confirm_text = text;
    confirm_gcode = gcode;
    confirm_action = action;
    confirm_close = close;
    go(screen_confirm);
  }
}

/// Clear the lines below the last item.
static void end_menu(void) {
  uint8_t n;

  count = item_n;
  for (n = item_n; n < top + ROWS; n++)
    if (n >= top)
      display_text_line((uint8_t)(1 + n - top), "");
}

/// Command with the value appended, value with 'decimals' digits.
static uint8_t command_value(const char *prefix, int32_t v, uint8_t decimals) {
  char c[CMD_LEN];
  size_t n = strlen(prefix);

  if (n + DISPLAY_COLS + 1 > CMD_LEN)
    return 1;                           // Doesn't fit, drop it.
  memcpy(c, prefix, n);
  lb_start();
  lb_fixed(v, decimals);
  strcpy(c + n, lb);
  return ui_command(c);
}

/// Send the pending change of a live edit, if the queue has room.
static void live_flush(void) {
  if (live_pending && edit_p &&
      command_value(edit_p->gcode, live_pending, edit_p->decimals))
    live_pending = 0;
}

static void screen_edit(void) {
  const param_t *p = edit_p;
  int32_t step = p->step, old = edit_v;
  char v[DISPLAY_COLS + 1];

  if ((key & ~BUTTON_FAST) == BUTTON_UP || (key & ~BUTTON_FAST) == BUTTON_DOWN) {
    if (key & BUTTON_FAST)
      step *= 10;
    if ((key & ~BUTTON_FAST) == BUTTON_UP)
      edit_v += step;
    else
      edit_v -= step;
    if (edit_v > p->max)
      edit_v = p->max;
    if (edit_v < p->min)
      edit_v = p->min;
    if (p->live)
      live_pending += edit_v - old;
  }
  else if (key == BUTTON_OK) {
    if ( ! p->live) {
      if (p->apply)
        p->apply(edit_v);
      else
        command_value(p->gcode, edit_v, p->decimals);
    }
    back();
    return;
  }
  else if (key == BUTTON_BACK) {
    back();
    return;
  }

  if (p->live)
    live_flush();
  title(p->label);
  fmt_value(v, p, edit_v);
  lb_start();
  lb_pad((uint8_t)((DISPLAY_COLS - strlen(v)) / 2));
  lb_str(v);
  display_text_line(1, lb);
  #if DISPLAY_LINES >= 4
    display_text_line(2, "");
    display_text_line(DISPLAY_LINES - 1, p->live ? "Up/Down, OK: done" :
                                                   "Up/Down, OK: set");
    {
      uint8_t l;
      for (l = 3; l < DISPLAY_LINES - 1; l++)
        display_text_line(l, "");
    }
  #endif
}

static void screen_confirm(void) {
  title(confirm_text);
  item_n = 0;
  if (item("< No", NULL)) {
    back();
    return;
  }
  if (item("Yes", NULL)) {
    back();
    if (confirm_close) {
      screen = NULL;
      depth = 0;
    }
    if (confirm_gcode)
      ui_command(confirm_gcode);
    if (confirm_action)
      pending_action = confirm_action;
    return;
  }
  end_menu();
}

/* ---- Values ------------------------------------------------------------ */

static int32_t get_hotend(uint8_t arg) {
  (void)arg;
  #ifdef HEATER_EXTRUDER
    return deg(temp_get_target(TEMP_SENSOR_extruder));
  #else
    return 0;
  #endif
}

static int32_t get_bed(uint8_t arg) {
  (void)arg;
  #ifdef HEATER_BED
    return deg(temp_get_target(TEMP_SENSOR_bed));
  #else
    return 0;
  #endif
}

static int32_t get_fan(uint8_t arg) {
  (void)arg;
  #ifdef HEATER_FAN
    return ((int32_t)heaters_runtime[HEATER_FAN].heater_output * 100 + 127) / 255;
  #else
    return 0;
  #endif
}

static void apply_fan(int32_t v) {
  command_value("M106 S", (v * 255 + 50) / 100, 0);
}

static int32_t get_speed(uint8_t arg) {
  (void)arg;
  return ((int32_t)next_target.target.f_multiplier * 100 + 128) / 256;
}

static int32_t get_flow(uint8_t arg) {
  (void)arg;
  return ((int32_t)next_target.target.e_multiplier * 100 + 128) / 256;
}

static const param_t p_hotend = {
  "Hotend", get_hotend, 0, 0, 0, HEATER_MAXTEMP - 15, 1, "M104 S", NULL, 0 };
static const param_t p_bed = {
  "Bed", get_bed, 0, 0, 0, BED_MAXTEMP - 10, 1, "M140 S", NULL, 0 };
static const param_t p_fan = {
  "Fan %", get_fan, 0, 0, 0, 100, 5, NULL, apply_fan, 0 };
static const param_t p_speed = {
  "Speed %", get_speed, 0, 0, 10, 500, 5, "M220 S", NULL, 0 };
static const param_t p_flow = {
  "Flow %", get_flow, 0, 0, 10, 500, 1, "M221 S", NULL, 0 };

#ifdef BABYSTEPPING
static int32_t get_babystep(uint8_t arg) {
  (void)arg;
  return babystep_offset() / 10;        // um -> 1/100 mm.
}
static const param_t p_babystep = {
  "Babystep Z", get_babystep, 0, 2, -200, 200, 1, "M290 Z", NULL, 1 };
#endif

/// Current position of an axis in 1/10 mm.
static int32_t get_pos(uint8_t axis) {
  update_current_position();
  return current_position.axis[axis] / 100;
}

static const param_t p_move[3] = {
  { "Move X", get_pos, X, 1, 0, (int32_t)(X_MAX * 10), 10, "G90\nG1 F3000 X", NULL, 0 },
  { "Move Y", get_pos, Y, 1, 0, (int32_t)(Y_MAX * 10), 10, "G90\nG1 F3000 Y", NULL, 0 },
  { "Move Z", get_pos, Z, 1, 0, (int32_t)(Z_MAX * 10), 1, "G90\nG1 F600 Z", NULL, 0 },
};

/* Settings, in the units of the G-code that sets them. */

static int32_t get_steps(uint8_t axis) {
  return (int32_t)settings.steps_per_m[axis];     // steps/mm * 1000
}
static int32_t get_feedrate(uint8_t axis) {
  return (int32_t)(settings.max_feedrate[axis] / 60);   // mm/min -> mm/s
}
static int32_t get_max_accel(uint8_t axis) {
  return (int32_t)settings.max_accel[axis];
}
static int32_t get_jerk(uint8_t axis) {
  return (int32_t)(settings.max_jerk[axis] / 6);  // mm/min -> 1/10 mm/s
}
static int32_t get_accel(uint8_t which) {
  return (int32_t)(which == 0 ? settings.acceleration :
                   which == 1 ? settings.accel_travel : settings.accel_retract);
}

static const param_t p_settings[] = {
  { "Steps/mm X", get_steps, X, 3, 1000, 4096000, 100, "M92 X", NULL, 0 },
  { "Steps/mm Y", get_steps, Y, 3, 1000, 4096000, 100, "M92 Y", NULL, 0 },
  { "Steps/mm Z", get_steps, Z, 3, 1000, 40960000, 100, "M92 Z", NULL, 0 },
  { "Steps/mm E", get_steps, E, 3, 1000, 4096000, 100, "M92 E", NULL, 0 },
  { "Max F X", get_feedrate, X, 0, 1, 1000, 1, "M203 X", NULL, 0 },
  { "Max F Y", get_feedrate, Y, 0, 1, 1000, 1, "M203 Y", NULL, 0 },
  { "Max F Z", get_feedrate, Z, 0, 1, 100, 1, "M203 Z", NULL, 0 },
  { "Max F E", get_feedrate, E, 0, 1, 200, 1, "M203 E", NULL, 0 },
  { "Accel print", get_accel, 0, 0, 10, 20000, 10, "M204 P", NULL, 0 },
  { "Accel travel", get_accel, 1, 0, 10, 20000, 10, "M204 T", NULL, 0 },
  { "Accel retr.", get_accel, 2, 0, 10, 20000, 10, "M204 R", NULL, 0 },
  { "Max acc X", get_max_accel, X, 0, 10, 50000, 10, "M201 X", NULL, 0 },
  { "Max acc Y", get_max_accel, Y, 0, 10, 50000, 10, "M201 Y", NULL, 0 },
  { "Max acc Z", get_max_accel, Z, 0, 1, 5000, 1, "M201 Z", NULL, 0 },
  { "Max acc E", get_max_accel, E, 0, 10, 50000, 10, "M201 E", NULL, 0 },
  { "Jerk X", get_jerk, X, 1, 0, 1000, 1, "M205 X", NULL, 0 },
  { "Jerk Y", get_jerk, Y, 1, 0, 1000, 1, "M205 Y", NULL, 0 },
  { "Jerk Z", get_jerk, Z, 1, 0, 1000, 1, "M205 Z", NULL, 0 },
  { "Jerk E", get_jerk, E, 1, 0, 1000, 1, "M205 E", NULL, 0 },
};

#ifdef LINEAR_ADVANCE
static int32_t get_la_k(uint8_t arg) {
  (void)arg;
  return (int32_t)(settings.la_k / 10);           // 1/10000 s -> 1/1000
}
static const param_t p_la_k = {
  "Lin. adv. K", get_la_k, 0, 3, 0, 2000, 5, "M900 K", NULL, 0 };
#endif

#ifdef Z_PROBE
static int32_t get_probe_z(uint8_t arg) {
  (void)arg;
  return probe_offset[Z] / 10;                    // um -> 1/100 mm
}
static const param_t p_probe_z = {
  "Probe Z off.", get_probe_z, 0, 2, -1000, 1000, 1, "M851 Z", NULL, 0 };
#endif

#ifdef FIRMWARE_RETRACT
static int32_t get_retract(uint8_t which) {
  return which ? (int32_t)settings.retract_feedrate :
                 (int32_t)(settings.retract_length / 10);  // um -> 1/100 mm
}
static const param_t p_retract_len = {
  "Retract mm", get_retract, 0, 2, 0, 1000, 5, "M207 S", NULL, 0 };
static const param_t p_retract_f = {
  "Retract F", get_retract, 1, 0, 60, 12000, 60, "M207 F", NULL, 0 };
#endif

/* ---- Actions ----------------------------------------------------------- */

/// Stop the file print: no more lines, drop the moves, heaters off, Z up.
static void stop_print(void) {
  gcode_sources &= (uint8_t)~GCODE_SOURCE_SD;
  emergency_quickstop();
  #ifdef POWER_LOSS_RECOVERY
    plr_clear();
  #endif
  job_stop();
  status_set_message("Print stopped");
  ui_command("M104 S0\nM140 S0\nM107\nG91\nG1 Z10 F600\nG90");
}

/// Continue after a filament change (like M108 from the host).
static void continue_filament(void) {
  temp_cancel_wait();
}

/* ---- Screens ----------------------------------------------------------- */

static void menu_temperature(void) {
  title("Temperature");
  item_back();
  item_edit(&p_hotend);
  #ifdef HEATER_BED
    item_edit(&p_bed);
  #endif
  #ifdef HEATER_FAN
    item_edit(&p_fan);
  #endif
  item_cmd("Preheat PLA", "M104 S" STR(PREHEAT_PLA_HOTEND) "\nM140 S"
           STR(PREHEAT_PLA_BED));
  item_cmd("Preheat PETG", "M104 S" STR(PREHEAT_PETG_HOTEND) "\nM140 S"
           STR(PREHEAT_PETG_BED));
  item_cmd("Cooldown", "M104 S0\nM140 S0\nM107");
  end_menu();
}

static void menu_tune(void) {
  title("Tune");
  item_back();
  item_edit(&p_speed);
  item_edit(&p_flow);
  item_edit(&p_hotend);
  #ifdef HEATER_BED
    item_edit(&p_bed);
  #endif
  #ifdef HEATER_FAN
    item_edit(&p_fan);
  #endif
  #ifdef BABYSTEPPING
    item_edit(&p_babystep);
  #endif
  item_cmd("Change filament", "M600");
  end_menu();
}

static void menu_motion(void) {
  uint8_t i;

  title("Motion");
  item_back();
  item_cmd("Home all", "G28");
  #ifdef BED_LEVELING
    item_cmd("Auto level", "G28\nG29");
  #endif
  #ifdef Z_STEPPER_ALIGN
    item_cmd("Align Z", "G28\nG34");
  #endif
  // Moves to absolute positions: only on homed axes.
  for (i = 0; i < 3; i++)
    if (axes_homed & (1U << i))
      item_edit(&p_move[i]);
  item_cmd("Steppers off", "M84");
  end_menu();
}

static void menu_settings(void) {
  uint8_t i;

  title("Settings");
  item_back();
  for (i = 0; i < sizeof(p_settings) / sizeof(p_settings[0]); i++)
    item_edit(&p_settings[i]);
  #ifdef LINEAR_ADVANCE
    item_edit(&p_la_k);
  #endif
  #ifdef Z_PROBE
    item_edit(&p_probe_z);
  #endif
  #ifdef FIRMWARE_RETRACT
    item_edit(&p_retract_len);
    item_edit(&p_retract_f);
  #endif
  item_cmd("Store settings", "M500");
  item_cmd("Load settings", "M501");
  item_confirm("Defaults", "Reset settings?", "M502", NULL, 0);
  end_menu();
}

#ifdef SD

/// Directory shown in the file list, "" for the top level.
static char dir_path[40];
static uint16_t dir_count;              ///< Entries, counted on entering.
static uint8_t dir_state;               ///< SD_ENTRY_ERROR: no card.
static char file_path[54];

static void menu_files(void);

/// (Re)count the entries of dir_path.
static void files_open(void) {
  char name[13];
  uint8_t r = SD_ENTRY_NONE;

  dir_count = 0;
  while (dir_count < 250 &&
         (r = sd_dir_entry(dir_path, dir_count, name)) != SD_ENTRY_NONE &&
         r != SD_ENTRY_ERROR)
    dir_count++;
  dir_state = r;
}

static void menu_files(void) {
  char name[13];
  uint16_t i;
  uint8_t r;

  title(dir_path[0] ? dir_path : "Print from SD");
  if (item(dir_path[0] ? "< Up" : "< Back", NULL)) {
    char *slash = strrchr(dir_path, '/');

    if (dir_path[0] == '\0') {
      back();
      return;
    }
    if (slash)
      *slash = '\0';
    else
      dir_path[0] = '\0';
    sel = top = 0;
    files_open();
    redraw = 1;
    return;
  }
  if (dir_state == SD_ENTRY_ERROR) {
    if (item("Mount card", NULL)) {
      ui_command("M21");
      dir_count = 0;
      dir_state = SD_ENTRY_NONE;
    }
    end_menu();
    return;
  }
  if (item("Refresh", NULL)) {
    files_open();
    redraw = 1;
    return;
  }
  // Only the entries in the shown window (and a selected one) are read.
  for (i = 0; i < dir_count; i++) {
    uint8_t n = item_n;

    if ((n < top || n >= top + ROWS) && n != sel) {
      item_n++;
      continue;
    }
    r = sd_dir_entry(dir_path, i, name);
    if (r == SD_ENTRY_DIR) {
      if (item(name, "/")) {
        size_t l = strlen(dir_path);

        if (l + 14 < sizeof(dir_path)) {
          if (l)
            dir_path[l++] = '/';
          strcpy(dir_path + l, name);
        }
        sel = top = 0;
        files_open();
        redraw = 1;
        return;
      }
    }
    else if (r == SD_ENTRY_FILE) {
      if (item(name, NULL)) {
        strcpy(file_path, "M23 ");
        if (dir_path[0]) {
          strcat(file_path, dir_path);
          strcat(file_path, "/");
        }
        strcat(file_path, name);
        strcat(file_path, "\nM24");
        confirm_text = "Print file?";
        confirm_gcode = file_path;
        confirm_action = NULL;
        confirm_close = 1;
        go(screen_confirm);
        return;
      }
    }
    else {
      item_n++;
    }
  }
  end_menu();
}

static void open_files(void) {
  dir_path[0] = '\0';
  files_open();
  go(menu_files);
}

#endif /* SD */

static void menu_main(void) {
  title("Main");
  item_back();
  #ifdef POWER_LOSS_RECOVERY
    if (plr_pending()) {
      item_cmd("Resume print", "M1000");
      item_confirm("Discard print", "Discard resume?", "M1000 C", NULL, 0);
    }
  #endif
  if (filament_change_active() && item("Continue", NULL))
    continue_filament();
  if (printing()) {
    #ifdef SD
      if (gcode_sources & GCODE_SOURCE_SD)
        item_cmd("Pause print", "M25");
      else
        item_cmd("Resume print", "M24");
    #endif
    item_confirm("Stop print", "Stop print?", NULL, stop_print, 1);
    item_sub("Tune", menu_tune);
  }
  else {
    #ifdef SD
      if (item("Print from SD", ">"))
        open_files();
    #endif
    item_sub("Motion", menu_motion);
  }
  item_sub("Temperature", menu_temperature);
  if ( ! printing())
    item_sub("Settings", menu_settings);
  end_menu();
}

uint8_t ui_menu_open(void) {
  return screen != NULL;
}

/// Run the current screen for one event (or a refresh).
static void menu_frame(void) {
  if (screen != screen_edit && screen != NULL) {
    if ((key & ~BUTTON_FAST) == BUTTON_UP) {
      if (sel)
        sel--;
      key = BUTTON_NONE;
    }
    else if ((key & ~BUTTON_FAST) == BUTTON_DOWN) {
      if (sel + 1 < count)
        sel++;
      key = BUTTON_NONE;
    }
    else if (key == BUTTON_BACK) {
      back();
      key = BUTTON_NONE;
    }
    if (sel < top)
      top = sel;
    if (sel >= top + ROWS)
      top = (uint8_t)(sel - ROWS + 1);
  }
  if (screen) {
    screen_t s = screen;

    item_n = 0;
    s();
    // A screen change: draw the new screen right away.
    if (screen != s && screen) {
      key = BUTTON_NONE;
      item_n = 0;
      if (sel >= top + ROWS)
        top = (uint8_t)(sel - ROWS + 1);
      screen();
    }
  }
}

uint8_t ui_execute(void) {
  const char *s;
  uint8_t saved;

  if (pending_action) {
    void (*a)(void) = pending_action;

    pending_action = NULL;
    a();
    redraw = 1;
    return 1;
  }
  if (cmd_tail == cmd_head)
    return 0;
  s = cmds[cmd_tail];
  saved = gcode_active;
  gcode_active = GCODE_SOURCE_INIT;
  while (*s) {
    while (*s && *s != '\n')
      gcode_parse_char((uint8_t)*s++);
    gcode_parse_char('\n');
    if (*s == '\n')
      s++;
  }
  gcode_active = saved;
  cmd_tail = (cmd_tail + 1) % CMDS;
  redraw = 1;                           // Show the new values.
  return 1;
}

#else /* DISPLAY_MENU */

uint8_t ui_execute(void) {
  return 0;
}

#endif /* DISPLAY_MENU */

/* ---- Common ------------------------------------------------------------ */

static uint16_t refresh_ticks;
static uint16_t greeting_ticks = 200;

void ui_init(void) {
  display_greeting();
}

void ui_tick(void) {
  #ifdef DISPLAY_MENU
    uint8_t e;

    while ((e = buttons_get()) != BUTTON_NONE) {
      idle_ticks = 0;
      greeting_ticks = 0;
      if (screen == NULL) {
        if (e == BUTTON_OK) {
          depth = 0;
          screen = menu_main;
          sel = top = 0;
          redraw = 1;
        }
        continue;
      }
      key = e;
      menu_frame();
      key = BUTTON_NONE;
    }
    live_flush();
    if (screen && ++idle_ticks >= MENU_TIMEOUT * 100U) {
      screen = NULL;
      depth = 0;
      redraw = 1;
    }
  #endif

  if (greeting_ticks) {
    greeting_ticks--;
    return;
  }

  // Refresh twice a second (values change), or right away after a change.
  #ifdef DISPLAY_MENU
    if (redraw) {
      redraw = 0;
      refresh_ticks = 0;
      if (screen) {
        key = BUTTON_NONE;
        menu_frame();
      }
      else {
        status_screen();
      }
      return;
    }
  #endif
  if (++refresh_ticks < 50)
    return;
  refresh_ticks = 0;
  #ifdef DISPLAY_MENU
    if (screen) {
      #ifdef SD
        // The file list reads the card, only on a button.
        if (screen == menu_files)
          return;
      #endif
      key = BUTTON_NONE;
      menu_frame();
      return;
    }
  #endif
  status_screen();
}

#endif /* DISPLAY */
