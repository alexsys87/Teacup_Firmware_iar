/** \file
  \brief Do stuff periodically.

  Note: the main loop function was called clock() in the original Teacup.
  That name is reserved by the C standard library (<time.h>), so it's
  clock_poll() now.
*/

#include "clock.h"
#include "pinio.h"
#include "sersendf.h"
#include "dda_queue.h"
#include "watchdog.h"
#include "debug.h"
#include "heater.h"
#include "serial.h"
#include "temp.h"
#include "timer.h"
#include "display.h"
#include "atomic.h"
#include "emergency_parser.h"
#include "gcode_queue.h"
#include "filament.h"
#include "babystep.h"

/**
  If the specific bit is set, execute the following block exactly once
  and then clear the flag.
*/
#define ifclock(F) for ( ; F; F = 0)

/**
  Every time our clock fires we increment this,
  so we know when 10ms/100ms/250ms/1s has elapsed.
*/
static volatile uint32_t clock_ms = 0;
static uint_fast8_t clock_counter_10ms = 0;
static uint_fast8_t clock_counter_100ms = 0;
static uint_fast8_t clock_counter_250ms = 0;
static uint_fast8_t clock_counter_1s = 0;

/**
  Flags to tell clock_poll() when above have elapsed.
*/
static volatile uint_fast8_t clock_flag_10ms = 0;
static volatile uint_fast8_t clock_flag_100ms = 0;
static volatile uint_fast8_t clock_flag_250ms = 0;
static volatile uint_fast8_t clock_flag_1s = 0;


/** Advance our clock by a tick.

  Update clock counters accordingly. Called from the TICK_TIME interrupt.
*/
TEACUP_HOT
void clock_tick(void) {
  clock_ms += TICK_TIME_MS;
  clock_counter_10ms += TICK_TIME_MS;
  if (clock_counter_10ms >= 10) {
    clock_counter_10ms -= 10;
    clock_flag_10ms = 1;

    clock_counter_100ms++;
    if (clock_counter_100ms >= 10) {
      clock_counter_100ms = 0;
      clock_flag_100ms = 1;
    }

    clock_counter_250ms++;
    if (clock_counter_250ms >= 25) {
      clock_counter_250ms = 0;
      clock_flag_250ms = 1;

      clock_counter_1s++;
      if (clock_counter_1s >= 4) {
        clock_counter_1s = 0;
        clock_flag_1s = 1;
      }
    }
  }
}

uint32_t clock_millis(void) {
  return clock_ms;
}

/** Do stuff every 1/4 second.

  Called from clock_10ms(), do not call directly.
*/
static void clock_250ms(void) {

  // Paused for a filament change: the hotend heater may be off, but the
  // motors must keep their position.
  if (heaters_all_zero() && ! filament_change_active()) {
    if (psu_timeout > (30 * 4)) {
      power_idle();
    }
    else {
      ATOMIC_START();
        psu_timeout++;
      ATOMIC_END();
    }
  }

  temp_heater_tick();

  ifclock(clock_flag_1s) {
    static uint8_t wait_for_temp = 0;

    #ifdef DISPLAY
      display_clock();
    #endif

    temp_residency_tick();
    temp_periodic_print();
    gcode_queue_keepalive();
    steppers_idle_tick(mb_tail_dda != NULL || temp_wait_active() ||
                       filament_change_active());

    if (temp_waiting()) {
      // Report temperatures while waiting (like Marlin during M109/M190):
      // hosts update their graphs from these, M105 has to wait meanwhile.
      temp_print(TEMP_SENSOR_none);
      wait_for_temp = 1;
    }
    else {
      if (wait_for_temp) {
        if (temp_wait_cancelled())
          serial_writestr("echo:Wait for temperature cancelled\n");
        else
          serial_writestr("Temp achieved\n");
        wait_for_temp = 0;
      }
    }

    if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
      DDA *dda = mb_tail_dda;

      // current position
      update_current_position();
      sersendf_P(("Pos: %lq,%lq,%lq,%lq,%lu\n"),
                 current_position.axis[X], current_position.axis[Y],
                 current_position.axis[Z], current_position.axis[E],
                 current_position.F);

      // target position
      if (dda != NULL)
        sersendf_P(("Dst: %lq,%lq,%lq,%lq,%lu\n"),
                   dda->endpoint.axis[X], dda->endpoint.axis[Y],
                   dda->endpoint.axis[Z], dda->endpoint.axis[E],
                   dda->endpoint.F);

      // Queue
      print_queue();

      // newline
      serial_writechar('\n');
    }
  }
}

/** Do stuff every 10 milliseconds.

  Called from clock_poll(), do not call directly.
*/
static void clock_10ms(void) {
  // reset watchdog
  wd_reset();

  temp_sensor_tick();

  soft_pwm_tick();
  #ifdef HEATER_FAN
    fan_tick();
  #endif
  filament_tick();
  babystep_tick();

  ifclock(clock_flag_100ms) {
    temp_pid_tick();
  }

  ifclock(clock_flag_250ms) {
    clock_250ms();
  }
}

/**
  Do reoccuring stuff. Call it occasionally in busy loops.

  Other than clock_tick() above, which is called at a constant interval, this
  is called from the main loop. So it can be called very often on an idle
  printer, but rather rarely on one running full speed.
*/
void clock_poll(void) {
  emergency_poll();

  // Receive and check lines also while a command blocks.
  gcode_queue_read_serial();

  ifclock(clock_flag_10ms) {
    clock_10ms();
  }

  #ifdef DISPLAY
    display_tick();
  #endif
}
