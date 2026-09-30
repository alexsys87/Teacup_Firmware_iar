/** \file
  \brief Main file - this is where it all starts, and ends.

  Teacup RepRap firmware for STM32F401 / STM32F411.
*/

#include "config_wrapper.h"
#include "arch.h"
#include "cpu.h"
#include "serial.h"
#include "dda_queue.h"
#include "gcode_parse.h"
#include "timer.h"
#include "temp.h"
#include "watchdog.h"
#include "debug.h"
#include "heater.h"
#include "analog.h"
#include "pinio.h"
#include "clock.h"
#include "spi.h"
#include "sd.h"
#include "display.h"
#include "ui.h"
#include "gcode_queue.h"
#include "beeper.h"
#include "endstops.h"
#include "spi_flash.h"
#include "settings.h"
#include "filament.h"
#include "probe.h"
#include "host_watch.h"
#include "power_loss.h"
#include "fans.h"
#include "print_stats.h"
#include "spindle.h"
#include "tmc.h"

#ifdef CANNED_CYCLE
  static const char canned_gcode_P[] = CANNED_CYCLE;
#endif

/** Initialise all the subsystems.

  Interrupts are enabled on Cortex-M after reset. We keep them off until
  everything is set up, just like the AVR version did.
*/
static void init(void) {

  __disable_irq();

  // M997: into the DFU bootloader instead of the firmware.
  cpu_check_bootloader();

  // Clocks first, everything else depends on them.
  cpu_init();

  // set up watchdog
  wd_init();

  // set up serial
  serial_init();

  // set up G-code parsing
  gcode_init();

  // set up inputs and outputs
  pinio_init();
  beeper_init();

  #ifdef SPI
    spi_init();
  #endif

  // set up timers
  timer_init();

  // endstop edge interrupts
  endstops_init();

  heater_init();

  #ifdef SPINDLE_LASER
    spindle_init();
  #endif

  // TMC drivers: UART, configured before the first move.
  tmc_init();

  // set up dda
  dda_init();

  // start the analog conversions of all analog temperature sensors
  analog_init();

  // set up temperature inputs
  temp_init();

  #ifdef SD
    sd_init();
  #endif

  // enable interrupts
  __enable_irq();

  // reset watchdog
  wd_reset();

  // prepare the power supply
  power_init();

  #ifdef DISPLAY
    display_init();
    ui_init();
  #endif

  // say hi to host
  serial_writestr("start\n");

  #ifdef SPI_FLASH
    spi_flash_init();
  #endif

  // Mount the card, so M20..M24 and M1000 work without M21 (a missing card
  // just reports "SD init fail").
  #ifdef SD_CARD
    sd_mount();
  #endif

  // Runtime settings: stored ones from Flash, or the configured defaults.
  settings_init();

  // STEP pulses by timers, if configured (reports the assignment).
  step_timers_init();

  // Filament runout sensor input.
  filament_init();

  // BLTouch servo (TIM9), after the step timers: it checks TIM9 is free.
  #ifdef Z_PROBE
    probe_init();
  #endif

  // Print statistics (M78).
  job_init();

  // I/O expander: fans, buttons.
  #ifdef PCF8574_ADDRESS
    expander_init();
  #endif

  // Hotend and controller fans.
  #ifdef FANS
    fans_init();
  #endif

  // An interrupted SD / flash print? Tell the host (M1000 resumes).
  #ifdef POWER_LOSS_RECOVERY
    plr_init();
  #endif

  // Report unusual reset causes. Power-on sets POR and BOR together.
  if (cpu_reset_flags & RCC_CSR_IWDGRSTF)
    serial_writestr("echo:Watchdog Reset\n");
  else if (cpu_reset_flags & RCC_CSR_SFTRSTF)
    serial_writestr("echo:Software Reset\n");
  else if ((cpu_reset_flags & RCC_CSR_BORRSTF) &&
           ! (cpu_reset_flags & RCC_CSR_PORRSTF))
    serial_writestr("echo:Brown out Reset\n");
  if (cpu_clock_source == CPU_CLOCK_HSI) {
    serial_writestr("// clock: HSE crystal failed, running from HSI\n");
    #ifdef USB_CDC
      // USB needs +-0.25 %, HSI gives +-1 %.
      serial_writestr("// USB clock from HSI, USB may not work\n");
    #endif
  }
  else if (cpu_clock_source == CPU_CLOCK_FAILED) {
    serial_writestr("// clock: PLL failed, timing is WRONG\n");
    #ifdef USB_CDC
      serial_writestr("// USB off, no 48 MHz clock\n");
    #endif
  }
  serial_writestr("ok\n");
}

/// this is where it all starts, and ends
///
/// Run init(), then loop: execute queued commands whenever the movement queue
/// has room, and keep the clock running. Serial lines are received and
/// checked in clock_poll() -> gcode_queue_read_serial().
int main(void) {

  init();

  // main loop
  for (;;) {
    // Filament ran out: pause (filament change) between two commands.
    filament_runout_service();
    #ifdef HOST_WATCH
      // Host gone while printing: park.
      host_watch_poll();
    #endif
    #ifdef POWER_LOSS_RECOVERY
      // File print: store the state at layer changes.
      plr_tick();
    #endif
    // Print job timer: stop after the last move, store the statistics.
    job_tick();

    // If the movement queue is full, a move command would block. Wait.
    if (queue_full() == 0) {

      // A command from the menu, else one from the host, else the SD card.
      #ifdef DISPLAY
        uint8_t menu_command = ui_execute();
      #else
        uint8_t menu_command = 0;
      #endif
      if ( ! menu_command && ! gcode_queue_execute()) {
        // Nothing from the host, continue with the SD card, if printing.
        #ifdef SD
          if (gcode_sources & GCODE_SOURCE_SD) {
            gcode_active = GCODE_SOURCE_SD;
            #ifdef POWER_LOSS_RECOVERY
              plr_line_begin();
            #endif
            if (sd_read_gcode_line()) {
              serial_writestr("\nSD file done.\n");
              gcode_sources &= (uint8_t)~GCODE_SOURCE_SD;
              // There is no pf_close(), subsequent reads will stick at EOF
              // and return zeros.
              #ifdef POWER_LOSS_RECOVERY
                plr_file_done();
              #endif
              job_stop_idle();
            }
            gcode_active = GCODE_SOURCE_INIT;
          }
        #endif
      }

      #ifdef CANNED_CYCLE
        /**
          WARNING!

          This code works on a per-character basis. Any data received over
          serial WILL be randomly distributed through the canned G-code.
        */
        {
          static uint32_t canned_gcode_pos = 0;

          gcode_parse_char((uint8_t)canned_gcode_P[canned_gcode_pos]);

          canned_gcode_pos++;
          if (canned_gcode_P[canned_gcode_pos] == 0)
            canned_gcode_pos = 0;
        }
      #endif /* CANNED_CYCLE */
    }

    clock_poll();
  }
}
