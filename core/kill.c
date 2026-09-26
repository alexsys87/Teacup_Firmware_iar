/** \file
  \brief Fatal error handling (Marlin's kill()).

  Messages follow Marlin's wording, so hosts like OctoPrint, Pronterface and
  Repetier recognise the halted state and stop sending:

    Error:Thermal Runaway, system stopped! Heater_ID: 0
    Error:Printer halted. kill() called!
*/

#include "kill.h"
#include "arch.h"
#include "config_wrapper.h"
#include "serial.h"
#include "msg.h"
#include "heater.h"
#include "temp.h"
#include "timer.h"
#include "pinio.h"
#include "dda_queue.h"
#include "watchdog.h"

volatile uint8_t printer_killed = 0;

_Noreturn void printer_kill(const char *reason, int16_t id) {
  static const char m999[] = "M999";
  uint8_t matches[SERIAL_NUM_PORTS] = { 0 };  // Per host port: chars of
                                              // "M999" matched, 5 = in tail.
  uint32_t loops = 0;

  // Nothing else may run from now on, not even the step interrupt.
  __disable_irq();
  printer_killed = 1;

  heater_emergency_off();
  timer_stop();
  unstep();
  queue_flush();
  power_off();

  // The error goes to all hosts, whichever command we were executing.
  serial_set_output(SERIAL_MASK_ALL);
  serial_writestr("Error:");
  serial_writestr(reason);
  if (id >= 0) {
    serial_writestr(" Heater_ID: ");
    #ifdef HEATER_BED
      if (id == (int16_t)TEMP_SENSOR_bed)
        serial_writestr("bed");
      else
    #endif
    write_uint32(serial_writechar, (uint32_t)id);
  }
  serial_writechar('\n');
  serial_writestr("Error:Printer halted. kill() called!\n");
  serial_flush();

  /*
    Halted. Keep outputs off, feed the watchdog (a watchdog reset would
    look like a normal start to the host) and wait for M999.

    Interrupts stay disabled, the host ports are polled every ~10 us
    (faster than one character at 250000 baud; USB NAKs the host until we
    read, so nothing gets lost there), watchdog and heater outputs are
    refreshed every ~10 ms. Few peripheral accesses per loop also keep
    emulators (Renode) fast.
  */
  for (;;) {
    volatile uint32_t d;
    int16_t c;
    uint8_t port = 0;
    uint8_t *match;

    if ((loops++ & 1023) == 0) {
      wd_reset();
      heater_emergency_off();
    }

    for (d = 0; d < F_CPU / 10000000UL * 20; d++)
      ;                           // ~10 us

    c = serial_rx_poll(&port);
    if (c < 0 || port >= SERIAL_NUM_PORTS)
      continue;
    match = &matches[port];
    if (c >= 'a' && c <= 'z')
      c -= 'a' - 'A';

    if (c == '\n' || c == '\r') {
      if (*match >= 4) {
        serial_writestr("echo:Restarting\n");
        serial_flush();
        NVIC_SystemReset();
      }
      *match = 0;
    }
    else if (*match < 4 && c == m999[*match]) {
      (*match)++;
    }
    else if (*match == 4 && (c == ' ' || c == '*')) {
      *match = 5;                 // Checksum or spaces follow.
    }
    else if (*match != 5) {
      *match = (c == 'M') ? 1 : 0;
    }
  }
}
