/** \file
  \brief Emergency parser, Marlin compatible.

  A small state machine per host port watches the raw character stream in
  the UART and USB interrupts. It recognises lines consisting of M112, M108,
  M410 or M876 S<n> (host prompt answer, continues like M108), optionally
  with a line number (N123) before and a checksum (*45) after. Comments,
  other commands and parameters are ignored.

  The same lines still go through the regular G-code parser afterwards, where
  these commands do nothing.
*/

#include "emergency_parser.h"
#include "arch.h"
#include "atomic.h"
#include "kill.h"
#include "temp.h"
#include "dda.h"
#include "dda_queue.h"
#include "gcode_parse.h"
#include "pinio.h"
#include "serial.h"

enum {
  EP_RESET = 0,     ///< Start of line, spaces allowed.
  EP_N,             ///< Reading a line number.
  EP_M,
  EP_M1,
  EP_M10,
  EP_M11,
  EP_M4,
  EP_M41,
  EP_M8,
  EP_M87,
  EP_DONE,          ///< Command complete, 'cmd' tells which.
  EP_TAIL,          ///< Checksum or spaces after a complete command.
  EP_IGNORE         ///< Something else, skip to end of line.
};

/// Parser state per host port, lines of different ports don't mix.
static uint8_t ep_states[SERIAL_NUM_PORTS];
static uint16_t ep_cmds[SERIAL_NUM_PORTS];
/// 'S' seen after the command (M876 S0 answers a prompt, M876 P1 doesn't).
static uint8_t ep_s_seen[SERIAL_NUM_PORTS];

/// Set by M410 in the interrupt, finished by emergency_poll().
static volatile uint8_t quickstop_pending = 0;

/// Number of finished quickstops, lets long operations (arcs) notice one.
static uint8_t quickstops = 0;

uint8_t emergency_quickstop_count(void) {
  return quickstops;
}

/// Stop step generation right now, in interrupt context.
static void quickstop_isr(void) {
  ATOMIC_START();
    TIM5->DIER = 0;
    NVIC_ClearPendingIRQ(TIM5_IRQn);
    unstep();
  ATOMIC_END();
  quickstop_pending = 1;
}

void emergency_quickstop(void) {
  quickstop_isr();
}

static void ep_execute(uint16_t cmd) {
  switch (cmd) {
    case 112:
      printer_kill("Emergency stop (M112)", KILL_NO_ID);
      // printer_kill() doesn't return. No 'break', to avoid an
      // "unreachable statement" warning in IAR.
    case 108:     // Never reached from 112.
      temp_cancel_wait();
      break;
    case 410:
      quickstop_isr();
      break;
    case 876:     // Only called with S: "Continue" of a host prompt (M600).
      temp_cancel_wait();
      break;
    default:
      break;
  }
}

void emergency_parser_char(uint8_t port, uint8_t c) {
  uint8_t ep_state;

  if (port >= SERIAL_NUM_PORTS)
    return;
  ep_state = ep_states[port];

  if (c >= 'a' && c <= 'z')
    c -= 'a' - 'A';

  if (c == '\n' || c == '\r') {
    ep_states[port] = EP_RESET;
    if ((ep_state == EP_DONE || ep_state == EP_TAIL) &&
        (ep_cmds[port] != 876 || ep_s_seen[port]))
      ep_execute(ep_cmds[port]);
    ep_s_seen[port] = 0;
    return;
  }

  switch (ep_state) {
    case EP_RESET:
      if (c == 'N')
        ep_state = EP_N;
      else if (c == 'M')
        ep_state = EP_M;
      else if (c != ' ')
        ep_state = EP_IGNORE;
      break;

    case EP_N:
      if (c == ' ')
        ep_state = EP_RESET;
      else if (c == 'M')
        ep_state = EP_M;
      else if ((c < '0' || c > '9') && c != '-')
        ep_state = EP_IGNORE;
      break;

    case EP_M:
      ep_state = (c == '1') ? EP_M1 : (c == '4') ? EP_M4 :
                 (c == '8') ? EP_M8 : EP_IGNORE;
      break;
    case EP_M8:
      ep_state = (c == '7') ? EP_M87 : EP_IGNORE;
      break;
    case EP_M87:
      if (c == '6') {
        ep_cmds[port] = 876;
        ep_state = EP_DONE;
      }
      else
        ep_state = EP_IGNORE;
      break;

    case EP_M1:
      ep_state = (c == '0') ? EP_M10 : (c == '1') ? EP_M11 : EP_IGNORE;
      break;

    case EP_M10:
      if (c == '8') {
        ep_cmds[port] = 108;
        ep_state = EP_DONE;
      }
      else
        ep_state = EP_IGNORE;
      break;

    case EP_M11:
      if (c == '2') {
        ep_cmds[port] = 112;
        ep_state = EP_DONE;
      }
      else
        ep_state = EP_IGNORE;
      break;

    case EP_M4:
      ep_state = (c == '1') ? EP_M41 : EP_IGNORE;
      break;

    case EP_M41:
      if (c == '0') {
        ep_cmds[port] = 410;
        ep_state = EP_DONE;
      }
      else
        ep_state = EP_IGNORE;
      break;

    case EP_DONE:
      // "M1120" is something else, "M112 " or "M112*37" is ours.
      ep_state = (c == ' ' || c == '*') ? EP_TAIL : EP_IGNORE;
      break;

    case EP_TAIL:
      // Parameters (M876 S0) and the checksum: wait for EOL.
      if (c == 'S')
        ep_s_seen[port] = 1;
      break;
    default:                      // EP_IGNORE: wait for EOL.
      break;
  }
  ep_states[port] = ep_state;
}

/**
  Finish M410 in main loop context: take the position where the steppers
  stopped as the new position and drop all queued moves. clock_poll() is
  never called from within dda_create(), so startpoint is consistent here.
*/
void emergency_poll(void) {
  enum axis_e i;

  if ( ! quickstop_pending)
    return;
  quickstop_pending = 0;

  ATOMIC_START();
    TIM5->DIER = 0;
    NVIC_ClearPendingIRQ(TIM5_IRQn);
    update_current_position();
    queue_flush();
    unstep();
  ATOMIC_END();
  endstops_off();

  for (i = X; i < E; i++) {
    startpoint.axis[i] = current_position.axis[i];
    if ( ! next_target.option_all_relative)
      next_target.target.axis[i] = current_position.axis[i];
  }
  if (next_target.option_all_relative || next_target.option_e_relative) {
    startpoint.axis[E] = 0;
  }
  else {
    startpoint.axis[E] = current_position.axis[E];
    next_target.target.axis[E] = current_position.axis[E];
  }
  dda_new_startpoint();
  quickstops++;
}
