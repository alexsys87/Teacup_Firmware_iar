/** \file
  \brief DDA Queue - manage the move queue
*/

#include "dda_queue.h"

#include <string.h>

#include "config_wrapper.h"
#include "timer.h"
#include "serial.h"
#include "temp.h"
#include "delay.h"
#include "sersendf.h"
#include "clock.h"
#include "cpu.h"
#include "babystep.h"

/**
  Movebuffer head pointer. Points to the last move in the queue. This variable
  is used both in and out of interrupts, but is only written outside of
  interrupts.
*/
static volatile uint_fast8_t mb_head = 0;

/// movebuffer tail pointer. Points to the currently executing move
/// this variable is read/written both in and out of interrupts.
volatile uint_fast8_t mb_tail = 0;

/// move buffer.
/// holds move queue
/// contents are read/written both in and out of interrupts, but
/// once writing starts in interrupts on a specific slot, the
/// slot will only be modified in interrupts until the slot is
/// is no longer live.
/// The size does not need to be a power of 2 anymore!
DDA movebuffer[MOVEBUFFER_SIZE];

/**
  Pointer to the currently ongoing movement, or NULL, if there's no movement
  ongoing. Actually a cache of movebuffer[mb_tail].
*/
DDA * volatile mb_tail_dda;

/// Find the next DDA index after 'x', where 0 <= x < MOVEBUFFER_SIZE
#define MB_NEXT(x) ((x) < MOVEBUFFER_SIZE - 1 ? (x) + 1 : 0)

/// check if the queue is completely full
uint_fast8_t queue_full(void) {
  uint_fast8_t head = mb_head;
  uint_fast8_t tail = mb_tail;

  return MB_NEXT(head) == tail;
}

/// number of free slots in the queue
uint_fast8_t queue_free(void) {
  uint_fast8_t head = mb_head;
  uint_fast8_t tail = mb_tail;
  uint_fast8_t used = (head >= tail) ? head - tail
                                     : head + MOVEBUFFER_SIZE - tail;

  return (MOVEBUFFER_SIZE - 1) - used;
}

// -------------------------------------------------------
// This is the one function called by the timer interrupt.
// It calls a few other functions, though.
// -------------------------------------------------------
/// Take a step or go to the next move.
TEACUP_HOT
TEACUP_STEP_RAMFUNC void queue_step(void) {
  DDA *dda = mb_tail_dda;

  // A move stopped by an endstop in dda_clock() isn't live any longer,
  // don't step it.
  if (dda != NULL && dda->live)
    dda_step(dda);

  /**
    Start the next move if this one is done and another one is available.

    This needs no atomic protection, because we're in an interrupt already.
  */
  if (dda == NULL || ! dda->live) {
    uint_fast8_t tail = mb_tail;
    uint_fast8_t head = mb_head;

    if (tail != head) {
      uint_fast8_t t = MB_NEXT(tail);

      mb_tail = t;
      dda = &movebuffer[t];
      mb_tail_dda = dda;
      dda_start(dda);
    }
    else {
      mb_tail_dda = NULL;
    }
  }
}

/// add a move to the movebuffer
/// \note this function waits for space to be available if necessary, check queue_full() first if waiting is a problem
/// This is the only function that modifies mb_head and it always called from outside an interrupt.
void enqueue_home(TARGET *t, uint8_t endstop_check, uint8_t endstop_stop_cond) {
  uint_fast8_t h;
  DDA *new_movebuffer;

  // don't call this function when the queue is full, but just in case, wait for a move to complete and free up the space for the passed target.
  // Keep the clock running while waiting: watchdog, temperatures, M410.
  while (queue_full())
    clock_poll();

  h = MB_NEXT(mb_head);
  new_movebuffer = &(movebuffer[h]);

  // Initialise queue entry to a known state. This also clears flags like
  // dda->live, dda->done and dda->wait_for_temp.
  new_movebuffer->allflags = 0;

  new_movebuffer->endstop_check = endstop_check;
  new_movebuffer->endstop_stop_cond = endstop_stop_cond;
  dda_create(new_movebuffer, t);

  /**
    It's pointless to queue up movements which don't actually move the stepper,
    e.g. pure velocity changes or movements shorter than a single motor step.

    That said, accept movements which do move the steppers by forwarding
    mb_head. Also kick off movements if it's the first movement after a pause.
  */
  if ( ! new_movebuffer->nullmove) {
    // make certain all writes to global memory
    // are flushed before modifying mb_head.
    __DMB();

    mb_head = h;

    if (mb_tail_dda == NULL) {
      /**
        Go to the next move.

        This is the version used from outside interrupts. The in-interrupt
        version is inlined (and simplified) in queue_step(). The step
        interrupt is idle here, so there's no race.
      */
      timer_reset();
      mb_tail = h;        // Valid ONLY if the queue was empty before!
      mb_tail_dda = new_movebuffer; // Dito!
      dda_start(new_movebuffer);
    }
  }
}

/// DEBUG - print queue.
/// Qt/hs format, t is tail, h is head, s is F/full, E/empty or neither
void print_queue(void) {
  uint_fast8_t tail = mb_tail;
  uint_fast8_t head = mb_head;
  char state = queue_full() ? 'F' : (mb_tail_dda ? ' ' : 'E');

  sersendf_P(("Queue: %d/%d%c\n"), tail, head, state);
}

/// dump queue for emergency stop and quickstop.
/// Call with interrupts disabled and the step interrupt stopped.
/// \todo effect on startpoint is undefined!
void queue_flush(void) {
  uint_fast8_t head = mb_head;
  uint_fast8_t i;

  // Mark all moves as finished, so lookahead doesn't join the next move
  // with one of them.
  for (i = 0; i < MOVEBUFFER_SIZE; i++) {
    movebuffer[i].live = 0;
    movebuffer[i].done = 1;
  }
  mb_tail = head;
  mb_tail_dda = NULL;
}

/// wait for queue to empty, including babysteps
void queue_wait(void) {
  // Babysteps are movements, too (homing, probing, M400 wait for them).
  while (mb_tail_dda != NULL || babystep_busy())
    clock_poll();
}
