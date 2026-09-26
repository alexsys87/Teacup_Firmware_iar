#ifndef _DDA_QUEUE
#define _DDA_QUEUE

#include "arch.h"
#include "dda.h"
#include "timer.h"

/*
  variables
*/

// this is the ringbuffer that holds the current and pending moves.
extern volatile uint_fast8_t mb_tail;
extern DDA movebuffer[MOVEBUFFER_SIZE];
/// Currently executing move, NULL if idle. Written in the step interrupt.
extern DDA * volatile mb_tail_dda;

/*
  methods
*/

// queue status methods
uint_fast8_t queue_full(void);

/// Number of free slots in the movement queue.
uint_fast8_t queue_free(void);

// take one step
void queue_step(void);

// add a new target to the queue
void enqueue_home(TARGET *t, uint8_t endstop_check, uint8_t endstop_stop_cond);

TEACUP_INLINE void enqueue(TARGET *t) {
  enqueue_home(t, 0, 0);
}

// print queue status
void print_queue(void);

// flush the queue for eg; emergency stop
void queue_flush(void);

// wait for queue to empty
void queue_wait(void);

#endif /* _DDA_QUEUE */
