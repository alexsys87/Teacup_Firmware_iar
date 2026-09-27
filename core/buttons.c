/** \file
  \brief Buttons on the PCF8574 I/O expander, see buttons.h.

  Each button connects its expander pin (BUTTON_UP_BIT etc.) to GND. The
  pins are read every 10 ms (expander_tick()). A change counts when two
  reads in a row agree (debounce, 10..20 ms).

  Up and down repeat while held: after 400 ms every 100 ms, after 2 s
  every 50 ms with BUTTON_FAST set (the menu changes values in larger
  steps then). OK and back don't repeat.
*/

#include "buttons.h"

#ifdef BUTTONS

#include "expander.h"

#define B_UP    0x01
#define B_DOWN  0x02
#define B_OK    0x04
#define B_BACK  0x08

/// Event queue, power of 2.
#define EVENTS 8
static uint8_t events[EVENTS];
static uint8_t ev_head, ev_tail;

static uint8_t last_raw, stable;
static uint16_t hold_ticks;

static void push(uint8_t e) {
  uint8_t next = (ev_head + 1) & (EVENTS - 1);

  if (next != ev_tail) {
    events[ev_head] = e;
    ev_head = next;
  }
}

/// Map the expander pins (low = pressed) to B_ bits.
static uint8_t read_raw(void) {
  uint8_t in = (uint8_t)~expander_inputs(), r = 0;

  if (in & (1U << BUTTON_UP_BIT))
    r |= B_UP;
  if (in & (1U << BUTTON_DOWN_BIT))
    r |= B_DOWN;
  if (in & (1U << BUTTON_OK_BIT))
    r |= B_OK;
  #ifdef BUTTON_BACK_BIT
    if (in & (1U << BUTTON_BACK_BIT))
      r |= B_BACK;
  #endif
  return r;
}

void buttons_tick(void) {
  uint8_t raw = read_raw(), pressed;

  if (raw != last_raw) {
    last_raw = raw;
    return;                             // Wait for a second equal read.
  }
  pressed = raw & (uint8_t)~stable;
  stable = raw;

  if (pressed & B_UP)
    push(BUTTON_UP);
  if (pressed & B_DOWN)
    push(BUTTON_DOWN);
  if (pressed & B_OK)
    push(BUTTON_OK);
  if (pressed & B_BACK)
    push(BUTTON_BACK);

  // Repeat up / down while one of them is held alone.
  if (stable == B_UP || stable == B_DOWN) {
    if (pressed)
      hold_ticks = 0;
    else if (hold_ticks < 0xFFFF)
      hold_ticks++;
    if (hold_ticks >= 200) {
      if (hold_ticks % 5 == 0)
        push((uint8_t)((stable == B_UP ? BUTTON_UP : BUTTON_DOWN) | BUTTON_FAST));
    }
    else if (hold_ticks >= 40 && hold_ticks % 10 == 0) {
      push(stable == B_UP ? BUTTON_UP : BUTTON_DOWN);
    }
  }
  else {
    hold_ticks = 0;
  }
}

uint8_t buttons_get(void) {
  uint8_t e;

  if (ev_head == ev_tail)
    return BUTTON_NONE;
  e = events[ev_tail];
  ev_tail = (ev_tail + 1) & (EVENTS - 1);
  return e;
}

uint8_t buttons_held(void) {
  return stable;
}

#endif /* BUTTONS */
