/** \file
  \brief Firmware retract (G10 / G11, M207 / M208), see retract.h.

  The slicer sends G10 / G11 instead of E moves (PrusaSlicer "Use firmware
  retraction"), length and speeds come from M207 / M208 and can be changed
  while printing.

  The retract is hidden from the G-code coordinates, like in Marlin: E
  moves relative, then the E coordinate of the queue end is restored, so
  absolute E of the following moves continues as if nothing happened. The
  Z lift is part of the Z correction, all moves keep it until G11.
*/

#include "retract.h"

#ifdef FIRMWARE_RETRACT

#include "dda.h"
#include "dda_queue.h"
#include "clock.h"
#include "settings.h"
#include "bed_leveling.h"

int32_t retract_hop_um;
static uint8_t retracted;

/// Queue a move, waiting for queue space; the queue end keeps its E, F.
static void queue_move(TARGET *t) {
  TARGET saved = startpoint;

  while (queue_full())
    clock_poll();
  enqueue(t);
  startpoint.axis[E] = saved.axis[E];
  startpoint.e_relative = saved.e_relative;
  startpoint.F = saved.F;
}

/// E by 'de' um (filament, no M221 flow) at 'feed' mm/min.
static void e_move(int32_t de, uint32_t feed) {
  TARGET t = startpoint;

  if (de == 0)
    return;
  t.axis[E] = de;
  t.e_relative = 1;
  t.e_multiplier = 256;
  t.f_multiplier = 256;
  t.F = feed ? feed : 1;
  queue_move(&t);
}

/// New Z lift, the move to it right away (X, Y, E stay).
static void hop(int32_t um) {
  TARGET t = startpoint;

  if (um == retract_hop_um)
    return;
  retract_hop_um = um;
  t.axis[E] = 0;
  t.e_relative = 1;
  t.f_multiplier = 256;
  t.F = settings.max_feedrate[Z];
  queue_move(&t);
}

void retract_do(void) {
  if (retracted)
    return;
  e_move(-(int32_t)settings.retract_length, settings.retract_feedrate);
  if (settings.retract_zlift > 0)
    hop(settings.retract_zlift);
  retracted = 1;
}

void recover_do(void) {
  if ( ! retracted)
    return;
  hop(0);
  e_move((int32_t)settings.retract_length + settings.recover_extra,
         settings.recover_feedrate);
  retracted = 0;
}

uint8_t retract_active(void) {
  return retracted;
}

void retract_reset(void) {
  retracted = 0;
  if (retract_hop_um) {
    retract_hop_um = 0;
    zcorr_sync_logical();
  }
}

#endif /* FIRMWARE_RETRACT */
