/** \file
  \brief Configuration for the Renode emulation tests (test/renode).

  Same pins as the Black Pill board file, but a dummy temperature sensor
  (Renode has no STM32F4 ADC model), ramping acceleration with lookahead and
  short residency time.
*/

#include "board.renode_test.h"

#include "printer.renode_test.h"
