/** \file
  \brief Top level configuration: selects controller board and printer.

  The board file is chosen by the device symbol set in the IAR project
  options (STM32F401xC / STM32F401xE / STM32F411xE), so both project
  configurations share this file.
*/

#if defined TEACUP_RENODE_TEST
  // Configuration used by the Renode emulation tests in test/.
  #include "../test/config.renode_test.h"
#else
  /*
    Prusa i3 Steel on a Black Pill (F401 or F411, chosen by F_CPU in the
    board file). Generic examples: board.blackpill_stm32f401.h,
    board.blackpill_stm32f411.h with printer.mendel.h.
  */
  #include "board.blackpill_p3steel.h"
  #include "printer.p3steel.h"
#endif
