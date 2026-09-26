/** \file
  \brief CPU initialisation for STM32F401 / STM32F411.

  Sets up the PLL from the board crystal (HSE_CLOCK_HZ) to run at F_CPU,
  with automatic fallback to the internal 16 MHz oscillator. Also switches
  on GPIO clocks, the DWT cycle counter (used by delays and step pulses)
  and sets interrupt priority grouping.
*/

#include "cpu.h"
#include "config_wrapper.h"

uint8_t cpu_clock_source = CPU_CLOCK_FAILED;
uint32_t cpu_reset_flags = 0;

#if defined CPU_STM32F411
  #if F_CPU > 100000000UL
    #error STM32F411 runs at 100 MHz max.
  #endif
#else
  #if F_CPU > 84000000UL
    #error STM32F401 runs at 84 MHz max.
  #endif
#endif

/* PLL: VCO input is always 1 MHz, VCO = F_CPU * PLL_P, USB = VCO / PLL_Q. */
#if F_CPU == 84000000UL
  #define PLL_P 4
  #define PLL_Q 7
#elif F_CPU == 96000000UL
  #define PLL_P 2
  #define PLL_Q 4
#elif F_CPU == 100000000UL
  #define PLL_P 2
  #define PLL_Q 5             /* 40 MHz, USB not usable at 100 MHz. */
#elif F_CPU == 48000000UL
  #define PLL_P 4
  #define PLL_Q 4
#else
  #error F_CPU must be 48000000, 84000000, 96000000 or 100000000.
#endif

#define PLL_VCO_MHZ     ((F_CPU / 1000000UL) * PLL_P)

#if (HSE_CLOCK_HZ % 1000000UL) != 0 || HSE_CLOCK_HZ < 4000000UL || HSE_CLOCK_HZ > 26000000UL
  #error HSE_CLOCK_HZ must be an integer number of MHz between 4 and 26.
#endif

#define PLLCFGR_VALUE(m, src) ((uint32_t)(m) | \
          ((uint32_t)PLL_VCO_MHZ << RCC_PLLCFGR_PLLN_Pos) | \
          ((uint32_t)(PLL_P / 2 - 1) << RCC_PLLCFGR_PLLP_Pos) | \
          (src) | \
          ((uint32_t)PLL_Q << RCC_PLLCFGR_PLLQ_Pos))

/* Flash wait states for 2.7..3.6 V supply, RM0368 / RM0383 table 5. */
#if defined CPU_STM32F411
  #if F_CPU <= 30000000UL
    #define FLASH_WS 0
  #elif F_CPU <= 64000000UL
    #define FLASH_WS 1
  #elif F_CPU <= 90000000UL
    #define FLASH_WS 2
  #else
    #define FLASH_WS 3
  #endif
  /* Voltage scale 1 is required above 84 MHz. */
  #if F_CPU > 84000000UL
    #define PWR_VOS_VALUE  (PWR_CR_VOS_1 | PWR_CR_VOS_0)
  #else
    #define PWR_VOS_VALUE  PWR_CR_VOS_1
  #endif
#else
  #define FLASH_WS         ((F_CPU - 1) / 30000000UL)
  #define PWR_VOS_VALUE    PWR_CR_VOS_1
#endif

/** Wait for a flag with timeout. \return 1 if the flag came up. */
static uint8_t wait_flag(volatile uint32_t *reg, uint32_t mask,
                         uint32_t value, uint32_t timeout) {
  while (timeout--) {
    if ((*reg & mask) == value)
      return 1;
  }
  return 0;
}

void cpu_init(void) {
  volatile uint32_t dummy;

  // Remember and clear the reset cause flags.
  cpu_reset_flags = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;

  /* Make sure we run from HSI with the PLL off, e.g. after a bootloader. */
  RCC->CR |= RCC_CR_HSION;
  wait_flag(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY, 100000);
  RCC->CFGR &= ~RCC_CFGR_SW;
  wait_flag(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI, 100000);
  RCC->CR &= ~RCC_CR_PLLON;
  wait_flag(&RCC->CR, RCC_CR_PLLRDY, 0, 100000);

  /* Regulator voltage scaling. */
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  dummy = RCC->APB1ENR;
  PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_VOS_VALUE;

  /* Crystal, with fallback to the internal RC oscillator. */
  RCC->CR |= RCC_CR_HSEON;
  if (wait_flag(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY, 500000)) {
    RCC->PLLCFGR = PLLCFGR_VALUE(HSE_CLOCK_HZ / 1000000UL,
                                 RCC_PLLCFGR_PLLSRC_HSE);
    cpu_clock_source = CPU_CLOCK_HSE;
  }
  else {
    RCC->CR &= ~RCC_CR_HSEON;
    RCC->PLLCFGR = PLLCFGR_VALUE(16, RCC_PLLCFGR_PLLSRC_HSI);
    cpu_clock_source = CPU_CLOCK_HSI;
  }

  RCC->CR |= RCC_CR_PLLON;
  if (wait_flag(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY, 500000)) {
    /* Flash wait states first, then caches and prefetch. */
    FLASH->ACR = FLASH_ACR_ICRST | FLASH_ACR_DCRST;
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN |
                 (FLASH_WS << FLASH_ACR_LATENCY_Pos);
    wait_flag(&FLASH->ACR, FLASH_ACR_LATENCY, FLASH_WS << FLASH_ACR_LATENCY_Pos,
              100000);

    /* AHB = F_CPU, APB1 = F_CPU / 2 (max 42/50 MHz), APB2 = F_CPU. */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2))
                | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    wait_flag(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL, 100000);
    SystemCoreClock = F_CPU;
  }
  else {
    cpu_clock_source = CPU_CLOCK_FAILED;
    SystemCoreClock = 16000000UL;
  }

  /* GPIO clocks. */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                  RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
                  RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOHEN;
  dummy = RCC->AHB1ENR;
  (void)dummy;

  /* Cycle counter for delays and step pulse timing. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* FPU: automatic and lazy context saving. An interrupt which doesn't
     use the FPU doesn't pay for stacking the FPU registers. */
  FPU->FPCCR |= FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk;

  /* 4 bits preemption priority, no subpriority. */
  NVIC_SetPriorityGrouping(3);

  /* Keep the watchdog and the step timer frozen while halted in debugger. */
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP | DBGMCU_APB1_FZ_DBG_TIM5_STOP;
}

/**
  Fault handlers. A fault leaves hardware PWM running, a heater could stay on
  forever. Resetting returns all pins to their inputs-only reset state.
  Define DEBUG_FAULT_HALT to stop in the handler for debugging instead.
*/
static void fault(void) {
  #ifdef DEBUG_FAULT_HALT
    __disable_irq();
    for (;;) ;
  #else
    NVIC_SystemReset();
  #endif
}

void HardFault_Handler(void)  { fault(); }
void MemManage_Handler(void)  { fault(); }
void BusFault_Handler(void)   { fault(); }
void UsageFault_Handler(void) { fault(); }
