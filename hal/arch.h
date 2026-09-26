/** \file
  \brief Chip and compiler abstraction for STM32F401 / STM32F411.

  Every source file which touches hardware includes this header. It pulls in
  the CMSIS device header and provides the few compiler specific helpers the
  firmware needs, so the code builds with IAR EWARM (primary target) as well
  as with arm-none-eabi-gcc (used for cross-checks only).
*/

#ifndef _ARCH_H
#define _ARCH_H

#include <stdint.h>

/* Device selection. Exactly one of these must come from the project options
   (IAR: Project > Options > C/C++ Compiler > Preprocessor > Defined symbols). */
#if ! defined STM32F401xC && ! defined STM32F401xE && ! defined STM32F411xE
  #error Define STM32F401xC, STM32F401xE or STM32F411xE in the project options.
#endif

#include "stm32f4xx.h"

#if defined STM32F411xE
  #define CPU_STM32F411
#else
  #define CPU_STM32F401
#endif

/**
  Force inlining of small helpers. IAR honours plain 'static inline' at
  medium/high optimisation; GCC gets the attribute on top.
*/
#if defined(__GNUC__) && ! defined(__ICCARM__)
  #define TEACUP_INLINE     static inline __attribute__((always_inline))
#else
  #define TEACUP_INLINE     static inline
#endif

/// Keep a function out of line (e.g. as an address for test hooks).
#if defined(__GNUC__) && ! defined(__ICCARM__)
  #define TEACUP_NOINLINE   __attribute__((noinline))
#else
  #define TEACUP_NOINLINE   _Pragma("inline=never")
#endif

/// Silence 'unused parameter' warnings.
#define TEACUP_UNUSED(x)  ((void)(x))

/**
  Single precision square root. GCC emits one VSQRT.F32 instruction, IAR
  uses sqrtf() from its library (also VSQRT based with the FPU enabled).
*/
#if defined(__ICCARM__)
  #include <math.h>
  #define teacup_sqrtf(x) sqrtf(x)
#else
  #define teacup_sqrtf(x) __builtin_sqrtf(x)
#endif

/**
  Function executed from RAM, e.g. while the Flash is being erased.
  IAR places __ramfunc code in a readwrite section, which the ICF
  initializes by copy. GCC needs the section copied with .data.
*/
#if defined(__ICCARM__)
  #define TEACUP_RAMFUNC  __ramfunc
#else
  #define TEACUP_RAMFUNC  __attribute__((section(".ramfunc"), noinline, long_call))
#endif

/**
  Time critical function (step path, interrupts, move planning). Put
  TEACUP_HOT in front of the definition:

    TEACUP_HOT
    void TIM5_IRQHandler(void) { ... }

  IAR: "#pragma optimize=speed" for this function. The pragma can't raise
  the optimization level above the one set for the file, it only changes
  the goal. So set the project (or at least these files) to level High,
  then the rest can stay at High/Size while these run at High/Speed. See
  README, "IAR: optimization".

  GCC: 'hot' attribute. The cross-check Makefile compiles the files with
  such functions at -O2 instead, see FAST_SRC there.
*/
#if defined(__ICCARM__)
  #define TEACUP_HOT      _Pragma("optimize=speed")
#elif defined(__GNUC__)
  #define TEACUP_HOT      __attribute__((hot))
#else
  #define TEACUP_HOT
#endif

/**
  STEP_CODE_IN_RAM: the step interrupt path (TIM5 handler, queue_step(),
  dda_step(), dda_start(), timer_set()) runs from RAM instead of Flash.
  Deterministic timing without Flash wait states and ART cache misses, but
  instruction fetches then share the bus with data accesses. Measure with
  M9001 whether it helps on your board.
*/
#ifdef STEP_CODE_IN_RAM
  #define TEACUP_STEP_RAMFUNC TEACUP_RAMFUNC
#else
  #define TEACUP_STEP_RAMFUNC
#endif

#ifndef MASK
  /// Bit mask for bit number PIN.
  #define MASK(PIN)       (1UL << (PIN))
#endif

#endif /* _ARCH_H */
