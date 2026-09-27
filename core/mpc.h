/** \file
  \brief Model predictive temperature control of the hotend (HOTEND_MPC,
         M306), like Marlin's MPCTEMP.
*/

#ifndef _MPC_H
#define _MPC_H

#include <stdint.h>
#include "config_wrapper.h"

/// Indices of settings.mpc[] (M306 letters).
enum mpc_param {
  MPC_P = 0,    ///< Heater power, W.
  MPC_C,        ///< Block heat capacity, J/K.
  MPC_R,        ///< Sensor responsiveness, 1/s.
  MPC_A,        ///< Ambient heat transfer, fan off, W/K.
  MPC_F,        ///< Ambient heat transfer, fan at full speed, W/K.
  MPC_H,        ///< Filament heat capacity, J/K per mm.
  MPC_PARAMS
};

#ifdef HOTEND_MPC

/// Compiled-in model values (MPC_HEATER_POWER etc.) into settings.mpc.
void mpc_defaults(void);

/// Forget the model state (target 0, heater off).
void mpc_reset(void);

/**
  One control step, every 100 ms. temp: measured temperature, target:
  wanted temperature, C. \return heater output 0..255.
*/
uint8_t mpc_run(float temp, float target);

/// M306 T: measure the model (heater power from settings.mpc[MPC_P]).
void mpc_autotune(uint16_t target);

/// M306 report line.
void mpc_report(void);

#endif /* HOTEND_MPC */

#endif /* _MPC_H */
