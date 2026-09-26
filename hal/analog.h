/** \file
  \brief Analog subsystem: ADC1 in continuous scan mode with DMA.
*/

#ifndef _ANALOG_H
#define _ANALOG_H

#include <stdint.h>

/// Full scale of analog_read(), 12 bit ADC.
#define ANALOG_MAX 4095

/// Start continuous conversion of all analog temperature sensors.
void analog_init(void);

/// Latest averaged 12 bit reading of temperature sensor 'index'.
uint16_t analog_read(uint8_t index);

#endif /* _ANALOG_H */
