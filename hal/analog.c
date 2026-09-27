/** \file
  \brief Analog subsystem, STM32F4 ADC1 + DMA2 Stream0.

  All analog temperature sensors (TT_THERMISTOR, TT_AD595) get converted in
  one scan sequence, ADC runs continuously and DMA writes the results in
  circular mode into adc_buffer (OVERSAMPLE rows of all scanned channels). No interrupts needed,
  analog_read() always averages the latest OVERSAMPLE samples.

  Resolution is 12 bits (0..ANALOG_MAX), converted to temperatures with
  the Steinhart-Hart equation in temp.c.
*/

#include "analog.h"
#include "arch.h"
#include "pinio.h"
#include "temp.h"
#include "delay.h"

#ifndef OVERSAMPLE
  #define OVERSAMPLE 6
#endif

/* Which sensor types are read by the ADC. */
#define ANALOG_TYPE_TT_THERMISTOR 1
#define ANALOG_TYPE_TT_AD595      1
#define ANALOG_TYPE_TT_MAX6675    0
#define ANALOG_TYPE_TT_MCP3008    0
#define ANALOG_TYPE_TT_MAX31865   0
#define ANALOG_TYPE_TT_DUMMY      0
#define ANALOG_TYPE_TT_PT100      0
#define ANALOG_TYPE_TT_INTERCOM   0
#define IS_ANALOG(type)           PIN_CAT(ANALOG_TYPE_, type)

/* Compile time check: analog sensors must be on ADC capable pins. */
#undef DEFINE_TEMP_SENSOR
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
  typedef char analog_pin_check_ ## name \
    [(IS_ANALOG(type) && PIN_ADC(pin) == ADC_NONE) ? -1 : 1];
#include "config_wrapper.h"
#undef DEFINE_TEMP_SENSOR

/* ADC channel of each temperature sensor, ADC_NONE for non-analog ones. */
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
  (IS_ANALOG(type) ? PIN_ADC(pin) : ADC_NONE),
static const uint8_t sensor_channel[NUM_TEMP_SENSORS] = {
  #include "config_wrapper.h"
};
#undef DEFINE_TEMP_SENSOR

/* Pin of each temperature sensor. */
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) PIN_ID(pin),
static const uint8_t sensor_pin[NUM_TEMP_SENSORS] = {
  #include "config_wrapper.h"
};
#undef DEFINE_TEMP_SENSOR

/// Position of each sensor in the scan sequence, 0xFF = not scanned.
static uint8_t sensor_slot[NUM_TEMP_SENSORS];
/// Number of conversions in the scan sequence.
static uint8_t num_slots;

/// DMA target: OVERSAMPLE rows of num_slots conversions each.
static volatile uint16_t adc_buffer[OVERSAMPLE * NUM_TEMP_SENSORS];

/// (Re)start ADC and DMA.
static void adc_start(void) {
  ADC1->CR2 = 0;                                // ADC off, DMA off.
  DMA2_Stream0->CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream0->CR & DMA_SxCR_EN)
    ;
  DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |
                DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

  DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
  DMA2_Stream0->M0AR = (uint32_t)&adc_buffer[0];
  DMA2_Stream0->NDTR = (uint32_t)num_slots * OVERSAMPLE;
  DMA2_Stream0->FCR  = 0;                       // Direct mode.
  DMA2_Stream0->CR   = (0UL << DMA_SxCR_CHSEL_Pos) |  // Channel 0 = ADC1.
                       DMA_SxCR_PL_1 |          // High priority.
                       DMA_SxCR_MSIZE_0 |       // 16 bit memory.
                       DMA_SxCR_PSIZE_0 |       // 16 bit peripheral.
                       DMA_SxCR_MINC |
                       DMA_SxCR_CIRC;
  DMA2_Stream0->CR  |= DMA_SxCR_EN;

  ADC1->SR = 0;
  ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
  delay_us(5);                                  // tSTAB.
  ADC1->CR2 |= ADC_CR2_SWSTART;
}

void analog_init(void) {
  uint8_t i;

  num_slots = 0;
  for (i = 0; i < NUM_TEMP_SENSORS; i++) {
    sensor_slot[i] = 0xFF;
    if (sensor_channel[i] != ADC_NONE) {
      gpio_pull(PIN_ID_PORT(sensor_pin[i]), PIN_ID_NUM(sensor_pin[i]),
                GPIO_PULL_NONE);
      gpio_mode(PIN_ID_PORT(sensor_pin[i]), PIN_ID_NUM(sensor_pin[i]),
                GPIO_MODE_ANALOG);
      sensor_slot[i] = num_slots++;
    }
  }

  if (num_slots == 0)
    return;

  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  (void)RCC->AHB1ENR;

  // ADC clock = PCLK2 / 4 = 21 / 24 / 25 MHz (max 36 MHz).
  ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;

  ADC1->CR1 = ADC_CR1_SCAN;                     // 12 bit, scan mode.
  ADC1->SMPR1 = 0;
  ADC1->SMPR2 = 0;
  ADC1->SQR1 = (uint32_t)(num_slots - 1) << ADC_SQR1_L_Pos;
  ADC1->SQR2 = 0;
  ADC1->SQR3 = 0;

  for (i = 0; i < NUM_TEMP_SENSORS; i++) {
    uint8_t ch = sensor_channel[i];
    uint8_t slot = sensor_slot[i];

    if (slot == 0xFF)
      continue;

    // 480 cycles sampling time, thermistor dividers are high impedance.
    if (ch >= 10)
      ADC1->SMPR1 |= 7UL << (3 * (ch - 10));
    else
      ADC1->SMPR2 |= 7UL << (3 * ch);

    if (slot < 6)
      ADC1->SQR3 |= (uint32_t)ch << (5 * slot);
    else if (slot < 12)
      ADC1->SQR2 |= (uint32_t)ch << (5 * (slot - 6));
    else
      ADC1->SQR1 |= (uint32_t)ch << (5 * (slot - 12));
  }

  adc_start();
}

uint16_t analog_read(uint8_t index) {
  uint32_t sum = 0, min = 0xFFFF, max = 0, v;
  uint8_t i, slot;

  if (index >= NUM_TEMP_SENSORS || (slot = sensor_slot[index]) == 0xFF)
    return 0;

  // Overrun would stop the ADC for good, restart it.
  if (ADC1->SR & ADC_SR_OVR)
    adc_start();

  for (i = 0; i < OVERSAMPLE; i++) {
    v = adc_buffer[i * num_slots + slot];
    sum += v;
    if (v < min) min = v;
    if (v > max) max = v;
  }

  #if OVERSAMPLE > 2
    return (uint16_t)((sum - min - max) / (OVERSAMPLE - 2));
  #else
    return (uint16_t)(sum / OVERSAMPLE);
  #endif
}
