/** \file
  \brief SPI master for STM32F4, polled, 8 bit, mode 0 (spi_mode() switches
  for one device, e.g. MAX31865 in mode 1).
*/

#include "spi.h"

#ifdef SPI

#include "arch.h"

#ifndef SPI_INSTANCE
  #define SPI_INSTANCE 2
  #define SPI_SCK_PIN  PB_13
  #define SPI_MISO_PIN PB_14
  #define SPI_MOSI_PIN PB_15
#endif

#if SPI_INSTANCE == 1
  #define SPIx            SPI1
  #define SPI_AF          5
  #define SPI_PCLK        (F_CPU)               // APB2
  #define SPI_CLOCK_ON()  (RCC->APB2ENR |= RCC_APB2ENR_SPI1EN)
#elif SPI_INSTANCE == 2
  #define SPIx            SPI2
  #define SPI_AF          5
  #define SPI_PCLK        (F_CPU / 2)           // APB1
  #define SPI_CLOCK_ON()  (RCC->APB1ENR |= RCC_APB1ENR_SPI2EN)
#elif SPI_INSTANCE == 3
  #define SPIx            SPI3
  #define SPI_AF          6
  #define SPI_PCLK        (F_CPU / 2)           // APB1
  #define SPI_CLOCK_ON()  (RCC->APB1ENR |= RCC_APB1ENR_SPI3EN)
#else
  #error SPI_INSTANCE must be 1, 2 or 3.
#endif

/// Baud rate divider exponent: f = PCLK / 2^(BR + 1).
static uint32_t spi_br_for(uint32_t max_hz) {
  uint32_t br = 0;

  while (br < 7 && (SPI_PCLK >> (br + 1)) > max_hz)
    br++;
  return br << SPI_CR1_BR_Pos;
}

void spi_init(void) {
  SPI_CLOCK_ON();

  SET_AF(SPI_SCK_PIN, SPI_AF);
  SET_AF(SPI_MISO_PIN, SPI_AF);
  SET_AF(SPI_MOSI_PIN, SPI_AF);
  PULLUP_ON(SPI_MISO_PIN);

  #ifdef SD_CARD_SELECT_PIN
    WRITE(SD_CARD_SELECT_PIN, 1);
    SET_OUTPUT(SD_CARD_SELECT_PIN);
  #endif
  #ifdef TEMP_MCP3008
    WRITE(MCP3008_SELECT_PIN, 1);
    SET_OUTPUT(MCP3008_SELECT_PIN);
  #endif

  // Master, mode 0, software slave select, MSB first, 8 bit.
  SPIx->CR1 = 0;
  SPIx->CR2 = 0;
  SPIx->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | spi_br_for(400000);
  SPIx->CR1 |= SPI_CR1_SPE;
}

uint8_t spi_rw(uint8_t byte) {
  while ( ! (SPIx->SR & SPI_SR_TXE))
    ;
  *(volatile uint8_t *)&SPIx->DR = byte;
  while ( ! (SPIx->SR & SPI_SR_RXNE))
    ;
  return *(volatile uint8_t *)&SPIx->DR;
}

/// Change clock, only while no transfer is ongoing.
static void spi_set_br(uint32_t br) {
  while (SPIx->SR & SPI_SR_BSY)
    ;
  SPIx->CR1 &= ~SPI_CR1_SPE;
  SPIx->CR1 = (SPIx->CR1 & ~SPI_CR1_BR) | br;
  SPIx->CR1 |= SPI_CR1_SPE;
}

void spi_mode(uint8_t mode) {
  uint32_t bits = ((mode & 2) ? SPI_CR1_CPOL : 0) | ((mode & 1) ? SPI_CR1_CPHA : 0);

  while (SPIx->SR & SPI_SR_BSY)
    ;
  SPIx->CR1 &= ~SPI_CR1_SPE;
  SPIx->CR1 = (SPIx->CR1 & ~(SPI_CR1_CPOL | SPI_CR1_CPHA)) | bits;
  SPIx->CR1 |= SPI_CR1_SPE;
}

void spi_speed_100_400(void) {
  spi_set_br(spi_br_for(400000));
}

void spi_speed_max(void) {
  spi_set_br(spi_br_for(12000000));
}

#endif /* SPI */
