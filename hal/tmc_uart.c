/** \file
  \brief Single wire UART to TMC2208 / TMC2209 stepper drivers (TMC_UART).

  USART1, USART2 or USART6 in half duplex mode (CR3.HDSEL): TX and RX on
  the TX pin, which is open drain with the internal pull-up (the drivers
  pull PDN_UART up as well). The receiver sees our own transmission (echo),
  core/tmc.c skips it.

  USART2 on PA2 is the host UART of the P3 Steel board: TMC_UART 2 needs
  NO_SERIAL_UART (host on USB only).
*/

#include "tmc_uart.h"

#ifdef TMC_UART

#include "arch.h"
#include "pinio.h"

#ifndef TMC_UART_BAUD
  #define TMC_UART_BAUD 115200UL
#endif

#if TMC_UART == 1
  #define TMC_USART         USART1
  #define TMC_AF            7
  #define TMC_PCLK          (F_CPU)             // APB2
  #define TMC_CLOCK_ON()    (RCC->APB2ENR |= RCC_APB2ENR_USART1EN)
#elif TMC_UART == 2
  #define TMC_USART         USART2
  #define TMC_AF            7
  #define TMC_PCLK          (F_CPU / 2)         // APB1
  #define TMC_CLOCK_ON()    (RCC->APB1ENR |= RCC_APB1ENR_USART2EN)
  #ifndef NO_SERIAL_UART
    #error TMC_UART 2 uses the host UART: define NO_SERIAL_UART (USB host only).
  #endif
#elif TMC_UART == 6
  #define TMC_USART         USART6
  #define TMC_AF            8
  #define TMC_PCLK          (F_CPU)             // APB2
  #define TMC_CLOCK_ON()    (RCC->APB2ENR |= RCC_APB2ENR_USART6EN)
#else
  #error TMC_UART must be 1, 2 or 6.
#endif

#if ! defined NO_SERIAL_UART && defined SERIAL_UART && SERIAL_UART == TMC_UART
  #error TMC_UART and SERIAL_UART use the same USART.
#endif

#ifndef TMC_UART_TX_PIN
  #error TMC_UART needs TMC_UART_TX_PIN.
#endif

void tmc_uart_init(void) {
  GPIO_TypeDef *port = PIN_PORT(TMC_UART_TX_PIN);
  uint32_t n = PIN_NUM(TMC_UART_TX_PIN);

  TMC_CLOCK_ON();
  (void)TMC_USART->SR;                      // Delay after clock enable.

  gpio_af(port, n, TMC_AF);
  port->OTYPER |= 1UL << n;                 // Open drain.
  port->PUPDR = (port->PUPDR & ~(3UL << (2 * n))) | (1UL << (2 * n));
  gpio_mode(port, n, GPIO_MODE_AF);

  TMC_USART->CR1 = 0;
  TMC_USART->BRR = (TMC_PCLK + TMC_UART_BAUD / 2) / TMC_UART_BAUD;
  TMC_USART->CR2 = 0;
  TMC_USART->CR3 = USART_CR3_HDSEL;
  TMC_USART->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

int16_t tmc_uart_getc(void) {
  uint32_t sr = TMC_USART->SR;

  // Reading DR after SR also clears an overrun.
  if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
    uint32_t dr = TMC_USART->DR;
    if (sr & USART_SR_RXNE)
      return (int16_t)(dr & 0xFFU);
  }
  return -1;
}

void tmc_uart_flush_rx(void) {
  while (tmc_uart_getc() >= 0)
    ;
}

void tmc_uart_send(const uint8_t *data, uint8_t n) {
  while (n--) {
    while ( ! (TMC_USART->SR & USART_SR_TXE))
      ;
    TMC_USART->DR = *data++;
  }
  while ( ! (TMC_USART->SR & USART_SR_TC))
    ;
}

#endif /* TMC_UART */
