/** \file
  \brief Host UART, STM32F4 USART. One of the host ports of serial.c.

  Two implementations with the same interface:

  SERIAL_DMA (default, NO_SERIAL_DMA switches it off):
    Receiving: DMA writes every character into a circular 256 byte buffer,
    no interrupt per character. The IDLE line interrupt (one character time
    after the last one) and the DMA transfer complete interrupt pass new
    characters to the emergency parser (M112, M108, M410) and check the
    XON/XOFF levels.
    Transmitting: the ring buffer is sent in blocks by DMA, one interrupt
    per block instead of one per character.

  Interrupt driven (NO_SERIAL_DMA):
    One interrupt per received and per transmitted character, ring buffers.

  Optional XON/XOFF flow control: XOFF is sent when the receive buffer is
  3/4 full, XON when it drained to 1/4.
*/

#include "uart.h"
#include "serial.h"
#include "arch.h"
#include "cpu.h"
#include "pinio.h"
#include "atomic.h"
#include "emergency_parser.h"

#ifndef NO_SERIAL_UART

#ifndef BAUD
  #define BAUD 115200
#endif

#ifndef SERIAL_UART
  #define SERIAL_UART   2
  #define SERIAL_TX_PIN PA_2
  #define SERIAL_RX_PIN PA_3
#endif

#if SERIAL_UART == 1
  #define UARTx             USART1
  #define UART_IRQn         USART1_IRQn
  #define UART_IRQHandler   USART1_IRQHandler
  #define UART_AF           7
  #define UART_PCLK         (F_CPU)             // APB2
  #define UART_CLOCK_ON()   (RCC->APB2ENR |= RCC_APB2ENR_USART1EN)
#elif SERIAL_UART == 2
  #define UARTx             USART2
  #define UART_IRQn         USART2_IRQn
  #define UART_IRQHandler   USART2_IRQHandler
  #define UART_AF           7
  #define UART_PCLK         (F_CPU / 2)         // APB1
  #define UART_CLOCK_ON()   (RCC->APB1ENR |= RCC_APB1ENR_USART2EN)
#elif SERIAL_UART == 6
  #define UARTx             USART6
  #define UART_IRQn         USART6_IRQn
  #define UART_IRQHandler   USART6_IRQHandler
  #define UART_AF           8
  #define UART_PCLK         (F_CPU)             // APB2
  #define UART_CLOCK_ON()   (RCC->APB2ENR |= RCC_APB2ENR_USART6EN)
#else
  #error SERIAL_UART must be 1, 2 or 6.
#endif

#define ASCII_XOFF 0x13
#define ASCII_XON  0x11

#ifdef SERIAL_DMA

/*
  DMA streams (RM0368 table 27/28, DMA request mapping):
    USART1: RX DMA2 stream 2 ch 4, TX DMA2 stream 7 ch 4
    USART2: RX DMA1 stream 5 ch 4, TX DMA1 stream 6 ch 4
    USART6: RX DMA2 stream 1 ch 5, TX DMA2 stream 6 ch 5
  None of them collides with the ADC (DMA2 stream 0).
*/
#if SERIAL_UART == 1
  #define DMAx              DMA2
  #define DMA_CLOCK_ON()    (RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN)
  #define RX_STREAM         DMA2_Stream2
  #define RX_STREAM_NUM     2
  #define RX_DMA_IRQn       DMA2_Stream2_IRQn
  #define RX_DMA_IRQHandler DMA2_Stream2_IRQHandler
  #define TX_STREAM         DMA2_Stream7
  #define TX_STREAM_NUM     7
  #define TX_DMA_IRQn       DMA2_Stream7_IRQn
  #define TX_DMA_IRQHandler DMA2_Stream7_IRQHandler
  #define DMA_CHANNEL       4UL
#elif SERIAL_UART == 2
  #define DMAx              DMA1
  #define DMA_CLOCK_ON()    (RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN)
  #define RX_STREAM         DMA1_Stream5
  #define RX_STREAM_NUM     5
  #define RX_DMA_IRQn       DMA1_Stream5_IRQn
  #define RX_DMA_IRQHandler DMA1_Stream5_IRQHandler
  #define TX_STREAM         DMA1_Stream6
  #define TX_STREAM_NUM     6
  #define TX_DMA_IRQn       DMA1_Stream6_IRQn
  #define TX_DMA_IRQHandler DMA1_Stream6_IRQHandler
  #define DMA_CHANNEL       4UL
#else
  #define DMAx              DMA2
  #define DMA_CLOCK_ON()    (RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN)
  #define RX_STREAM         DMA2_Stream1
  #define RX_STREAM_NUM     1
  #define RX_DMA_IRQn       DMA2_Stream1_IRQn
  #define RX_DMA_IRQHandler DMA2_Stream1_IRQHandler
  #define TX_STREAM         DMA2_Stream6
  #define TX_STREAM_NUM     6
  #define TX_DMA_IRQn       DMA2_Stream6_IRQn
  #define TX_DMA_IRQHandler DMA2_Stream6_IRQHandler
  #define DMA_CHANNEL       5UL
#endif

/* Interrupt flags of a stream: bit offset within LISR/HISR. */
#define DMA_FLAG_SHIFT(n)   (((n) & 3) == 0 ? 0 : ((n) & 3) == 1 ? 6 : \
                             ((n) & 3) == 2 ? 16 : 22)
#define DMA_TCIF(n)         (1UL << (DMA_FLAG_SHIFT(n) + 5))
#define DMA_ALL_FLAGS(n)    (0x3DUL << DMA_FLAG_SHIFT(n))
#define DMA_ISR(n)          ((n) < 4 ? DMAx->LISR : DMAx->HISR)
#define DMA_IFCR(n)         (*((n) < 4 ? &DMAx->LIFCR : &DMAx->HIFCR))

/// Buffer sizes, powers of 2, 256 max.
#define RX_BUFSIZE 256
#define TX_BUFSIZE 256

static volatile uint8_t rxbuf[RX_BUFSIZE];      // Written by DMA.
static volatile uint8_t txbuf[TX_BUFSIZE];
static uint16_t rxtail;                         // Main loop reads here.
static uint16_t ep_pos;                         // Emergency parser (ISR).
static volatile uint16_t txhead, txtail;
static volatile uint16_t tx_len;                // DMA block in flight, 0 = idle.

#ifdef XONXOFF
  static volatile uint8_t flowchar;             // XON/XOFF to send asap.
  static volatile uint8_t xoff_sent;
#endif

/// Where DMA writes the next character.
static uint16_t rx_head(void) {
  uint16_t ndtr = (uint16_t)RX_STREAM->NDTR;

  return (uint16_t)(RX_BUFSIZE - ndtr) & (RX_BUFSIZE - 1);
}

static uint16_t rx_count(void) {
  uint16_t head = rx_head();

  return (uint16_t)(head - rxtail) & (RX_BUFSIZE - 1);
}

static uint16_t tx_free(void) {
  uint16_t head = txhead;
  uint16_t tail = txtail;

  return (TX_BUFSIZE - 1) - ((uint16_t)(head - tail) & (TX_BUFSIZE - 1));
}

/**
  Start the next DMA block, if the transmitter is idle and there's data.
  Call with the serial interrupts masked.
*/
static void tx_start(void) {
  uint16_t head, tail, len;

  if (tx_len)
    return;

  #ifdef XONXOFF
    if (flowchar) {
      // Flow control characters bypass the buffer. DMA is idle, so the
      // data register is free after at most one character time.
      while ( ! (UARTx->SR & USART_SR_TXE))
        ;
      UARTx->DR = flowchar;
      flowchar = 0;
    }
  #endif

  head = txhead;
  tail = txtail;
  if (head == tail)
    return;

  // One contiguous block, up to the buffer end.
  len = (head > tail) ? (uint16_t)(head - tail) : (uint16_t)(TX_BUFSIZE - tail);
  tx_len = len;

  DMA_IFCR(TX_STREAM_NUM) = DMA_ALL_FLAGS(TX_STREAM_NUM);
  TX_STREAM->M0AR = (uint32_t)&txbuf[tail];
  TX_STREAM->NDTR = len;
  TX_STREAM->CR |= DMA_SxCR_EN;
}

/// Block sent: advance and start the next one.
static void tx_done(void) {
  uint16_t len = tx_len;
  uint16_t tail = txtail;

  txtail = (uint16_t)(tail + len) & (TX_BUFSIZE - 1);
  tx_len = 0;
  tx_start();
}

/**
  Handle a finished TX block by polling the flag. Used where the DMA
  interrupt can't run (interrupts masked, e.g. in printer_kill()).
  Call with the serial interrupts masked.
*/
static void tx_poll(void) {
  if (tx_len && (DMA_ISR(TX_STREAM_NUM) & DMA_TCIF(TX_STREAM_NUM))) {
    DMA_IFCR(TX_STREAM_NUM) = DMA_TCIF(TX_STREAM_NUM);
    tx_done();
  }
}

/**
  Pass new characters to the emergency parser, check XON/XOFF.
  Runs in the USART (IDLE) and RX DMA interrupts, same priority.
*/
static void rx_scan(void) {
  uint16_t head = rx_head();

  while (ep_pos != head) {
    emergency_parser_char(SERIAL_UART_PORT, rxbuf[ep_pos]);
    ep_pos = (ep_pos + 1) & (RX_BUFSIZE - 1);
  }

  #ifdef XONXOFF
    if ( ! xoff_sent && rx_count() > RX_BUFSIZE * 3 / 4) {
      xoff_sent = 1;
      flowchar = ASCII_XOFF;
      tx_start();
    }
  #endif
}

void uart_init(void) {
  UART_CLOCK_ON();
  DMA_CLOCK_ON();
  (void)RCC->AHB1ENR;

  // RX with pull-up, so a disconnected line reads idle.
  SET_AF(SERIAL_TX_PIN, UART_AF);
  SET_AF(SERIAL_RX_PIN, UART_AF);
  PULLUP_ON(SERIAL_RX_PIN);

  UARTx->CR1 = 0;
  UARTx->CR2 = 0;                               // 1 stop bit.
  UARTx->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
  // Oversampling by 16: BRR = PCLK / BAUD, rounded.
  UARTx->BRR = (uint32_t)((UART_PCLK + BAUD / 2) / BAUD);

  // RX: circular, peripheral to memory, byte wide.
  RX_STREAM->CR = 0;
  while (RX_STREAM->CR & DMA_SxCR_EN)
    ;
  DMA_IFCR(RX_STREAM_NUM) = DMA_ALL_FLAGS(RX_STREAM_NUM);
  RX_STREAM->PAR = (uint32_t)&UARTx->DR;
  RX_STREAM->M0AR = (uint32_t)&rxbuf[0];
  RX_STREAM->NDTR = RX_BUFSIZE;
  RX_STREAM->FCR = 0;                           // Direct mode.
  RX_STREAM->CR = (DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC |
                  DMA_SxCR_CIRC | DMA_SxCR_PL_0 | DMA_SxCR_TCIE;
  RX_STREAM->CR |= DMA_SxCR_EN;

  // TX: memory to peripheral, byte wide, started per block.
  TX_STREAM->CR = 0;
  while (TX_STREAM->CR & DMA_SxCR_EN)
    ;
  DMA_IFCR(TX_STREAM_NUM) = DMA_ALL_FLAGS(TX_STREAM_NUM);
  TX_STREAM->PAR = (uint32_t)&UARTx->DR;
  TX_STREAM->FCR = 0;
  TX_STREAM->CR = (DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC |
                  DMA_SxCR_DIR_0 | DMA_SxCR_PL_0 | DMA_SxCR_TCIE;

  UARTx->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE;

  NVIC_SetPriority(UART_IRQn, IRQ_PRIO_SERIAL);
  NVIC_SetPriority(RX_DMA_IRQn, IRQ_PRIO_SERIAL);
  NVIC_SetPriority(TX_DMA_IRQn, IRQ_PRIO_SERIAL);
  NVIC_EnableIRQ(UART_IRQn);
  NVIC_EnableIRQ(RX_DMA_IRQn);
  NVIC_EnableIRQ(TX_DMA_IRQn);
}

TEACUP_HOT
void UART_IRQHandler(void) {
  uint32_t sr = UARTx->SR;

  if (sr & (USART_SR_IDLE | USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {
    // Clear: SR read (above) followed by a DR read. DMA took the data
    // already, the line is idle.
    (void)UARTx->DR;
  }
  rx_scan();
}

TEACUP_HOT
void RX_DMA_IRQHandler(void) {
  // Buffer wrapped. Scan here too, in case the line never gets idle
  // (host streaming continuously).
  DMA_IFCR(RX_STREAM_NUM) = DMA_ALL_FLAGS(RX_STREAM_NUM);
  rx_scan();
}

TEACUP_HOT
void TX_DMA_IRQHandler(void) {
  if (DMA_ISR(TX_STREAM_NUM) & DMA_TCIF(TX_STREAM_NUM)) {
    DMA_IFCR(TX_STREAM_NUM) = DMA_ALL_FLAGS(TX_STREAM_NUM);
    tx_done();
  }
}

uint16_t uart_rxchars(void) {
  return rx_count();
}

uint8_t uart_popchar(void) {
  uint8_t c = 0;

  if (rx_head() != rxtail) {
    c = rxbuf[rxtail];
    rxtail = (rxtail + 1) & (RX_BUFSIZE - 1);
  }

  #ifdef XONXOFF
    if (xoff_sent && rx_count() < RX_BUFSIZE / 4) {
      ATOMIC_START_NOSTEP();
        xoff_sent = 0;
        flowchar = ASCII_XON;
        tx_start();
      ATOMIC_END_NOSTEP();
    }
  #endif

  return c;
}

void uart_writechar(uint8_t data) {
  for (;;) {
    uint8_t done = 0;

    ATOMIC_START_NOSTEP();
      if (tx_free()) {
        uint16_t head = txhead;

        txbuf[head] = data;
        txhead = (head + 1) & (TX_BUFSIZE - 1);
        tx_start();
        done = 1;
      }
      else {
        tx_poll();                  // Buffer full: maybe the DMA is done.
      }
    ATOMIC_END_NOSTEP();

    if (done)
      return;
  }
}

void uart_flush(void) {
  uint32_t timeout;

  for (;;) {
    uint8_t empty;

    ATOMIC_START_NOSTEP();
      uint16_t head = txhead;
      uint16_t tail = txtail;
      uint16_t len = tx_len;

      tx_poll();
      empty = (head == tail) && len == 0;
    ATOMIC_END_NOSTEP();

    if (empty)
      break;
  }

  // Wait for the last character to leave the shift register.
  for (timeout = F_CPU / 100; timeout; timeout--) {
    if (UARTx->SR & USART_SR_TC)
      break;
  }
}

int16_t uart_rx_poll(void) {
  uint8_t c;

  // DMA keeps receiving with interrupts disabled.
  if (rx_head() == rxtail)
    return -1;
  c = rxbuf[rxtail];
  rxtail = (rxtail + 1) & (RX_BUFSIZE - 1);
  return c;
}


#else /* ! SERIAL_DMA */

/// Buffer sizes, must be powers of 2, 256 max.
#define RX_BUFSIZE 128
#define TX_BUFSIZE 128

static volatile uint8_t rxbuf[RX_BUFSIZE];
static volatile uint8_t txbuf[TX_BUFSIZE];
static volatile uint16_t rxhead, rxtail;        // head: write, tail: read
static volatile uint16_t txhead, txtail;

#ifdef XONXOFF
  static volatile uint8_t flowchar;             // XON/XOFF to send asap.
  static volatile uint8_t xoff_sent;
#endif

/*
  Buffer fill levels. Every volatile index is read exactly once into a local,
  so there is a single volatile access per statement (the order of two
  volatile accesses within one expression is undefined in C).
*/
static uint16_t rx_count(void) {
  uint16_t head = rxhead;
  uint16_t tail = rxtail;

  return (uint16_t)(head - tail) & (RX_BUFSIZE - 1);
}

static uint16_t tx_free(void) {
  uint16_t head = txhead;
  uint16_t tail = txtail;

  return (TX_BUFSIZE - 1) - ((uint16_t)(head - tail) & (TX_BUFSIZE - 1));
}

void uart_init(void) {
  UART_CLOCK_ON();

  // RX with pull-up, so a disconnected line reads idle.
  SET_AF(SERIAL_TX_PIN, UART_AF);
  SET_AF(SERIAL_RX_PIN, UART_AF);
  PULLUP_ON(SERIAL_RX_PIN);

  UARTx->CR1 = 0;
  UARTx->CR2 = 0;                               // 1 stop bit.
  UARTx->CR3 = 0;                               // No flow control.
  // Oversampling by 16: BRR = PCLK / BAUD, rounded.
  UARTx->BRR = (uint32_t)((UART_PCLK + BAUD / 2) / BAUD);
  UARTx->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

  NVIC_SetPriority(UART_IRQn, IRQ_PRIO_SERIAL);
  NVIC_EnableIRQ(UART_IRQn);
}

/// Move one byte from the transmit buffer into the hardware, if possible.
TEACUP_HOT
static void tx_pump(void) {
  uint16_t tail;

  #ifdef XONXOFF
    uint8_t fc = flowchar;

    if (fc) {
      UARTx->DR = fc;
      flowchar = 0;
      return;
    }
  #endif

  tail = txtail;

  if (txhead != tail) {
    UARTx->DR = txbuf[tail];
    txtail = (tail + 1) & (TX_BUFSIZE - 1);
  }
  else {
    UARTx->CR1 &= ~USART_CR1_TXEIE;
  }
}

TEACUP_HOT
void UART_IRQHandler(void) {
  uint32_t sr = UARTx->SR;

  // Reading DR after SR also clears ORE, NE and FE.
  if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {
    uint8_t c = (uint8_t)UARTx->DR;

    if ((sr & USART_SR_RXNE) && ! (sr & USART_SR_FE)) {
      uint16_t head = rxhead;
      uint16_t next = (head + 1) & (RX_BUFSIZE - 1);
      uint16_t tail = rxtail;

      if (next != tail) {                       // Drop if buffer is full.
        rxbuf[head] = c;
        rxhead = next;
      }

      // M112, M108, M410 act right away, even with a full buffer.
      emergency_parser_char(SERIAL_UART_PORT, c);
      #ifdef XONXOFF
        if ( ! xoff_sent && rx_count() > RX_BUFSIZE * 3 / 4) {
          flowchar = ASCII_XOFF;
          xoff_sent = 1;
          UARTx->CR1 |= USART_CR1_TXEIE;
        }
      #endif
    }
  }

  if ((UARTx->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE))
    tx_pump();
}

uint16_t uart_rxchars(void) {
  return rx_count();
}

uint8_t uart_popchar(void) {
  uint8_t c = 0;
  uint16_t tail = rxtail;

  if (rxhead != tail) {
    c = rxbuf[tail];
    rxtail = (tail + 1) & (RX_BUFSIZE - 1);
  }

  #ifdef XONXOFF
    if (xoff_sent && rx_count() < RX_BUFSIZE / 4) {
      ATOMIC_START_NOSTEP();
        flowchar = ASCII_XON;
        xoff_sent = 0;
        UARTx->CR1 |= USART_CR1_TXEIE;
      ATOMIC_END_NOSTEP();
    }
  #endif

  return c;
}

/**
  Send one character. If the buffer is full, we feed the hardware ourselves.
  This makes the function safe to use from any interrupt priority, e.g. for
  debug messages from dda_clock().
*/
void uart_writechar(uint8_t data) {
  for (;;) {
    uint8_t done = 0;

    ATOMIC_START_NOSTEP();
      if (tx_free()) {
        uint16_t head = txhead;

        txbuf[head] = data;
        txhead = (head + 1) & (TX_BUFSIZE - 1);
        UARTx->CR1 |= USART_CR1_TXEIE;
        done = 1;
      }
      else if (UARTx->SR & USART_SR_TXE) {
        tx_pump();
      }
    ATOMIC_END_NOSTEP();

    if (done)
      return;
  }
}

void uart_flush(void) {
  uint32_t timeout;

  for (;;) {
    uint8_t empty;

    ATOMIC_START_NOSTEP();
      uint16_t head = txhead;
      uint16_t tail = txtail;

      empty = (head == tail);
      if ( ! empty && (UARTx->SR & USART_SR_TXE))
        tx_pump();
    ATOMIC_END_NOSTEP();

    if (empty)
      break;
  }

  // Wait for the last character to leave the shift register (about one
  // character time; the timeout covers 1200 baud).
  for (timeout = F_CPU / 100; timeout; timeout--) {
    if (UARTx->SR & USART_SR_TC)
      break;
  }
}

int16_t uart_rx_poll(void) {
  uint32_t sr = UARTx->SR;

  if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {
    uint8_t c = (uint8_t)UARTx->DR;

    // No handler runs with interrupts disabled, clear the pending state so
    // WFI in the halted state sleeps again.
    NVIC_ClearPendingIRQ(UART_IRQn);

    if ((sr & USART_SR_RXNE) && ! (sr & USART_SR_FE))
      return c;
  }
  return -1;
}


#endif /* SERIAL_DMA */

#endif /* NO_SERIAL_UART */
