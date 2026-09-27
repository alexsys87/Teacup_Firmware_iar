/** \file
  \brief I2C master for STM32F4 (I2C v1 peripheral).

  Sequence per transmission (RM0368 master transmitter):
    START -> SB -> address -> ADDR -> data ... -> last data -> BTF -> STOP.
  The queue stores 11 bit entries: bit 8 marks the last byte of a
  transmission, bit 9 an address entry, which starts each transmission.
  So devices with different addresses (display, I/O expander) can share
  the bus and the queue.

  Bit 10 on an address entry makes it a read of one byte, a transmission
  of its own (RM0368 master receiver, single byte): START -> SB ->
  address + R -> ADDR (ACK is off already, clear ADDR, set STOP) ->
  RXNE -> data.
*/

#include "i2c.h"

#ifdef I2C

#include "arch.h"
#include "cpu.h"
#include "pinio.h"
#include "atomic.h"

#ifndef I2C_INSTANCE
  #define I2C_INSTANCE 1
  #define I2C_SCL_PIN  PB_8
  #define I2C_SDA_PIN  PB_9
#endif
#ifndef I2C_SPEED
  #define I2C_SPEED    100000UL
#endif
/// Alternate function of the pins. I2C2_SDA/I2C3_SDA on PB3/PB4/PB8 use AF9.
#ifndef I2C_SCL_AF
  #define I2C_SCL_AF   4
#endif
#ifndef I2C_SDA_AF
  #define I2C_SDA_AF   4
#endif

#if I2C_INSTANCE == 1
  #define I2Cx           I2C1
  #define I2C_EV_IRQn_x  I2C1_EV_IRQn
  #define I2C_ER_IRQn_x  I2C1_ER_IRQn
  #define I2C_EV_HANDLER I2C1_EV_IRQHandler
  #define I2C_ER_HANDLER I2C1_ER_IRQHandler
  #define I2C_CLK_EN     RCC_APB1ENR_I2C1EN
  #define I2C_RST        RCC_APB1RSTR_I2C1RST
#elif I2C_INSTANCE == 2
  #define I2Cx           I2C2
  #define I2C_EV_IRQn_x  I2C2_EV_IRQn
  #define I2C_ER_IRQn_x  I2C2_ER_IRQn
  #define I2C_EV_HANDLER I2C2_EV_IRQHandler
  #define I2C_ER_HANDLER I2C2_ER_IRQHandler
  #define I2C_CLK_EN     RCC_APB1ENR_I2C2EN
  #define I2C_RST        RCC_APB1RSTR_I2C2RST
#elif I2C_INSTANCE == 3
  #define I2Cx           I2C3
  #define I2C_EV_IRQn_x  I2C3_EV_IRQn
  #define I2C_ER_IRQn_x  I2C3_ER_IRQn
  #define I2C_EV_HANDLER I2C3_EV_IRQHandler
  #define I2C_ER_HANDLER I2C3_ER_IRQHandler
  #define I2C_CLK_EN     RCC_APB1ENR_I2C3EN
  #define I2C_RST        RCC_APB1RSTR_I2C3RST
#else
  #error I2C_INSTANCE must be 1, 2 or 3.
#endif

#define I2C_PCLK_MHZ   (F_CPU / 2 / 1000000UL)    // APB1

/// Queue size, power of 2.
#define I2C_BUFSIZE 64

enum {
  I2C_IDLE = 0,       ///< No transmission.
  I2C_ACTIVE,         ///< Transmission ongoing, sending data.
  I2C_WAIT_DATA,      ///< Transmission open, queue ran empty (clock stretched).
  I2C_ENDING,         ///< Last byte written, waiting for BTF to send STOP.
  I2C_READING         ///< Read of one byte, waiting for ADDR, then RXNE.
};

static volatile uint16_t i2c_buf[I2C_BUFSIZE];
static volatile uint16_t i2c_head, i2c_tail;
static volatile uint8_t i2c_state;
static uint8_t i2c_address;         ///< Default address (i2c_write()).
static uint8_t i2c_inited;
static uint8_t w_open;              ///< Writer: transmission started, not ended.
static volatile uint8_t rd_state;   ///< I2C_READ_... of the one read.
static volatile uint8_t rd_value;

#define I2C_IRQS  (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN)

/// Whether the queue is empty. One volatile access per statement.
static uint8_t i2c_queue_empty(void) {
  uint16_t head = i2c_head;
  uint16_t tail = i2c_tail;

  return head == tail;
}

/// Take the next entry from the queue. Queue must not be empty.
static uint16_t i2c_queue_pop(void) {
  uint16_t tail = i2c_tail;
  uint16_t d = i2c_buf[tail];

  i2c_tail = (tail + 1) & (I2C_BUFSIZE - 1);
  return d;
}

/// Wait for a pending STOP to complete, with timeout.
static void i2c_wait_stop(void) {
  uint32_t timeout = 100000;

  while ((I2Cx->CR1 & I2C_CR1_STOP) && timeout--)
    ;
}

/// Begin a new transmission. Call with interrupts disabled.
static void i2c_start(void) {
  i2c_wait_stop();
  i2c_state = I2C_ACTIVE;
  I2Cx->CR2 |= I2C_IRQS;
  I2Cx->CR1 |= I2C_CR1_START;
}

void i2c_init(uint8_t address) {
  if (address)
    i2c_address = address;
  if (i2c_inited)
    return;                     // Display and expander both init the bus.
  i2c_inited = 1;
  i2c_head = 0;
  i2c_tail = 0;
  i2c_state = I2C_IDLE;

  // Open drain with pull-ups (external ones recommended for 400 kHz).
  gpio_open_drain(PIN_PORT(I2C_SCL_PIN), PIN_NUM(I2C_SCL_PIN), 1);
  gpio_open_drain(PIN_PORT(I2C_SDA_PIN), PIN_NUM(I2C_SDA_PIN), 1);
  PULLUP_ON(I2C_SCL_PIN);
  PULLUP_ON(I2C_SDA_PIN);
  SET_AF(I2C_SCL_PIN, I2C_SCL_AF);
  SET_AF(I2C_SDA_PIN, I2C_SDA_AF);

  RCC->APB1ENR |= I2C_CLK_EN;
  RCC->APB1RSTR |= I2C_RST;
  RCC->APB1RSTR &= ~I2C_RST;

  I2Cx->CR1 = 0;
  I2Cx->CR2 = I2C_PCLK_MHZ;
  #if I2C_SPEED > 100000UL
    // Fast mode, duty 2:1.
    I2Cx->CCR = I2C_CCR_FS |
                ((F_CPU / 2 + 3 * I2C_SPEED - 1) / (3 * I2C_SPEED));
    I2Cx->TRISE = I2C_PCLK_MHZ * 300 / 1000 + 1;
  #else
    I2Cx->CCR = (F_CPU / 2 + 2 * I2C_SPEED - 1) / (2 * I2C_SPEED);
    I2Cx->TRISE = I2C_PCLK_MHZ + 1;
  #endif
  I2Cx->CR1 = I2C_CR1_PE;

  NVIC_SetPriority(I2C_EV_IRQn_x, IRQ_PRIO_I2C);
  NVIC_SetPriority(I2C_ER_IRQn_x, IRQ_PRIO_I2C);
  NVIC_EnableIRQ(I2C_EV_IRQn_x);
  NVIC_EnableIRQ(I2C_ER_IRQn_x);
}

uint8_t i2c_busy(void) {
  return i2c_state != I2C_IDLE;
}

uint8_t i2c_tx_open(void) {
  return w_open;
}

/// Put an entry into the queue, wait for room.
static void i2c_push(uint16_t entry) {
  uint16_t head = i2c_head;       // Written by the writer only.
  uint16_t next = (head + 1) & (I2C_BUFSIZE - 1);
  uint16_t tail;

  // Wait for room in the queue, the interrupt drains it.
  do {
    tail = i2c_tail;
  } while (next == tail);

  i2c_buf[head] = entry;
  i2c_head = next;
}

/// Start a transmission if the bus is idle, or continue a stretched one.
static void i2c_kick(void) {
  ATOMIC_START_NOSTEP();
    uint8_t state = i2c_state;

    if (state == I2C_IDLE) {
      i2c_start();
    }
    else if (state == I2C_WAIT_DATA) {
      i2c_state = I2C_ACTIVE;
      I2Cx->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN;
    }
  ATOMIC_END_NOSTEP();
}

uint8_t i2c_read_from(uint8_t address) {
  if (w_open || rd_state == I2C_READ_PENDING)
    return 0;
  rd_state = I2C_READ_PENDING;
  i2c_push(0x600 | address);
  i2c_kick();
  return 1;
}

uint8_t i2c_read_result(uint8_t *value) {
  uint8_t r = rd_state;

  if (r == I2C_READ_DONE || r == I2C_READ_FAILED) {
    *value = rd_value;
    rd_state = I2C_READ_NONE;
  }
  return r;
}

void i2c_write(uint8_t data, uint8_t last_byte) {
  i2c_write_to(i2c_address, data, last_byte);
}

void i2c_write_to(uint8_t address, uint8_t data, uint8_t last_byte) {
  if ( ! w_open) {
    i2c_push(0x200 | address);
    w_open = 1;
  }
  i2c_push(data | (last_byte ? 0x100 : 0));
  if (last_byte)
    w_open = 0;
  i2c_kick();
}

void I2C_EV_HANDLER(void) {
  uint32_t sr1 = I2Cx->SR1;

  if (sr1 & I2C_SR1_SB) {                       // EV5: START sent.
    uint16_t a = i2c_queue_empty() ? 0 : i2c_queue_pop();

    if (a & 0x400) {                            // A read.
      i2c_state = I2C_READING;
      I2Cx->DR = (uint8_t)a | 1;
      return;
    }
    // Every transmission starts with its address entry.
    I2Cx->DR = (a & 0x200) ? (uint8_t)a : i2c_address;
    return;
  }

  if (i2c_state == I2C_READING) {
    if (sr1 & I2C_SR1_ADDR) {                   // EV6, one byte to receive:
      // ACK is off; clear ADDR and set STOP right after, without an
      // interrupt in between (RM0368, single byte reception).
      ATOMIC_START();
        (void)I2Cx->SR2;
        I2Cx->CR1 |= I2C_CR1_STOP;
      ATOMIC_END();
    }
    else if (sr1 & I2C_SR1_RXNE) {              // EV7: the byte.
      rd_value = (uint8_t)I2Cx->DR;
      rd_state = I2C_READ_DONE;
      I2Cx->CR2 &= ~I2C_IRQS;
      i2c_state = I2C_IDLE;
      if ( ! i2c_queue_empty())
        i2c_start();
    }
    return;
  }

  if (sr1 & I2C_SR1_ADDR) {                     // EV6: address acknowledged.
    (void)I2Cx->SR2;
    sr1 = I2Cx->SR1;
  }

  if (i2c_state == I2C_ENDING) {
    if (sr1 & I2C_SR1_BTF) {                    // EV8_2: all bytes out.
      I2Cx->CR1 |= I2C_CR1_STOP;
      I2Cx->CR2 &= ~I2C_IRQS;
      i2c_state = I2C_IDLE;
      if ( ! i2c_queue_empty())                 // Next transmission queued.
        i2c_start();
    }
    return;
  }

  if (sr1 & I2C_SR1_TXE) {                      // EV8: ready for data.
    if ( ! i2c_queue_empty()) {
      uint16_t d = i2c_queue_pop();

      I2Cx->DR = (uint8_t)d;
      if (d & 0x100) {
        i2c_state = I2C_ENDING;
        I2Cx->CR2 &= ~I2C_CR2_ITBUFEN;          // Wait for BTF only.
      }
    }
    else {
      // Queue empty in the middle of a transmission. The bus stays
      // stretched until i2c_write() delivers more data.
      i2c_state = I2C_WAIT_DATA;
      I2Cx->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
    }
  }
}

/**
  Error (NACK, arbitration lost, bus error): stop and drop the rest of the
  current transmission, then continue with the next one, if any.
*/
void I2C_ER_HANDLER(void) {
  I2Cx->SR1 &= ~(I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR | I2C_SR1_OVR);
  I2Cx->CR1 |= I2C_CR1_STOP;
  I2Cx->CR2 &= ~I2C_IRQS;

  if (i2c_state == I2C_READING) {
    rd_value = 0xFF;                            // A read has no data entries.
    rd_state = I2C_READ_FAILED;
  }
  else if (i2c_state != I2C_ENDING) {
    while ( ! i2c_queue_empty()) {
      if (i2c_queue_pop() & 0x100)
        break;
    }
  }
  i2c_state = I2C_IDLE;

  if ( ! i2c_queue_empty())
    i2c_start();
}

#endif /* I2C */
