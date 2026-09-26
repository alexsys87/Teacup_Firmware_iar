/** \file
  \brief USB CDC ACM virtual COM port, register level driver for the
         STM32F401 / STM32F411 OTG_FS core in device mode.

  No ST USB library, no HAL. The core runs in slave (non-DMA) mode with
  dedicated transmit FIFOs.

  Endpoints:
    EP0      control, 64 bytes
    EP1 OUT  bulk, CDC data host -> printer, 64 bytes
    EP1 IN   bulk, CDC data printer -> host, 64 bytes
    EP2 IN   interrupt, CDC notifications (declared, never used)

  FIFO RAM (320 words): RX 128, TX0 32, TX1 64, TX2 16 words.

  Receiving: every OUT packet goes straight from the RX FIFO into a ring
  buffer. EP1 OUT is only armed while at least one packet fits into the
  ring buffer, otherwise the core NAKs and the host waits. This is real
  flow control, no character is ever lost, however fast the host sends.
  Received characters are passed to the emergency parser (M112, M108,
  M410) right in the interrupt, like on the UART.

  Transmitting: a ring buffer, sent as one packet (up to 64 bytes) per
  transfer. A transfer which ends with a full packet is followed by a zero
  length packet, so the host doesn't hold back the data.

  VBUS sensing is off (the Black Pill doesn't connect VBUS to PA9, and PA9
  is used otherwise). Unplugging the cable therefore looks like a suspend
  to the core: output is dropped while suspended, the host resets and
  re-enumerates the device when it's plugged in again.

  With interrupts disabled (halted state after printer_kill()) the driver
  is polled from usb_cdc_rx_poll(), usb_cdc_writechar() and
  usb_cdc_flush(), so M999 works over USB, too.

  Errata ES0182 / ES0287 (F401 / F411): a transmit FIFO write sequence must
  not be interrupted by other OTG_FS register accesses. All FIFO writes run
  either in the USB interrupt or with it masked, so they never are.
*/

#include "usb_cdc.h"

#ifdef USB_CDC

#include "arch.h"
#include "cpu.h"
#include "pinio.h"
#include "atomic.h"
#include "delay.h"
#include "serial.h"
#include "kill.h"
#include "emergency_parser.h"

/* ---------------------------------------------------------------------- */
/*  Configuration                                                          */
/* ---------------------------------------------------------------------- */

#ifndef USB_VID
  #define USB_VID           0x0483
#endif
#ifndef USB_PID
  #define USB_PID           0x5740
#endif
#ifndef USB_MANUFACTURER
  #define USB_MANUFACTURER  "Teacup"
#endif
#ifndef USB_PRODUCT
  #define USB_PRODUCT       "Teacup 3D printer"
#endif

/// Ring buffer sizes, powers of 2. RX must hold at least one packet.
#ifndef USB_RX_BUFSIZE
  #define USB_RX_BUFSIZE    256
#endif
#ifndef USB_TX_BUFSIZE
  #define USB_TX_BUFSIZE    512
#endif

/// How long usb_cdc_writechar() waits for the host to read.
#ifndef USB_TX_TIMEOUT_MS
  #define USB_TX_TIMEOUT_MS 50
#endif
/// How long usb_cdc_flush() waits at most.
#define USB_FLUSH_TIMEOUT_MS 100

#if (USB_RX_BUFSIZE & (USB_RX_BUFSIZE - 1)) || USB_RX_BUFSIZE < 128
  #error USB_RX_BUFSIZE must be a power of 2, 128 or more.
#endif
#if (USB_TX_BUFSIZE & (USB_TX_BUFSIZE - 1)) || USB_TX_BUFSIZE < 64
  #error USB_TX_BUFSIZE must be a power of 2, 64 or more.
#endif

/* USB needs exactly 48 MHz from PLL Q, see cpu.c. */
#if F_CPU != 48000000UL && F_CPU != 84000000UL && F_CPU != 96000000UL
  #error USB_CDC needs F_CPU 48, 84 or 96 MHz (48 MHz USB clock), not 100 MHz.
#endif

/*
  PA11 (DM) and PA12 (DP) belong to USB now. Catch configurations which
  still use them for something else. Unused pin names expand to 0 here.
*/
#define USB_PIN_TAKEN(p)  (PIN_ID(p) == 0x0B || PIN_ID(p) == 0x0C)

#if USB_PIN_TAKEN(X_STEP_PIN) || USB_PIN_TAKEN(X_DIR_PIN) || \
    USB_PIN_TAKEN(X_MIN_PIN) || USB_PIN_TAKEN(X_MAX_PIN) || \
    USB_PIN_TAKEN(X_ENABLE_PIN) || \
    USB_PIN_TAKEN(Y_STEP_PIN) || USB_PIN_TAKEN(Y_DIR_PIN) || \
    USB_PIN_TAKEN(Y_MIN_PIN) || USB_PIN_TAKEN(Y_MAX_PIN) || \
    USB_PIN_TAKEN(Y_ENABLE_PIN) || \
    USB_PIN_TAKEN(Z_STEP_PIN) || USB_PIN_TAKEN(Z_DIR_PIN) || \
    USB_PIN_TAKEN(Z_MIN_PIN) || USB_PIN_TAKEN(Z_MAX_PIN) || \
    USB_PIN_TAKEN(Z_ENABLE_PIN) || \
    USB_PIN_TAKEN(Z2_STEP_PIN) || USB_PIN_TAKEN(Z2_DIR_PIN) || \
    USB_PIN_TAKEN(Z2_ENABLE_PIN) || \
    USB_PIN_TAKEN(E_STEP_PIN) || USB_PIN_TAKEN(E_DIR_PIN) || \
    USB_PIN_TAKEN(E_ENABLE_PIN) || USB_PIN_TAKEN(STEPPER_ENABLE_PIN)
  #error A stepper or endstop pin is PA11/PA12, which USB_CDC needs.
#endif
#if USB_PIN_TAKEN(PS_ON_PIN) || USB_PIN_TAKEN(PS_MOSFET_PIN) || \
    USB_PIN_TAKEN(DEBUG_LED_PIN) || USB_PIN_TAKEN(BEEPER_PIN) || \
    USB_PIN_TAKEN(SERIAL_TX_PIN) || USB_PIN_TAKEN(SERIAL_RX_PIN) || \
    USB_PIN_TAKEN(SPI_SCK_PIN) || USB_PIN_TAKEN(SPI_MISO_PIN) || \
    USB_PIN_TAKEN(SPI_MOSI_PIN) || USB_PIN_TAKEN(SPI_FLASH_CS_PIN) || \
    USB_PIN_TAKEN(SD_CARD_SELECT_PIN) || USB_PIN_TAKEN(MCP3008_SELECT_PIN)
  #error A pin in use is PA11/PA12, which USB_CDC needs.
#endif
#if defined I2C && (USB_PIN_TAKEN(I2C_SCL_PIN) || USB_PIN_TAKEN(I2C_SDA_PIN))
  #error An I2C pin is PA11/PA12, which USB_CDC needs.
#endif
#if defined DISPLAY_BUS_4BIT && \
    (USB_PIN_TAKEN(DISPLAY_RS_PIN) || USB_PIN_TAKEN(DISPLAY_RW_PIN) || \
     USB_PIN_TAKEN(DISPLAY_E_PIN) || USB_PIN_TAKEN(DISPLAY_D4_PIN) || \
     USB_PIN_TAKEN(DISPLAY_D5_PIN) || USB_PIN_TAKEN(DISPLAY_D6_PIN) || \
     USB_PIN_TAKEN(DISPLAY_D7_PIN))
  #error A display pin is PA11/PA12, which USB_CDC needs.
#endif

/* Heater pins come from the DEFINE_HEATER() lines. */
#undef DEFINE_HEATER_ACTUAL
#define DEFINE_HEATER_ACTUAL(name, pin, ...) \
  _Static_assert( ! USB_PIN_TAKEN(pin), \
                 "Heater " #name " is on PA11/PA12, which USB_CDC needs.");
#include "config_wrapper.h"
#undef DEFINE_HEATER_ACTUAL

/* ---------------------------------------------------------------------- */
/*  Hardware                                                               */
/* ---------------------------------------------------------------------- */

#define USBx        USB_OTG_FS
#define USBD        ((USB_OTG_DeviceTypeDef *)(USB_OTG_FS_PERIPH_BASE + \
                                               USB_OTG_DEVICE_BASE))
#define INEP(n)     ((USB_OTG_INEndpointTypeDef *)(USB_OTG_FS_PERIPH_BASE + \
                     USB_OTG_IN_ENDPOINT_BASE + (n) * USB_OTG_EP_REG_SIZE))
#define OUTEP(n)    ((USB_OTG_OUTEndpointTypeDef *)(USB_OTG_FS_PERIPH_BASE + \
                     USB_OTG_OUT_ENDPOINT_BASE + (n) * USB_OTG_EP_REG_SIZE))
#define DFIFO(n)    (*(volatile uint32_t *)(USB_OTG_FS_PERIPH_BASE + \
                     USB_OTG_FIFO_BASE + (n) * USB_OTG_FIFO_SIZE))
#define PCGCCTL     (*(volatile uint32_t *)(USB_OTG_FS_PERIPH_BASE + \
                     USB_OTG_PCGCCTL_BASE))

#define EP0_MPS           64
#define CDC_DATA_EP       1
#define CDC_NOTIFY_EP     2
#define CDC_DATA_MPS      64
#define CDC_NOTIFY_MPS    16

/* FIFO RAM layout in 32 bit words, 320 words available. */
#define FIFO_RX_WORDS     128
#define FIFO_TX0_WORDS    32
#define FIFO_TX1_WORDS    64
#define FIFO_TX2_WORDS    16

/* GRXSTSP packet status. */
#define PKTSTS_OUT_DATA   2
#define PKTSTS_OUT_DONE   3
#define PKTSTS_SETUP_DONE 4
#define PKTSTS_SETUP_DATA 6

#define EPTYP_BULK        2UL
#define EPTYP_INTERRUPT   3UL

#define MS_TO_CYCLES(ms)  ((uint32_t)(ms) * (F_CPU / 1000UL))

/* ---------------------------------------------------------------------- */
/*  Descriptors                                                            */
/* ---------------------------------------------------------------------- */

#define LO(x)   ((uint8_t)((x) & 0xFF))
#define HI(x)   ((uint8_t)(((x) >> 8) & 0xFF))

static const uint8_t device_desc[18] = {
  18, 1,                  // bLength, DEVICE
  0x00, 0x02,             // USB 2.0
  0x02, 0x02, 0x00,       // CDC, ACM (Windows loads usbser.sys by this)
  EP0_MPS,
  LO(USB_VID), HI(USB_VID),
  LO(USB_PID), HI(USB_PID),
  0x00, 0x01,             // bcdDevice 1.00
  1, 2, 3,                // manufacturer, product, serial number strings
  1                       // one configuration
};

#define CONFIG_DESC_LEN 67

static const uint8_t config_desc[CONFIG_DESC_LEN] = {
  9, 2, LO(CONFIG_DESC_LEN), HI(CONFIG_DESC_LEN),
  2,                      // interfaces
  1,                      // bConfigurationValue
  0,                      // no string
  0x80,                   // bus powered
  50,                     // 100 mA

  // Interface 0: communication class, ACM, no AT command protocol (keeps
  // Linux ModemManager from probing the printer with AT commands).
  9, 4, 0, 0, 1, 0x02, 0x02, 0x00, 0,
  5, 0x24, 0x00, 0x10, 0x01,          // header, CDC 1.10
  5, 0x24, 0x01, 0x00, 1,             // call management, data interface 1
  4, 0x24, 0x02, 0x02,                // ACM: line coding, control line state
  5, 0x24, 0x06, 0, 1,                // union: master 0, slave 1
  7, 5, 0x80 | CDC_NOTIFY_EP, 0x03, CDC_NOTIFY_MPS, 0, 16,

  // Interface 1: CDC data.
  9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
  7, 5, CDC_DATA_EP, 0x02, CDC_DATA_MPS, 0, 0,
  7, 5, 0x80 | CDC_DATA_EP, 0x02, CDC_DATA_MPS, 0, 0
};

static const uint8_t string0_desc[4] = { 4, 3, 0x09, 0x04 };  // en-US

/* ---------------------------------------------------------------------- */
/*  State                                                                  */
/* ---------------------------------------------------------------------- */

static uint8_t usb_ok;                          ///< Core came up.
static volatile uint8_t usb_configured;         ///< SET_CONFIGURATION 1.
static volatile uint8_t usb_suspended;
static volatile uint8_t line_state;             ///< Bit 0 DTR, bit 1 RTS.
static uint8_t line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };

// EP0.
static uint32_t setup_words[2];                 ///< Last SETUP packet.
#define SETUP ((const uint8_t *)setup_words)
static const uint8_t *ep0_in_ptr;
static uint16_t ep0_in_rem;
static uint8_t ep0_in_zlp;                      ///< Short transfer, end with ZLP.
static uint8_t ep0_in_more;                     ///< Another packet follows.
static uint8_t ep0_buf[128];                    ///< Built descriptors.
static uint8_t ep0_rx[EP0_MPS];                 ///< OUT data stage.
static uint8_t ep0_rx_len;
static uint8_t ep0_out_req;                     ///< Request waiting for data.

// CDC data.
static volatile uint8_t rxbuf[USB_RX_BUFSIZE];
static volatile uint16_t rxhead;                ///< Written by the ISR.
static volatile uint16_t rxtail;                ///< Read by the main loop.
static uint16_t ep_pos;                         ///< Emergency parser (ISR).
static volatile uint8_t rx_armed;               ///< EP1 OUT enabled.

static volatile uint8_t txbuf[USB_TX_BUFSIZE];
static volatile uint16_t txhead, txtail;
static volatile uint8_t tx_busy;                ///< EP1 IN transfer running.
static uint8_t tx_last_len;                     ///< For the ZLP decision.
static volatile uint8_t tx_stalled;             ///< Host didn't read, drop.

static void usb_irq(void);

/* ---------------------------------------------------------------------- */
/*  Helpers                                                                */
/* ---------------------------------------------------------------------- */

static uint16_t rx_count(void) {
  uint16_t head = rxhead;
  uint16_t tail = rxtail;

  return (uint16_t)(head - tail) & (USB_RX_BUFSIZE - 1);
}

static uint16_t tx_count(void) {
  uint16_t head = txhead;
  uint16_t tail = txtail;

  return (uint16_t)(head - tail) & (USB_TX_BUFSIZE - 1);
}

static uint8_t wait_flag(volatile uint32_t *reg, uint32_t mask,
                         uint32_t value, uint32_t ms) {
  uint32_t start = DWT->CYCCNT;

  while ((*reg & mask) != value) {
    if (DWT->CYCCNT - start > MS_TO_CYCLES(ms))
      return 0;
  }
  return 1;
}

static void flush_tx_fifo(uint32_t num) {        // num 0x10 = all
  wait_flag(&USBx->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL, USB_OTG_GRSTCTL_AHBIDL, 2);
  USBx->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (num << USB_OTG_GRSTCTL_TXFNUM_Pos);
  wait_flag(&USBx->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH, 0, 2);
}

static void flush_rx_fifo(void) {
  wait_flag(&USBx->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL, USB_OTG_GRSTCTL_AHBIDL, 2);
  USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
  wait_flag(&USBx->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH, 0, 2);
}

/*
  Pending interrupts. Two volatile reads in one expression have no defined
  order in C (IAR Pa082), so read them one after the other.
*/
static uint32_t gint_pending(void) {
  uint32_t sts = USBx->GINTSTS;

  return sts & USBx->GINTMSK;
}

static uint32_t daint_pending(void) {
  uint32_t daint = USBD->DAINT;

  return daint & USBD->DAINTMSK;
}

/**
  Where may a waiting function spin, and how?
    WAIT_SPIN  the USB interrupt runs by itself
    WAIT_POLL  it can't (interrupts off, masked, same priority): call the
               handler ourselves
    WAIT_NONE  don't wait at all (called from the step interrupt, which
               may have interrupted the USB handler)
*/
enum { WAIT_NONE, WAIT_SPIN, WAIT_POLL };

static uint8_t wait_mode(void) {
  uint32_t ipsr = __get_IPSR();
  uint32_t basepri = __get_BASEPRI();

  if (ipsr != 0) {
    int32_t irq = (int32_t)ipsr - 16;
    uint32_t prio = NVIC_GetPriority((IRQn_Type)irq);

    if (irq == (int32_t)OTG_FS_IRQn && ! printer_killed)
      return WAIT_NONE;                   // Output from our own handler.
    if (prio < IRQ_PRIO_SERIAL)
      return WAIT_NONE;
    if (prio == IRQ_PRIO_SERIAL)
      return WAIT_POLL;
  }
  if (__get_PRIMASK())
    return WAIT_POLL;
  if (basepri != 0 &&
      (basepri >> (8U - __NVIC_PRIO_BITS)) <= IRQ_PRIO_SERIAL)
    return WAIT_POLL;
  return WAIT_SPIN;
}

/// Run the interrupt handler by hand. Call with the USB interrupt masked.
static void usb_poll(void) {
  if (usb_ok && gint_pending()) {
    usb_irq();
    NVIC_ClearPendingIRQ(OTG_FS_IRQn);
  }
}

/// Copy a linear buffer into a TX FIFO, as one packet.
static void ep_write_packet(uint8_t ep, const uint8_t *data, uint16_t len) {
  uint16_t i;

  INEP(ep)->DIEPTSIZ = (1UL << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len;
  INEP(ep)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;

  for (i = 0; i < len; i += 4) {
    uint32_t w = 0;
    uint8_t k;

    for (k = 0; k < 4 && i + k < len; k++)
      w |= (uint32_t)data[i + k] << (8 * k);
    DFIFO(ep) = w;
  }
}

/* ---------------------------------------------------------------------- */
/*  CDC data endpoints                                                     */
/* ---------------------------------------------------------------------- */

/// Enable EP1 OUT if a whole packet fits. Call with USB interrupt masked.
static void rx_arm(void) {
  if ( ! usb_configured || rx_armed)
    return;
  if ((USB_RX_BUFSIZE - 1) - rx_count() < CDC_DATA_MPS)
    return;                               // Host gets NAK until we read.

  OUTEP(CDC_DATA_EP)->DOEPTSIZ = (1UL << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
                                 CDC_DATA_MPS;
  OUTEP(CDC_DATA_EP)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
  rx_armed = 1;
}

/// Move a received OUT packet from the RX FIFO into the ring buffer.
static void rx_read_packet(uint16_t bcnt) {
  uint16_t head = rxhead;
  uint16_t i;

  for (i = 0; i < bcnt; i += 4) {
    uint32_t w = DFIFO(0);
    uint8_t k;

    for (k = 0; k < 4 && i + k < bcnt; k++) {
      uint16_t next = (head + 1) & (USB_RX_BUFSIZE - 1);

      if (next != rxtail) {               // Can't overflow, see rx_arm().
        rxbuf[head] = (uint8_t)w;
        head = next;
      }
      w >>= 8;
    }
  }
  rxhead = head;

  // M112, M108, M410 act right away. Not in the halted state: there
  // printer_kill() itself polls us and looks for M999.
  if (printer_killed) {
    ep_pos = head;
  }
  else {
    while (ep_pos != rxhead) {
      uint8_t c = rxbuf[ep_pos];

      ep_pos = (ep_pos + 1) & (USB_RX_BUFSIZE - 1);
      emergency_parser_char(SERIAL_USB_PORT, c);
    }
  }
}

/// Start the next IN packet, if idle. Call with USB interrupt masked.
static void tx_kick(void) {
  uint16_t n, tail, i;

  if ( ! usb_configured || tx_busy)
    return;

  n = tx_count();
  if (n == 0) {
    // Transfer ended with a full packet: terminate it.
    if (tx_last_len == CDC_DATA_MPS) {
      tx_last_len = 0;
      tx_busy = 1;
      ep_write_packet(CDC_DATA_EP, 0, 0);
    }
    return;
  }
  if (n > CDC_DATA_MPS)
    n = CDC_DATA_MPS;

  // Packet of 64 bytes max: 16 words, the FIFO has 64, and there's only
  // one packet in flight. So there's always room.
  INEP(CDC_DATA_EP)->DIEPTSIZ = (1UL << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | n;
  INEP(CDC_DATA_EP)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;

  tail = txtail;
  for (i = 0; i < n; i += 4) {
    uint32_t w = 0;
    uint8_t k;

    for (k = 0; k < 4 && i + k < n; k++) {
      w |= (uint32_t)txbuf[tail] << (8 * k);
      tail = (tail + 1) & (USB_TX_BUFSIZE - 1);
    }
    DFIFO(CDC_DATA_EP) = w;
  }
  txtail = tail;
  tx_last_len = (uint8_t)n;
  tx_busy = 1;
}

/// Activate the CDC endpoints (SET_CONFIGURATION 1).
static void cdc_configure(void) {
  INEP(CDC_DATA_EP)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP |
      (EPTYP_BULK << USB_OTG_DIEPCTL_EPTYP_Pos) |
      ((uint32_t)CDC_DATA_EP << USB_OTG_DIEPCTL_TXFNUM_Pos) |
      USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_SNAK | CDC_DATA_MPS;
  INEP(CDC_NOTIFY_EP)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP |
      (EPTYP_INTERRUPT << USB_OTG_DIEPCTL_EPTYP_Pos) |
      ((uint32_t)CDC_NOTIFY_EP << USB_OTG_DIEPCTL_TXFNUM_Pos) |
      USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_SNAK | CDC_NOTIFY_MPS;
  OUTEP(CDC_DATA_EP)->DOEPCTL = USB_OTG_DOEPCTL_USBAEP |
      (EPTYP_BULK << USB_OTG_DOEPCTL_EPTYP_Pos) |
      USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_SNAK | CDC_DATA_MPS;

  flush_tx_fifo(CDC_DATA_EP);
  flush_tx_fifo(CDC_NOTIFY_EP);

  USBD->DAINTMSK |= (1UL << CDC_DATA_EP) | (1UL << CDC_NOTIFY_EP) |
                    (1UL << (16 + CDC_DATA_EP));

  // Old output (from before the host connected) is dropped.
  txtail = txhead;
  tx_busy = 0;
  tx_last_len = 0;
  tx_stalled = 0;
  rx_armed = 0;
  usb_configured = 1;
  rx_arm();
}

/// Deactivate the CDC endpoints (bus reset, SET_CONFIGURATION 0).
static void cdc_deconfigure(void) {
  uint8_t ep;

  usb_configured = 0;
  rx_armed = 0;
  tx_busy = 0;
  line_state = 0;

  for (ep = 1; ep < 4; ep++) {
    if (INEP(ep)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
      INEP(ep)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
    INEP(ep)->DIEPCTL = 0;
    INEP(ep)->DIEPINT = 0xFFFF;
    if (OUTEP(ep)->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
      OUTEP(ep)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
    OUTEP(ep)->DOEPCTL = 0;
    OUTEP(ep)->DOEPINT = 0xFFFF;
  }
  USBD->DAINTMSK = (1UL << 0) | (1UL << 16);
}

/* ---------------------------------------------------------------------- */
/*  Control endpoint                                                       */
/* ---------------------------------------------------------------------- */

/// Prepare EP0 OUT for SETUP packets and OUT data / status stages.
static void ep0_arm_out(void) {
  OUTEP(0)->DOEPTSIZ = (3UL << USB_OTG_DOEPTSIZ_STUPCNT_Pos) |
                       (1UL << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | EP0_MPS;
  OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
}

static void ep0_in_next(void) {
  uint16_t n = (ep0_in_rem > EP0_MPS) ? EP0_MPS : ep0_in_rem;

  ep_write_packet(0, ep0_in_ptr, n);
  ep0_in_ptr += n;
  ep0_in_rem -= n;
  ep0_in_more = (ep0_in_rem > 0) || (n == EP0_MPS && ep0_in_zlp);
}

/// Data stage IN (or, with len 0, the status stage of an OUT request).
static void ep0_send(const uint8_t *data, uint16_t len) {
  uint16_t wlength = (uint16_t)(SETUP[6] | (SETUP[7] << 8));

  if (len > wlength)
    len = wlength;
  ep0_in_ptr = data;
  ep0_in_rem = len;
  ep0_in_zlp = (len < wlength);
  ep0_in_next();
}

static void ep0_ack(void) {
  ep0_send(0, 0);
}

static void ep0_stall(void) {
  INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
  OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
  ep0_in_more = 0;
  ep0_out_req = 0;
}

/// ASCII string -> string descriptor in ep0_buf.
static uint16_t string_desc(const char *s) {
  uint16_t n = 0;

  while (s[n] && n < (sizeof(ep0_buf) - 2) / 2) {
    ep0_buf[2 + 2 * n] = (uint8_t)s[n];
    ep0_buf[3 + 2 * n] = 0;
    n++;
  }
  ep0_buf[0] = (uint8_t)(2 + 2 * n);
  ep0_buf[1] = 3;
  return (uint16_t)(2 + 2 * n);
}

/// Serial number: the 96 bit unique ID as 24 hex digits.
static uint16_t serial_desc(void) {
  static const char hex[] = "0123456789ABCDEF";
  char s[25];
  uint8_t i;

  for (i = 0; i < 12; i++) {
    uint8_t b = *(const volatile uint8_t *)(UID_BASE + i);

    s[2 * i] = hex[b >> 4];
    s[2 * i + 1] = hex[b & 0x0F];
  }
  s[24] = '\0';
  return string_desc(s);
}

static void get_descriptor(void) {
  uint8_t type = SETUP[3];
  uint8_t index = SETUP[2];

  switch (type) {
    case 1:
      ep0_send(device_desc, sizeof(device_desc));
      break;
    case 2:
      ep0_send(config_desc, sizeof(config_desc));
      break;
    case 3:
      if (index == 0)
        ep0_send(string0_desc, sizeof(string0_desc));
      else if (index == 1)
        ep0_send(ep0_buf, string_desc(USB_MANUFACTURER));
      else if (index == 2)
        ep0_send(ep0_buf, string_desc(USB_PRODUCT));
      else if (index == 3)
        ep0_send(ep0_buf, serial_desc());
      else
        ep0_stall();
      break;
    default:                              // Device qualifier etc.: FS only.
      ep0_stall();
      break;
  }
}

/// Endpoint address (wIndex) -> 1 if that endpoint exists.
static uint8_t valid_ep(uint8_t addr) {
  uint8_t ep = addr & 0x7F;

  if (ep == 0)
    return 1;
  if ( ! usb_configured)
    return 0;
  return (addr == (0x80 | CDC_DATA_EP)) || (addr == CDC_DATA_EP) ||
         (addr == (0x80 | CDC_NOTIFY_EP));
}

static void set_halt(uint8_t addr, uint8_t halt) {
  uint8_t ep = addr & 0x7F;

  if (ep == 0)
    return;
  if (addr & 0x80) {
    if (halt)
      INEP(ep)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
    else
      INEP(ep)->DIEPCTL = (INEP(ep)->DIEPCTL & ~USB_OTG_DIEPCTL_STALL) |
                          USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
  }
  else {
    if (halt)
      OUTEP(ep)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
    else
      OUTEP(ep)->DOEPCTL = (OUTEP(ep)->DOEPCTL & ~USB_OTG_DOEPCTL_STALL) |
                           USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
  }
}

static uint8_t get_halt(uint8_t addr) {
  uint8_t ep = addr & 0x7F;

  if (addr & 0x80)
    return (INEP(ep)->DIEPCTL & USB_OTG_DIEPCTL_STALL) ? 1 : 0;
  return (OUTEP(ep)->DOEPCTL & USB_OTG_DOEPCTL_STALL) ? 1 : 0;
}

static void standard_request(void) {
  uint8_t recipient = SETUP[0] & 0x1F;
  uint8_t request = SETUP[1];
  uint16_t value = (uint16_t)(SETUP[2] | (SETUP[3] << 8));
  uint8_t index = SETUP[4];

  switch (request) {
    case 0:                               // GET_STATUS
      ep0_buf[0] = 0;
      ep0_buf[1] = 0;
      if (recipient == 2) {
        if ( ! valid_ep(index)) {
          ep0_stall();
          break;
        }
        ep0_buf[0] = get_halt(index);
      }
      ep0_send(ep0_buf, 2);
      break;

    case 1:                               // CLEAR_FEATURE
    case 3:                               // SET_FEATURE
      if (recipient == 2 && value == 0) { // ENDPOINT_HALT
        if ( ! valid_ep(index)) {
          ep0_stall();
          break;
        }
        set_halt(index, request == 3);
      }
      ep0_ack();                          // Remote wakeup: ignored.
      break;

    case 5:                               // SET_ADDRESS
      // The core finishes the status stage with the old address.
      USBD->DCFG = (USBD->DCFG & ~USB_OTG_DCFG_DAD) |
                   (((uint32_t)value & 0x7F) << USB_OTG_DCFG_DAD_Pos);
      ep0_ack();
      break;

    case 6:                               // GET_DESCRIPTOR
      get_descriptor();
      break;

    case 8:                               // GET_CONFIGURATION
      ep0_buf[0] = usb_configured;
      ep0_send(ep0_buf, 1);
      break;

    case 9:                               // SET_CONFIGURATION
      if (value == 1) {
        cdc_deconfigure();
        cdc_configure();
        ep0_ack();
      }
      else if (value == 0) {
        cdc_deconfigure();
        ep0_ack();
      }
      else {
        ep0_stall();
      }
      break;

    case 10:                              // GET_INTERFACE
      ep0_buf[0] = 0;
      ep0_send(ep0_buf, 1);
      break;

    case 11:                              // SET_INTERFACE
      if (value == 0)
        ep0_ack();
      else
        ep0_stall();
      break;

    default:
      ep0_stall();
      break;
  }
}

static void class_request(void) {
  uint16_t value = (uint16_t)(SETUP[2] | (SETUP[3] << 8));

  switch (SETUP[1]) {
    case 0x20:                            // SET_LINE_CODING
      ep0_out_req = 0x20;                 // Data stage follows.
      break;

    case 0x21:                            // GET_LINE_CODING
      ep0_send(line_coding, sizeof(line_coding));
      break;

    case 0x22:                            // SET_CONTROL_LINE_STATE
      if ((value & 1) && ! (line_state & 1))
        tx_stalled = 0;                   // Port opened, try again.
      line_state = (uint8_t)(value & 3);
      ep0_ack();
      break;

    case 0x23:                            // SEND_BREAK
      ep0_ack();
      break;

    default:
      ep0_stall();
      break;
  }
}

static void ep0_setup(void) {
  uint8_t type = SETUP[0] & 0x60;

  ep0_in_more = 0;
  ep0_out_req = 0;
  ep0_rx_len = 0;

  if (type == 0x00)
    standard_request();
  else if (type == 0x20 && (SETUP[0] & 0x1F) == 1 && SETUP[4] == 0)
    class_request();                      // To the communication interface.
  else
    ep0_stall();
}

/// EP0 OUT transfer done: OUT data stage or status stage of an IN request.
static void ep0_out_done(void) {
  if (ep0_out_req == 0x20) {
    uint8_t i;

    ep0_out_req = 0;
    if (ep0_rx_len >= sizeof(line_coding)) {
      for (i = 0; i < sizeof(line_coding); i++)
        line_coding[i] = ep0_rx[i];
    }
    ep0_ack();
  }
}

/* ---------------------------------------------------------------------- */
/*  Interrupt handling                                                     */
/* ---------------------------------------------------------------------- */

static void bus_reset(void) {
  uint8_t ep;

  cdc_deconfigure();
  usb_suspended = 0;
  ep0_in_more = 0;
  ep0_out_req = 0;

  USBD->DCTL &= ~USB_OTG_DCTL_RWUSIG;
  flush_tx_fifo(0x10);

  for (ep = 0; ep < 4; ep++) {
    INEP(ep)->DIEPINT = 0xFFFF;
    OUTEP(ep)->DOEPINT = 0xFFFF;
  }
  INEP(0)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
  OUTEP(0)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;

  USBD->DAINTMSK = (1UL << 0) | (1UL << 16);
  USBD->DOEPMSK = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM;
  USBD->DIEPMSK = USB_OTG_DIEPMSK_XFRCM;
  USBD->DCFG &= ~USB_OTG_DCFG_DAD;

  ep0_arm_out();
}

static void enum_done(void) {
  // Full speed: EP0 max packet size 64 (MPSIZ = 0).
  INEP(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
  USBD->DCTL |= USB_OTG_DCTL_CGINAK;
}

static void rx_fifo_level(void) {
  uint32_t sts = USBx->GRXSTSP;
  uint8_t ep = (uint8_t)(sts & USB_OTG_GRXSTSP_EPNUM);
  uint16_t bcnt = (uint16_t)((sts & USB_OTG_GRXSTSP_BCNT) >>
                             USB_OTG_GRXSTSP_BCNT_Pos);
  uint8_t pktsts = (uint8_t)((sts & USB_OTG_GRXSTSP_PKTSTS) >>
                             USB_OTG_GRXSTSP_PKTSTS_Pos);
  uint16_t i;

  if (pktsts == PKTSTS_SETUP_DATA) {
    setup_words[0] = DFIFO(0);
    setup_words[1] = DFIFO(0);
    for (i = 8; i < bcnt; i += 4)         // Never, SETUP is 8 bytes.
      (void)DFIFO(0);
  }
  else if (pktsts == PKTSTS_OUT_DATA && ep == CDC_DATA_EP) {
    rx_read_packet(bcnt);
  }
  else if (pktsts == PKTSTS_OUT_DATA && ep == 0) {
    for (i = 0; i < bcnt; i += 4) {
      uint32_t w = DFIFO(0);
      uint8_t k;

      for (k = 0; k < 4 && i + k < bcnt; k++) {
        if (ep0_rx_len < sizeof(ep0_rx))
          ep0_rx[ep0_rx_len++] = (uint8_t)w;
        w >>= 8;
      }
    }
  }
  else {
    for (i = 0; i < bcnt; i += 4)         // Unexpected data: drain it.
      (void)DFIFO(0);
  }
}

static void out_ep_irq(void) {
  uint32_t daint = daint_pending();

  if (daint & (1UL << 16)) {
    uint32_t i = OUTEP(0)->DOEPINT;

    OUTEP(0)->DOEPINT = i;
    if (i & USB_OTG_DOEPINT_XFRC)
      ep0_out_done();
    if (i & USB_OTG_DOEPINT_STUP)
      ep0_setup();
    if (i & (USB_OTG_DOEPINT_XFRC | USB_OTG_DOEPINT_STUP))
      ep0_arm_out();
  }
  if (daint & (1UL << (16 + CDC_DATA_EP))) {
    uint32_t i = OUTEP(CDC_DATA_EP)->DOEPINT;

    OUTEP(CDC_DATA_EP)->DOEPINT = i;
    if (i & USB_OTG_DOEPINT_XFRC) {
      rx_armed = 0;
      rx_arm();
    }
  }
}

static void in_ep_irq(void) {
  uint32_t daint = daint_pending();

  if (daint & (1UL << 0)) {
    uint32_t i = INEP(0)->DIEPINT;

    INEP(0)->DIEPINT = i;
    if ((i & USB_OTG_DIEPINT_XFRC) && ep0_in_more)
      ep0_in_next();
  }
  if (daint & (1UL << CDC_DATA_EP)) {
    uint32_t i = INEP(CDC_DATA_EP)->DIEPINT;

    INEP(CDC_DATA_EP)->DIEPINT = i;
    if (i & USB_OTG_DIEPINT_XFRC) {
      tx_busy = 0;
      tx_stalled = 0;                     // The host reads.
      tx_kick();
    }
  }
  if (daint & (1UL << CDC_NOTIFY_EP)) {
    uint32_t i = INEP(CDC_NOTIFY_EP)->DIEPINT;

    INEP(CDC_NOTIFY_EP)->DIEPINT = i;
  }
}

TEACUP_HOT
static void usb_irq(void) {
  uint32_t sts = gint_pending();

  if (sts & USB_OTG_GINTSTS_USBRST) {
    USBx->GINTSTS = USB_OTG_GINTSTS_USBRST;
    bus_reset();
  }
  if (sts & USB_OTG_GINTSTS_ENUMDNE) {
    USBx->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
    enum_done();
  }
  if (sts & USB_OTG_GINTSTS_USBSUSP) {
    USBx->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
    usb_suspended = 1;                    // Or unplugged, see top.
  }
  if (sts & USB_OTG_GINTSTS_WKUINT) {
    USBx->GINTSTS = USB_OTG_GINTSTS_WKUINT;
    usb_suspended = 0;
  }

  // Received packets. STUP and XFRC of OUT endpoints show up only after
  // their status entry was popped, so look at GINTSTS again afterwards.
  while (USBx->GINTSTS & USB_OTG_GINTSTS_RXFLVL)
    rx_fifo_level();

  sts = gint_pending();
  if (sts & USB_OTG_GINTSTS_OEPINT)
    out_ep_irq();
  if (sts & USB_OTG_GINTSTS_IEPINT)
    in_ep_irq();
}

TEACUP_HOT
void OTG_FS_IRQHandler(void) {
  usb_irq();
}

/* ---------------------------------------------------------------------- */
/*  Interface                                                              */
/* ---------------------------------------------------------------------- */

void usb_cdc_init(void) {
  // No 48 MHz without PLL.
  if (cpu_clock_source == CPU_CLOCK_FAILED)
    return;

  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
  (void)RCC->AHB2ENR;
  // Clean state, e.g. after a bootloader used USB.
  RCC->AHB2RSTR |= RCC_AHB2RSTR_OTGFSRST;
  RCC->AHB2RSTR &= ~RCC_AHB2RSTR_OTGFSRST;

  SET_AF(PA_11, 10);                      // DM
  SET_AF(PA_12, 10);                      // DP

  USBx->GAHBCFG = 0;
  USBx->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

  // Core soft reset.
  if ( ! wait_flag(&USBx->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL,
                   USB_OTG_GRSTCTL_AHBIDL, 10))
    return;
  USBx->GRSTCTL = USB_OTG_GRSTCTL_CSRST;
  if ( ! wait_flag(&USBx->GRSTCTL, USB_OTG_GRSTCTL_CSRST, 0, 10))
    return;
  delay_us(3);

  // Device mode, turnaround time 6 for AHB >= 32 MHz (RM0368 table 133).
  USBx->GUSBCFG = (USBx->GUSBCFG & ~(USB_OTG_GUSBCFG_FHMOD |
                                     USB_OTG_GUSBCFG_FDMOD |
                                     USB_OTG_GUSBCFG_TRDT)) |
                  USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL |
                  (6UL << USB_OTG_GUSBCFG_TRDT_Pos);
  if ( ! wait_flag(&USBx->GINTSTS, USB_OTG_GINTSTS_CMOD, 0, 50))
    return;

  // Transceiver on, no VBUS sensing (B session always valid).
  USBx->GCCFG = USB_OTG_GCCFG_PWRDWN | USB_OTG_GCCFG_NOVBUSSENS;
  PCGCCTL = 0;

  // Full speed, internal PHY. Stay disconnected until set up.
  USBD->DCFG = 3UL;
  USBD->DCTL = USB_OTG_DCTL_SDIS;

  USBx->GRXFSIZ = FIFO_RX_WORDS;
  USBx->DIEPTXF0_HNPTXFSIZ = ((uint32_t)FIFO_TX0_WORDS << 16) | FIFO_RX_WORDS;
  USBx->DIEPTXF[0] = ((uint32_t)FIFO_TX1_WORDS << 16) |
                     (FIFO_RX_WORDS + FIFO_TX0_WORDS);
  USBx->DIEPTXF[1] = ((uint32_t)FIFO_TX2_WORDS << 16) |
                     (FIFO_RX_WORDS + FIFO_TX0_WORDS + FIFO_TX1_WORDS);
  USBx->DIEPTXF[2] = ((uint32_t)FIFO_TX2_WORDS << 16) |
                     (FIFO_RX_WORDS + FIFO_TX0_WORDS + FIFO_TX1_WORDS +
                      FIFO_TX2_WORDS);
  flush_tx_fifo(0x10);
  flush_rx_fifo();

  USBD->DIEPMSK = 0;
  USBD->DOEPMSK = 0;
  USBD->DAINTMSK = 0;
  cdc_deconfigure();

  USBx->GINTSTS = 0xFFFFFFFFUL;
  USBx->GINTMSK = USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM |
                  USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM |
                  USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_IEPINT |
                  USB_OTG_GINTMSK_OEPINT;
  USBx->GAHBCFG = USB_OTG_GAHBCFG_GINT;

  NVIC_SetPriority(OTG_FS_IRQn, IRQ_PRIO_SERIAL);
  NVIC_ClearPendingIRQ(OTG_FS_IRQn);
  NVIC_EnableIRQ(OTG_FS_IRQn);
  usb_ok = 1;

  // Pull-up on DP: the host sees the device now. After a reset (M999,
  // watchdog) DP was released for a few milliseconds, so the host sees a
  // replug and enumerates again.
  delay_us(3000);
  USBD->DCTL &= ~USB_OTG_DCTL_SDIS;
}

uint16_t usb_cdc_rxchars(void) {
  return rx_count();
}

uint8_t usb_cdc_popchar(void) {
  uint8_t c = 0;
  uint16_t tail = rxtail;

  if (rxhead != tail) {
    c = rxbuf[tail];
    rxtail = (tail + 1) & (USB_RX_BUFSIZE - 1);
  }

  // Room for another packet? Then let the host send again.
  if ( ! rx_armed) {
    ATOMIC_START_NOSTEP();
      rx_arm();
    ATOMIC_END_NOSTEP();
  }
  return c;
}

void usb_cdc_writechar(uint8_t data) {
  uint8_t mode = WAIT_SPIN;
  uint8_t waiting = 0;
  uint32_t start = 0;

  if ( ! usb_ok)
    return;

  for (;;) {
    uint8_t full = 0;

    ATOMIC_START_NOSTEP();
      if ( ! usb_configured || usb_suspended) {
        // Nobody there: drop.
      }
      else if (tx_count() < USB_TX_BUFSIZE - 1) {
        uint16_t head = txhead;

        txbuf[head] = data;
        txhead = (head + 1) & (USB_TX_BUFSIZE - 1);
        tx_kick();
      }
      else if ( ! tx_stalled && (line_state & 1)) {
        full = 1;                         // Host is there, wait for it.
      }
    ATOMIC_END_NOSTEP();

    if ( ! full)
      return;

    if ( ! waiting) {
      waiting = 1;
      start = DWT->CYCCNT;
      mode = wait_mode();                 // Outside ATOMIC, it sets BASEPRI.
      if (mode == WAIT_NONE)
        return;
    }
    else if (DWT->CYCCNT - start > MS_TO_CYCLES(USB_TX_TIMEOUT_MS)) {
      tx_stalled = 1;                     // Until the host reads again.
      return;
    }

    if (mode == WAIT_POLL) {
      ATOMIC_START_NOSTEP();
        usb_poll();
      ATOMIC_END_NOSTEP();
    }
  }
}

void usb_cdc_flush(void) {
  uint8_t mode;
  uint32_t start = DWT->CYCCNT;

  if ( ! usb_ok)
    return;
  mode = wait_mode();
  if (mode == WAIT_NONE)
    return;

  for (;;) {
    uint8_t done;

    ATOMIC_START_NOSTEP();
      if (mode == WAIT_POLL)
        usb_poll();
      done = ! usb_configured || usb_suspended || tx_stalled ||
             ! (line_state & 1) || (tx_count() == 0 && ! tx_busy);
    ATOMIC_END_NOSTEP();

    if (done)
      return;
    if (DWT->CYCCNT - start > MS_TO_CYCLES(USB_FLUSH_TIMEOUT_MS)) {
      tx_stalled = 1;
      return;
    }
  }
}

int16_t usb_cdc_rx_poll(void) {
  if ( ! usb_ok)
    return -1;

  ATOMIC_START_NOSTEP();
    usb_poll();
  ATOMIC_END_NOSTEP();

  if (rx_count() == 0)
    return -1;
  return usb_cdc_popchar();
}

uint8_t usb_cdc_connected(void) {
  return usb_configured && ! usb_suspended && (line_state & 1);
}

#endif /* USB_CDC */
