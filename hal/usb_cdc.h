/** \file
  \brief USB CDC ACM virtual COM port on the OTG_FS core (PA11/PA12).

  Low level driver of the USB host port. Firmware code doesn't call it
  directly, it uses serial.h, which sends to and receives from all host
  ports (UART and USB CDC).

  Enabled with USB_CDC in the board file. Options (board file):

    USB_VID, USB_PID        vendor / product ID, default 0x0483 / 0x5740
                            (STMicroelectronics Virtual COM Port)
    USB_MANUFACTURER        manufacturer string, default "Teacup"
    USB_PRODUCT             product string, default "Teacup 3D printer"

  The serial number string is made from the chip's unique ID, so every
  board gets its own stable /dev/serial/by-id/... name (Linux) or COM
  port number (Windows).
*/

#ifndef _USB_CDC_H
#define _USB_CDC_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef USB_CDC

/// Initialise the OTG_FS core and connect to the bus.
void usb_cdc_init(void);

/// Number of characters waiting in the receive buffer.
uint16_t usb_cdc_rxchars(void);

/// Read one character. Returns 0 if nothing is available.
uint8_t usb_cdc_popchar(void);

/**
  Send one character. While the host has the port open (DTR set) and the
  transmit buffer is full, this waits up to USB_TX_TIMEOUT_MS for the host
  to read. Without a host (cable unplugged, port closed, host not reading)
  characters are dropped, the printer never blocks on USB.
*/
void usb_cdc_writechar(uint8_t data);

/// Wait until everything is sent, with timeout. Works with interrupts off.
void usb_cdc_flush(void);

/// Polled receive for use with interrupts disabled. -1 = no character.
int16_t usb_cdc_rx_poll(void);

/// 1 if configured by the host, not suspended and the port is open (DTR).
uint8_t usb_cdc_connected(void);

#endif /* USB_CDC */

#endif /* _USB_CDC_H */
