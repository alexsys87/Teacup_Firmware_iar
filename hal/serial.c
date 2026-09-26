/** \file
  \brief Serial subsystem: distributes host I/O over the host ports.

  The UART (uart.c) is always there, USB CDC (usb_cdc.c) is added with
  USB_CDC. See serial.h for how output is routed.
*/

#include "serial.h"
#include "uart.h"
#ifdef USB_CDC
  #include "usb_cdc.h"
#endif

/// Ports serial_writechar() sends to.
static volatile uint8_t out_mask = SERIAL_MASK_ALL;

void serial_init(void) {
  uart_init();
  #ifdef USB_CDC
    usb_cdc_init();
  #endif
}

uint16_t serial_port_rxchars(uint8_t port) {
  #ifdef USB_CDC
    if (port == SERIAL_USB_PORT)
      return usb_cdc_rxchars();
  #endif
  return (port == SERIAL_UART_PORT) ? uart_rxchars() : 0;
}

uint8_t serial_port_popchar(uint8_t port) {
  #ifdef USB_CDC
    if (port == SERIAL_USB_PORT)
      return usb_cdc_popchar();
  #endif
  return (port == SERIAL_UART_PORT) ? uart_popchar() : 0;
}

uint8_t serial_set_output(uint8_t mask) {
  uint8_t old = out_mask;

  out_mask = mask & SERIAL_MASK_ALL;
  return old;
}

void serial_writechar(uint8_t data) {
  uint8_t mask = out_mask;

  if (mask & SERIAL_MASK(SERIAL_UART_PORT))
    uart_writechar(data);
  #ifdef USB_CDC
    if (mask & SERIAL_MASK(SERIAL_USB_PORT))
      usb_cdc_writechar(data);
  #endif
}

void serial_writestr(char const *data) {
  char c;

  while ((c = *data++) != 0)
    serial_writechar((uint8_t)c);
}

void serial_flush(void) {
  uart_flush();
  #ifdef USB_CDC
    usb_cdc_flush();
  #endif
}

int16_t serial_rx_poll(uint8_t *port) {
  int16_t c = uart_rx_poll();

  if (c >= 0) {
    *port = SERIAL_UART_PORT;
    return c;
  }
  #ifdef USB_CDC
    c = usb_cdc_rx_poll();
    if (c >= 0) {
      *port = SERIAL_USB_PORT;
      return c;
    }
  #endif
  return -1;
}
