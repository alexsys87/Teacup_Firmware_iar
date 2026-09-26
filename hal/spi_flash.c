/** \file
  \brief SPI NOR flash driver (W25Q16 ... W25Q128 and compatibles).

  WeAct "Black Pill" boards have an unpopulated SOIC-8 footprint (U3) on
  the bottom side, wired to SPI1:

    CS  PA4    CLK PA5    DO (MISO) PA6    DI (MOSI) PA7

  Standard commands only, 3 byte addresses (chips up to 16 MB). The chip is
  detected by its JEDEC ID at startup; without a chip everything using it
  falls back (settings to internal Flash, no flash files).

  Waiting for erase/program keeps the clock running (clock_poll()), so
  heaters and host communication continue. The chip select is released
  while waiting, other devices on the bus (SD card, MAX6675) can be used
  meanwhile.
*/

#include "spi_flash.h"

#ifdef SPI_FLASH

#include "spi.h"
#include "pinio.h"
#include "serial.h"
#include "sersendf.h"
#include "clock.h"
#include "watchdog.h"

#define CMD_WRITE_ENABLE  0x06
#define CMD_READ_STATUS   0x05
#define CMD_READ          0x03
#define CMD_PAGE_PROGRAM  0x02
#define CMD_SECTOR_ERASE  0x20
#define CMD_JEDEC_ID      0x9F
#define CMD_RELEASE_PD    0xAB

#define STATUS_BUSY       0x01

static uint32_t flash_size = 0;

static void select_chip(void) {
  spi_speed_max();
  WRITE(SPI_FLASH_CS_PIN, 0);
}

static void deselect_chip(void) {
  WRITE(SPI_FLASH_CS_PIN, 1);
}

static void send_addr(uint8_t cmd, uint32_t addr) {
  spi_rw(cmd);
  spi_rw((uint8_t)(addr >> 16));
  spi_rw((uint8_t)(addr >> 8));
  spi_rw((uint8_t)addr);
}

static uint8_t read_status(void) {
  uint8_t s;

  select_chip();
  spi_rw(CMD_READ_STATUS);
  s = spi_rw(0xFF);
  deselect_chip();
  return s;
}

/// Wait for program/erase to finish, keep the printer running meanwhile.
static void wait_ready(void) {
  while (read_status() & STATUS_BUSY) {
    wd_reset();
    clock_poll();
  }
}

static void write_enable(void) {
  select_chip();
  spi_rw(CMD_WRITE_ENABLE);
  deselect_chip();
}

uint8_t spi_flash_init(void) {
  uint8_t mfr, type, cap;
  uint32_t i;

  WRITE(SPI_FLASH_CS_PIN, 1);
  SET_OUTPUT(SPI_FLASH_CS_PIN);

  // Wake up from deep power down, if in there (needs 3 us).
  select_chip();
  spi_rw(CMD_RELEASE_PD);
  deselect_chip();
  for (i = 0; i < F_CPU / 100000; i++)
    __NOP();

  select_chip();
  spi_rw(CMD_JEDEC_ID);
  mfr = spi_rw(0xFF);
  type = spi_rw(0xFF);
  cap = spi_rw(0xFF);
  deselect_chip();

  if (mfr == 0x00 || mfr == 0xFF || cap < 0x10 || cap > 0x18) {
    flash_size = 0;
    serial_writestr("echo:SPI flash not found\n");
    return 0;
  }
  flash_size = 1UL << cap;
  sersendf_P(("echo:SPI flash: manufacturer %sx type %sx, %lu kB\n"), mfr,
             type, flash_size / 1024);
  return 1;
}

uint8_t spi_flash_present(void) {
  return flash_size != 0;
}

uint32_t spi_flash_size(void) {
  return flash_size;
}

void spi_flash_read(uint32_t addr, void *buf, uint32_t len) {
  uint8_t *p = (uint8_t *)buf;

  select_chip();
  send_addr(CMD_READ, addr);
  while (len--)
    *p++ = spi_rw(0xFF);
  deselect_chip();
}

void spi_flash_program(uint32_t addr, const void *buf, uint32_t len) {
  const uint8_t *p = (const uint8_t *)buf;

  while (len) {
    // Up to the end of the current page.
    uint32_t n = SPI_FLASH_PAGE - (addr & (SPI_FLASH_PAGE - 1));

    if (n > len)
      n = len;
    write_enable();
    select_chip();
    send_addr(CMD_PAGE_PROGRAM, addr);
    addr += n;
    len -= n;
    while (n--)
      spi_rw(*p++);
    deselect_chip();
    wait_ready();
  }
}

void spi_flash_erase_sector(uint32_t addr) {
  write_enable();
  select_chip();
  send_addr(CMD_SECTOR_ERASE, addr & ~(SPI_FLASH_SECTOR - 1));
  deselect_chip();
  wait_ready();
}

#endif /* SPI_FLASH */
