/** \file
  \brief Persistent storage in Flash ("EEPROM emulation").

  STM32F401/F411 have no EEPROM. The last Flash sector holds settings
  records and print statistics records (version FLASH_STORE_STATS),
  appended one after the other (wear levelling: the sector is
  erased only when full, a 128 kB sector takes several hundred saves).

    STM32F401xC (256 kB): sector 5, 0x08020000, 128 kB
    STM32F401xE, STM32F411xE (512 kB): sector 7, 0x08060000, 128 kB

  The linker must not place code there: the ICF files in cmsis/linker end
  the ROM region before this sector.

  Record layout, all words:
    magic | length (16) + version (16) | CRC-32 of payload | payload...
  The magic word is programmed last. A save interrupted by a reset leaves
  an incomplete record without magic; it's ignored and the sector gets
  erased with the next save.

  Erasing and programming run from RAM with interrupts disabled: the Flash
  can't be read while it's busy, so code running from Flash would stall
  anyway. The watchdog is fed in the wait loops.
*/

#include "flash_store.h"

#include <string.h>
#include "arch.h"
#include "config_wrapper.h"
#include "spi_flash.h"

#ifndef FLASH_STORE_SECTOR
  #if defined STM32F401xC
    #define FLASH_STORE_SECTOR  5
    #define FLASH_STORE_ADDR    0x08020000UL
  #else
    #define FLASH_STORE_SECTOR  7
    #define FLASH_STORE_ADDR    0x08060000UL
  #endif
#endif
#ifndef FLASH_STORE_SIZE
  /// Usable size. Smaller values (tests) make the sector fill up earlier.
  #define FLASH_STORE_SIZE      0x20000UL
#endif

#define STORE_MAGIC   0x53505443UL      // "CTPS"
#define ERASED        0xFFFFFFFFUL
#define HEADER_WORDS  3

#define STORE_WORD(i) (((volatile const uint32_t *)FLASH_STORE_ADDR)[i])

/*
  With SPI_FLASH_SETTINGS and a chip found, the records go to the first
  4 kB sector of the SPI flash instead: no CPU stall while erasing (the
  printer keeps running, M500 works during a print), internal Flash isn't
  worn. Same record format.
*/
#ifndef SPI_SETTINGS_ADDR
  #define SPI_SETTINGS_ADDR   0x000000UL
  #define SPI_SETTINGS_SIZE   0x1000UL
#endif

static uint8_t on_spi(void) {
  #if defined SPI_FLASH && defined SPI_FLASH_SETTINGS
    return spi_flash_present();
  #else
    return 0;
  #endif
}

static uint32_t store_size(void) {
  #if defined SPI_FLASH && defined SPI_FLASH_SETTINGS
    if (on_spi())
      return SPI_SETTINGS_SIZE;
  #endif
  return FLASH_STORE_SIZE;
}

static uint32_t store_word(uint32_t i) {
  #if defined SPI_FLASH && defined SPI_FLASH_SETTINGS
    if (on_spi()) {
      uint32_t w;

      spi_flash_read(SPI_SETTINGS_ADDR + 4UL * i, &w, 4);
      return w;
    }
  #endif
  return STORE_WORD(i);
}

uint8_t flash_store_on_spi(void) {
  return on_spi();
}

uint32_t flash_store_crc32(const void *data, uint32_t length) {
  const uint8_t *p = (const uint8_t *)data;
  uint32_t crc = 0xFFFFFFFFUL;
  uint8_t bit;

  while (length--) {
    crc ^= *p++;
    for (bit = 0; bit < 8; bit++)
      crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
  }
  return ~crc;
}

/**
  Scan the sector.
  \param last  Word index of the newest valid settings record header, or -1.
  \param last_stats  The same for the statistics record (FLASH_STORE_STATS).
  \param free  Word index of the first erased word after all records, or
               -1 if the rest of the sector is dirty (interrupted write).
*/
static void scan(int32_t *last, int32_t *last_stats, int32_t *free_at) {
  uint32_t i = 0;
  const uint32_t words = store_size() / 4;

  *last = -1;
  *last_stats = -1;
  *free_at = -1;

  while (i + HEADER_WORDS <= words) {
    uint32_t magic = store_word(i);
    uint32_t header, len_words;

    if (magic == ERASED) {
      // Erased header: free space if the header words are all erased.
      if (store_word(i + 1) == ERASED && store_word(i + 2) == ERASED)
        *free_at = (int32_t)i;
      return;
    }
    if (magic != STORE_MAGIC)
      return;                                   // Dirty, needs erase.

    header = store_word(i + 1);
    len_words = ((header & 0xFFFFUL) + 3) / 4;
    if (i + HEADER_WORDS + len_words > words)
      return;
    if ((header >> 16) == FLASH_STORE_STATS)
      *last_stats = (int32_t)i;
    else
      *last = (int32_t)i;
    i += HEADER_WORDS + len_words;
  }
}

uint8_t flash_store_read(void *data, uint16_t length, uint16_t version) {
  int32_t last, last_stats, free_at;
  uint32_t header;

  scan(&last, &last_stats, &free_at);
  if (version == FLASH_STORE_STATS)
    last = last_stats;
  if (last < 0)
    return 0;

  header = store_word((uint32_t)last + 1);
  if ((header & 0xFFFFUL) != length || (header >> 16) != version)
    return 0;

  #if defined SPI_FLASH && defined SPI_FLASH_SETTINGS
    if (on_spi())
      spi_flash_read(SPI_SETTINGS_ADDR + 4UL * (uint32_t)(last + HEADER_WORDS),
                     data, length);
    else
  #endif
  memcpy(data, (const void *)(FLASH_STORE_ADDR + 4UL * (uint32_t)(last + HEADER_WORDS)),
         length);

  return flash_store_crc32(data, length) == store_word((uint32_t)last + 2);
}

/*
  Low level Flash access, executed from RAM. Only register accesses in
  here, no calls into Flash.
*/
#define FLASH_KEY1  0x45670123UL
#define FLASH_KEY2  0xCDEF89ABUL

static TEACUP_RAMFUNC uint32_t flash_wait(void) {
  while (FLASH->SR & FLASH_SR_BSY) {
    #ifdef USE_WATCHDOG
      IWDG->KR = 0xAAAA;
    #endif
  }
  return FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR |
                      FLASH_SR_WRPERR | FLASH_SR_OPERR);
}

/**
  Optionally erase the sector, then program 'count' words at 'dst'.
  The first word is programmed last (it's the magic).
*/
static TEACUP_RAMFUNC uint32_t flash_program(uint8_t erase,
                                             volatile uint32_t *dst,
                                             const uint32_t *src,
                                             uint32_t count) {
  uint32_t err = 0, i;

  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
  FLASH->SR = FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_WRPERR |
              FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR;

  if (erase) {
    FLASH->CR = FLASH_CR_SER | FLASH_CR_PSIZE_1 |
                ((uint32_t)FLASH_STORE_SECTOR << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    err = flash_wait();
    FLASH->CR = 0;
  }

  if ( ! err) {
    FLASH->CR = FLASH_CR_PG | FLASH_CR_PSIZE_1;      // 32 bit programming.
    for (i = 1; i < count && ! err; i++) {
      dst[i] = src[i];
      err = flash_wait();
    }
    if ( ! err) {
      dst[0] = src[0];
      err = flash_wait();
    }
    FLASH->CR = 0;
  }

  FLASH->CR = FLASH_CR_LOCK;

  // Caches might hold old Flash contents.
  FLASH->ACR &= ~(FLASH_ACR_DCEN | FLASH_ACR_ICEN);
  FLASH->ACR |= FLASH_ACR_DCRST | FLASH_ACR_ICRST;
  FLASH->ACR &= ~(FLASH_ACR_DCRST | FLASH_ACR_ICRST);
  FLASH->ACR |= FLASH_ACR_DCEN | FLASH_ACR_ICEN;

  return err;
}

uint8_t flash_store_write(const void *data, uint16_t length, uint16_t version) {
  // Header + payload, word aligned. Settings are well below 1 kB.
  static uint32_t buf[HEADER_WORDS + 256];
  // Erasing: the newest record of the other kind (settings / statistics)
  // is written back first.
  static uint32_t keep[HEADER_WORDS + 256];
  uint32_t words = HEADER_WORDS + (length + 3U) / 4U;
  uint32_t keep_words = 0;
  int32_t last, last_stats, other, free_at;
  uint8_t erase = 0;
  uint32_t err, i;
  uint32_t primask;

  if (length > 1024)
    return 0;

  for (i = 0; i < words; i++)
    buf[i] = ERASED;
  buf[0] = STORE_MAGIC;
  buf[1] = (uint32_t)length | ((uint32_t)version << 16);
  buf[2] = flash_store_crc32(data, length);
  for (i = 0; i < length; i++)
    ((uint8_t *)&buf[HEADER_WORDS])[i] = ((const uint8_t *)data)[i];

  scan(&last, &last_stats, &free_at);
  if (free_at < 0 || (uint32_t)free_at + words > store_size() / 4) {
    erase = 1;
    other = (version == FLASH_STORE_STATS) ? last : last_stats;
    if (other >= 0) {
      keep_words = HEADER_WORDS +
                   ((store_word((uint32_t)other + 1) & 0xFFFFUL) + 3) / 4;
      if (keep_words > HEADER_WORDS + 256)
        keep_words = 0;
      for (i = 0; i < keep_words; i++)
        keep[i] = store_word((uint32_t)other + i);
    }
    free_at = (int32_t)keep_words;
  }

  #if defined SPI_FLASH && defined SPI_FLASH_SETTINGS
    if (on_spi()) {
      uint32_t addr = SPI_SETTINGS_ADDR + 4UL * (uint32_t)free_at;

      (void)primask;
      (void)err;
      if (erase) {
        spi_flash_erase_sector(SPI_SETTINGS_ADDR);
        if (keep_words) {
          spi_flash_program(SPI_SETTINGS_ADDR + 4, &keep[1], (keep_words - 1) * 4);
          spi_flash_program(SPI_SETTINGS_ADDR, &keep[0], 4);
        }
      }
      // Magic last, like with internal Flash.
      spi_flash_program(addr + 4, &buf[1], (words - 1) * 4);
      spi_flash_program(addr, &buf[0], 4);
      for (i = 0; i < words; i++)
        if (store_word((uint32_t)free_at + i) != buf[i])
          return 0;
      return 1;
    }
  #endif

  primask = __get_PRIMASK();
  __disable_irq();
  err = 0;
  if (erase && keep_words) {
    err = flash_program(1, (volatile uint32_t *)FLASH_STORE_ADDR, keep, keep_words);
    erase = 0;
  }
  if ( ! err)
    err = flash_program(erase,
                        (volatile uint32_t *)(FLASH_STORE_ADDR + 4UL * (uint32_t)free_at),
                        buf, words);
  __set_PRIMASK(primask);

  if (err)
    return 0;

  // Verify.
  for (i = 0; i < words; i++)
    if (STORE_WORD(free_at + i) != buf[i])
      return 0;
  return 1;
}
