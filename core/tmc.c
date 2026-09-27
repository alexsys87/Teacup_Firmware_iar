/** \file
  \brief TMC2208 / TMC2209 stepper drivers configured over UART (TMC_UART).

  Datagrams (TMC2209 datasheet, chapter 4):
    write: 0x05, address, register | 0x80, 4 data bytes (MSB first), CRC8
    read:  0x05, address, register, CRC8
    reply: 0x05, 0xFF, register, 4 data bytes, CRC8
  The CRC is CRC8 with polynomial x^8 + x^2 + x + 1, bits LSB first.

  Each axis with a TMC_<axis>_ADDR gets:
    GCONF       UART control (pdn_disable), microsteps from MRES
                (mstep_reg_select), internal current reference,
                en_SpreadCycle unless stealthChop (M569)
    IHOLD_IRUN  run current (M906) and hold current (TMC_HOLD_MULTIPLIER)
                from TMC_RSENSE, vsense for low currents
    CHOPCONF    TOFF 3, HSTRT 1, HEND -1, TBL 24 clocks, MRES, interpolation
    PWMCONF     stealthChop with automatic tuning (Marlin's defaults)
    TPWMTHRS 0  stealthChop at all speeds (if selected), TPOWERDOWN 128
  Writes are verified with the transmission counter IFCNT.

  Without motor supply the drivers don't answer and forget everything.
  tmc_tick() looks at GSTAT once a second (while the power supply is on)
  and configures a driver again after a reset or undervoltage, the first
  move after power on waits for the configuration (tmc_ready()).
*/

#include "tmc.h"

#ifdef TMC_UART

#include "tmc_uart.h"
#include "arch.h"
#include "pinio.h"
#include "settings.h"
#include "serial.h"
#include "sersendf.h"
#include "delay.h"

#ifndef TMC_RSENSE
  #define TMC_RSENSE 0.11               ///< Sense resistor, ohm.
#endif
#ifndef TMC_HOLD_MULTIPLIER
  #define TMC_HOLD_MULTIPLIER 0.5       ///< Hold current / run current.
#endif
#ifndef TMC_MICROSTEPS
  #define TMC_MICROSTEPS 16
#endif
#ifndef TMC_X_MICROSTEPS
  #define TMC_X_MICROSTEPS TMC_MICROSTEPS
#endif
#ifndef TMC_Y_MICROSTEPS
  #define TMC_Y_MICROSTEPS TMC_MICROSTEPS
#endif
#ifndef TMC_Z_MICROSTEPS
  #define TMC_Z_MICROSTEPS TMC_MICROSTEPS
#endif
#ifndef TMC_E_MICROSTEPS
  #define TMC_E_MICROSTEPS TMC_MICROSTEPS
#endif

#if ! defined TMC_X_ADDR && ! defined TMC_Y_ADDR && ! defined TMC_Z_ADDR && \
    ! defined TMC_E_ADDR
  #error TMC_UART needs at least one of TMC_X_ADDR .. TMC_E_ADDR.
#endif

#define REG_GCONF       0x00
#define REG_GSTAT       0x01
#define REG_IFCNT       0x02
#define REG_IOIN        0x06
#define REG_IHOLD_IRUN  0x10
#define REG_TPOWERDOWN  0x11
#define REG_TPWMTHRS    0x13
#define REG_CHOPCONF    0x6C
#define REG_DRV_STATUS  0x6F
#define REG_PWMCONF     0x70

#define GCONF_EN_SPREADCYCLE  (1UL << 2)
#define GCONF_PDN_DISABLE     (1UL << 6)
#define GCONF_MSTEP_REG_SEL   (1UL << 7)
#define GCONF_MULTISTEP_FILT  (1UL << 8)

#define GSTAT_RESET           0x01U
#define GSTAT_DRV_ERR         0x02U
#define GSTAT_UV_CP           0x04U

/// PWMCONF: PWM_OFS 36, PWM_GRAD 14, freq 2/683, autoscale, autograd,
/// PWM_REG 8, PWM_LIM 12.
#define PWMCONF_VALUE         0xC80D0E24UL

/// Writes of one configuration, checked against IFCNT.
#define CONFIG_WRITES         7

/// Reply timeout: 12 characters at 115200 baud are about 1 ms.
#define TMC_TIMEOUT_CYCLES    (3UL * (F_CPU / 1000UL))

typedef struct {
  uint8_t  addr;
  uint8_t  axis;
  uint16_t microsteps;
} tmc_cfg_t;

static const tmc_cfg_t tmc_cfg[] = {
  #ifdef TMC_X_ADDR
    { TMC_X_ADDR, X, TMC_X_MICROSTEPS },
  #endif
  #ifdef TMC_Y_ADDR
    { TMC_Y_ADDR, Y, TMC_Y_MICROSTEPS },
  #endif
  #ifdef TMC_Z_ADDR
    { TMC_Z_ADDR, Z, TMC_Z_MICROSTEPS },
  #endif
  #ifdef TMC_E_ADDR
    { TMC_E_ADDR, E, TMC_E_MICROSTEPS },
  #endif
};
#define TMC_COUNT (sizeof(tmc_cfg) / sizeof(tmc_cfg[0]))

enum { TMC_NONE = 0, TMC_OK, TMC_WRITE_ERROR };

/// Per driver: TMC_NONE (not answering), TMC_OK, TMC_WRITE_ERROR.
static uint8_t tmc_state[TMC_COUNT];

/// Set by power_on(): configure before the next move.
volatile uint8_t tmc_power_up;

static const char axis_char[4] = { 'X', 'Y', 'Z', 'E' };

static uint8_t tmc_crc(const uint8_t *d, uint8_t n) {
  uint8_t crc = 0, i, j, b;

  for (i = 0; i < n; i++) {
    b = d[i];
    for (j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (b & 1U))
        crc = (uint8_t)((crc << 1) ^ 0x07U);
      else
        crc = (uint8_t)(crc << 1);
      b >>= 1;
    }
  }
  return crc;
}

static void tmc_write(uint8_t addr, uint8_t reg, uint32_t v) {
  uint8_t d[8];

  d[0] = 0x05;
  d[1] = addr;
  d[2] = reg | 0x80U;
  d[3] = (uint8_t)(v >> 24);
  d[4] = (uint8_t)(v >> 16);
  d[5] = (uint8_t)(v >> 8);
  d[6] = (uint8_t)v;
  d[7] = tmc_crc(d, 7);
  tmc_uart_send(d, 8);
  tmc_uart_flush_rx();                    // Our echo.
}

/**
  Read a register. \return 1 with *v set, 0 without valid reply.

  The echo of the request (second byte = address, not 0xFF) is skipped by
  the frame sync, so it doesn't matter whether the USART received it.
*/
static uint8_t tmc_read(uint8_t addr, uint8_t reg, uint32_t *v) {
  uint8_t req[4], buf[8], n = 0;
  uint32_t start;

  req[0] = 0x05;
  req[1] = addr;
  req[2] = reg;
  req[3] = tmc_crc(req, 3);
  tmc_uart_flush_rx();
  tmc_uart_send(req, 4);

  start = DWT->CYCCNT;
  while (DWT->CYCCNT - start < TMC_TIMEOUT_CYCLES) {
    int16_t c = tmc_uart_getc();

    if (c < 0)
      continue;
    if (n == 1 && c != 0xFF)
      n = 0;                              // Not a reply, resync.
    if (n == 0 && c != 0x05)
      continue;
    buf[n++] = (uint8_t)c;
    if (n == 8) {
      if (buf[2] != reg || tmc_crc(buf, 7) != buf[7]) {
        n = 0;                            // Something else, go on.
        continue;
      }
      *v = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) |
           ((uint32_t)buf[5] << 8) | buf[6];
      return 1;
    }
  }
  return 0;
}

/// Current scale CS (0..31) and vsense for a run current, like Marlin.
static uint8_t current_to_cs(uint32_t ma, uint8_t *vsense) {
  float cs = 32.f * 1.41421356f * (float)ma / 1000.f *
             (float)(TMC_RSENSE + 0.02) / 0.325f - 1.f;

  *vsense = 0;
  if (cs < 16.f) {
    *vsense = 1;
    cs = 32.f * 1.41421356f * (float)ma / 1000.f *
         (float)(TMC_RSENSE + 0.02) / 0.180f - 1.f;
  }
  if (cs < 0.f)
    cs = 0.f;
  if (cs > 31.f)
    cs = 31.f;
  return (uint8_t)cs;
}

/// RMS current in mA of a current scale.
static uint32_t cs_to_current(uint8_t cs, uint8_t vsense) {
  float vfs = vsense ? 0.180f : 0.325f;

  return (uint32_t)((float)(cs + 1) / 32.f * vfs /
                    (float)(TMC_RSENSE + 0.02) / 1.41421356f * 1000.f + 0.5f);
}

/// MRES of a microstep setting (256 -> 0 .. 1 -> 8).
static uint8_t microsteps_to_mres(uint16_t ms) {
  uint8_t mres = 8;

  while (ms > 1 && mres) {
    ms >>= 1;
    mres--;
  }
  return mres;
}

/// Write the whole configuration of driver i. \return new tmc_state.
static uint8_t tmc_configure(uint8_t i) {
  const tmc_cfg_t *c = &tmc_cfg[i];
  uint32_t before, after, gconf, chopconf;
  uint8_t cs, vsense, hold;

  if ( ! tmc_read(c->addr, REG_IFCNT, &before))
    return TMC_NONE;

  cs = current_to_cs(settings.tmc_current[c->axis], &vsense);
  hold = (uint8_t)((float)cs * (float)TMC_HOLD_MULTIPLIER);

  gconf = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SEL | GCONF_MULTISTEP_FILT;
  if ( ! (settings.tmc_stealth & (1UL << c->axis)))
    gconf |= GCONF_EN_SPREADCYCLE;

  chopconf = 3UL                            // TOFF 3
           | (0UL << 4)                     // HSTRT 1
           | (2UL << 7)                     // HEND -1
           | (1UL << 15)                    // TBL 24 clocks
           | ((uint32_t)vsense << 17)
           | ((uint32_t)microsteps_to_mres(c->microsteps) << 24)
           | (1UL << 28);                   // intpol: 256 microsteps inside

  tmc_write(c->addr, REG_GSTAT, GSTAT_RESET | GSTAT_DRV_ERR | GSTAT_UV_CP);
  tmc_write(c->addr, REG_GCONF, gconf);
  tmc_write(c->addr, REG_IHOLD_IRUN,
            hold | ((uint32_t)cs << 8) | (10UL << 16));
  tmc_write(c->addr, REG_TPOWERDOWN, 128);
  tmc_write(c->addr, REG_CHOPCONF, chopconf);
  tmc_write(c->addr, REG_PWMCONF, PWMCONF_VALUE);
  tmc_write(c->addr, REG_TPWMTHRS, 0);

  if ( ! tmc_read(c->addr, REG_IFCNT, &after))
    return TMC_NONE;
  if (((after - before) & 0xFFU) != CONFIG_WRITES)
    return TMC_WRITE_ERROR;
  return TMC_OK;
}

/// Configure driver i and report changes of its state.
static void tmc_setup(uint8_t i, const char *why) {
  uint8_t old = tmc_state[i];

  tmc_state[i] = tmc_configure(i);
  if (tmc_state[i] == TMC_OK) {
    if (why) {
      sersendf_P(("echo:TMC %c "), axis_char[tmc_cfg[i].axis]);
      serial_writestr(why);
      serial_writestr(", configured\n");
    }
  }
  else if (tmc_state[i] == TMC_WRITE_ERROR)
    sersendf_P(("echo:TMC %c write error\n"), axis_char[tmc_cfg[i].axis]);
  else if (old != TMC_NONE)
    sersendf_P(("echo:TMC %c not responding\n"), axis_char[tmc_cfg[i].axis]);
}

void tmc_init(void) {
  tmc_uart_init();
  // Motor supply off (PS_ON): the drivers don't answer yet, tmc_ready()
  // and tmc_tick() configure them later.
  if (power_is_on())
    tmc_power_up = 1;
}

void tmc_ready(void) {
  uint8_t i;

  tmc_power_up = 0;
  for (i = 0; i < TMC_COUNT; i++)
    tmc_setup(i, NULL);
}

void tmc_tick(void) {
  uint8_t i;
  uint32_t gstat;

  if ( ! power_is_on())
    return;
  if (tmc_power_up) {
    tmc_ready();
    return;
  }
  for (i = 0; i < TMC_COUNT; i++) {
    if ( ! tmc_read(tmc_cfg[i].addr, REG_GSTAT, &gstat)) {
      if (tmc_state[i] != TMC_NONE)
        sersendf_P(("echo:TMC %c not responding\n"), axis_char[tmc_cfg[i].axis]);
      tmc_state[i] = TMC_NONE;
      continue;
    }
    if (gstat & GSTAT_DRV_ERR)
      tmc_setup(i, "driver error (overtemperature / short)");
    else if (gstat & (GSTAT_RESET | GSTAT_UV_CP))
      tmc_setup(i, tmc_state[i] == TMC_NONE ? "found" : "reset");
    else if (tmc_state[i] != TMC_OK)
      tmc_setup(i, "found");
  }
}

void tmc_apply(void) {
  uint8_t i;

  if ( ! power_is_on())
    return;
  for (i = 0; i < TMC_COUNT; i++)
    tmc_setup(i, NULL);
}

void tmc_report_settings(void) {
  uint8_t i, any;

  serial_writestr("echo:  M906");
  for (i = 0; i < TMC_COUNT; i++)
    sersendf_P((" %c%lu"), axis_char[tmc_cfg[i].axis],
               settings.tmc_current[tmc_cfg[i].axis]);
  serial_writechar('\n');
  for (any = 0; any < 2; any++) {
    uint8_t n = 0;

    for (i = 0; i < TMC_COUNT; i++) {
      uint8_t stealth = (settings.tmc_stealth >> tmc_cfg[i].axis) & 1U;

      if (stealth != any)
        continue;
      if (n++ == 0)
        sersendf_P(("echo:  M569 S%su"), any);
      sersendf_P((" %c"), axis_char[tmc_cfg[i].axis]);
    }
    if (n)
      serial_writechar('\n');
  }
}

/// Status flags reported by M122: DRV_STATUS bits, then GSTAT bits.
static const struct {
  uint32_t mask;
  const char *text;
} flags[] = {
  { 1UL << 1, ", OVERTEMPERATURE" },
  { 1UL << 0, ", overtemperature warning" },
  { 3UL << 2, ", SHORT TO GND" },
  { 3UL << 4, ", SHORT (low side)" },
  { 1UL << 6, ", open load A" },
  { 1UL << 7, ", open load B" },
  { GSTAT_UV_CP, ", undervoltage" },
  { GSTAT_DRV_ERR, ", driver error" },
};

void tmc_report(void) {
  uint8_t i;

  for (i = 0; i < TMC_COUNT; i++) {
    const tmc_cfg_t *c = &tmc_cfg[i];
    uint32_t ioin, gconf, ihold_irun, chopconf, drv, gstat;
    uint8_t cs, vsense, k;

    sersendf_P(("echo:TMC %c (address %su): "), axis_char[c->axis], c->addr);
    if ( ! tmc_read(c->addr, REG_IOIN, &ioin) ||
         ! tmc_read(c->addr, REG_GCONF, &gconf) ||
         ! tmc_read(c->addr, REG_IHOLD_IRUN, &ihold_irun) ||
         ! tmc_read(c->addr, REG_CHOPCONF, &chopconf) ||
         ! tmc_read(c->addr, REG_DRV_STATUS, &drv) ||
         ! tmc_read(c->addr, REG_GSTAT, &gstat)) {
      serial_writestr("not responding\n");
      continue;
    }
    cs = (uint8_t)((ihold_irun >> 8) & 0x1FU);
    vsense = (uint8_t)((chopconf >> 17) & 1U);
    sersendf_P(("TMC220%c, "), ((ioin >> 24) == 0x21) ? '9' : '8');
    serial_writestr(tmc_state[i] == TMC_OK ? "ok" : "not configured");
    sersendf_P((", %lu mA (IRUN %su, IHOLD %su), 1/%u, "),
               cs_to_current(cs, vsense), cs, (uint8_t)(ihold_irun & 0x1FU),
               (uint16_t)(256U >> ((chopconf >> 24) & 0x0FU)));
    serial_writestr((gconf & GCONF_EN_SPREADCYCLE) ? "spreadCycle\n"
                                                   : "stealthChop\n");
    sersendf_P(("echo:  CS_ACTUAL %su, "), (uint8_t)((drv >> 16) & 0x1FU));
    serial_writestr((drv & (1UL << 31)) ? "standstill" : "moving");
    for (k = 0; k < sizeof(flags) / sizeof(flags[0]); k++)
      if ((k < 6 ? drv : gstat) & flags[k].mask)
        serial_writestr(flags[k].text);
    serial_writechar('\n');
  }
}

#endif /* TMC_UART */
