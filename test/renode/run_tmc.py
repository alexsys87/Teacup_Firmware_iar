#!/usr/bin/env python3
"""Renode test of the TMC2209 UART configuration (TMC_UART) on the P3 Steel
configuration. In the test the drivers are on USART6 (PC6), the host stays
on USART2; models/TeacupTMC2209.cs emulates four drivers (X 0, Y 1, Z 2,
E 3) on the single wire, with echo:

  make CHIP=F401 TEST=0 EXTRA="-DTMC_UART=6 -DTMC_UART_TX_PIN=PC_6" BUILD_SUFFIX=_tmc
  run_tmc.py [renode] [teacup.elf]

Checks:
  - after M80 all drivers are configured: GCONF (UART, MRES, stealthChop
    X Y Z / spreadCycle E), IHOLD_IRUN from 800 / 600 mA with 0.11 ohm
    (vsense), CHOPCONF 1/32, PWMCONF, TPOWERDOWN, GSTAT cleared
  - M906 X1000: new IRUN without vsense, report
  - M569 S0 X: spreadCycle on X, report of both modes
  - M122: type, current, microsteps, mode; injected overtemperature
    warning shows up
  - a driver losing power is found by the 1 s check and configured again
  - a missing driver: "not responding", M122 too; back: configured
  - motor supply off (M81), drivers reset: the first move configures them
    before it runs
  - M503 reports M906 / M569, M500 / M502 / M501 store and load them
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0_tmc/teacup.elf')

# USART6 with the Teacup USART model (TC after each character).
repl_src = open(os.path.join(HERE, 'stm32f401_dma.repl')).read()
repl_src = repl_src.replace('usart6: UART.STM32_UART @ sysbus <0x40011400, +0x100>\n    -> nvic@71',
                            'usart6: UART.TeacupSTM32_UART @ sysbus <0x40011400, +0x100>\n'
                            '    frequency: 84000000\n    IRQ -> nvic@71')
assert 'TeacupSTM32_UART @ sysbus <0x40011400' in repl_src
REPL = '/tmp/teacup_tmc_%d.repl' % os.getpid()
open(REPL, 'w').write(repl_src)

OVERSAMPLE, SLOTS, ADC_MAX = 16, 2, 4095
RP, PTS = 4700.0, [(25.0, 100000.0), (150.0, 1641.9), (250.0, 226.15)]
def sh_coef():
    (t1, r1), (t2, r2), (t3, r3) = PTS
    l1, l2, l3 = map(math.log, (r1, r2, r3))
    y1, y2, y3 = (1 / (t + 273.15) for t in (t1, t2, t3))
    g2, g3 = (y2 - y1) / (l2 - l1), (y3 - y1) / (l3 - l1)
    c = (g3 - g2) / (l3 - l2) / (l1 + l2 + l3)
    b = g2 - c * (l1 * l1 + l1 * l2 + l2 * l2)
    return y1 - (b + l1 * l1 * c) * l1, b, c
A_, B_, C_ = sh_coef()
def adc_to_c(adc):
    l = math.log(RP * adc / (ADC_MAX - adc))
    return 1 / (A_ + B_ * l + C_ * l ** 3) - 273.15
def c_to_adc(t):
    lo, hi = 1, ADC_MAX - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if adc_to_c(mid) > t: lo = mid
        else: hi = mid
    return hi
ROOM = c_to_adc(25.0)
_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
ADCBUF = int(re.search(r'^([0-9a-f]+) \S adc_buffer$', _nm, re.M).group(1), 16)

lines = []
def cmd(c): lines.append(c)
def mark(name): cmd('echo "@@MARK %s"' % name)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def adc(hotend, bed):
    for i in range(OVERSAMPLE):
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 0), hotend))
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 1), bed))
REGS = {'GCONF': 0x00, 'GSTAT': 0x01, 'IHOLD_IRUN': 0x10, 'TPOWERDOWN': 0x11,
        'TPWMTHRS': 0x13, 'CHOPCONF': 0x6C, 'PWMCONF': 0x70}
def regs(tag, addrs=(0, 1, 2, 3)):
    for a in addrs:
        for name, r in REGS.items():
            cmd('echo "@@REG %s %d %s"' % (tag, a, name)); cmd('sysbus.tmc Reg %d 0x%02X' % (a, r))

for _m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs', 'TeacupTMC2209.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', _m))
cmd('mach create "p3"')
cmd('machine LoadPlatformDescription @%s' % REPL)
cmd('machine LoadPlatformDescriptionFromString "tmc: UART.TeacupTMC2209 @ sysbus <0x50100000, +0x100>"')
cmd('emulation CreateUARTHub "tmchub"')
cmd('connector Connect sysbus.usart6 tmchub')
cmd('connector Connect sysbus.tmc tmchub')
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
cmd('sysbus.gpioPortA OnGPIO 0 true')    # filament present
for port, n in (('B', 10), ('B', 3), ('B', 15)):
    cmd('sysbus.gpioPort%s OnGPIO %d true' % (port, n))
run('0.02'); adc(ROOM, ROOM); run('0.4')
regs('boot', (0,))
mark('m80'); send('M80\n'); run('1.8')
regs('on')
mark('m906'); send('M906 X1000\nM906\n'); run('0.2')
regs('m906', (0,))
mark('m569'); send('M569 S0 X\nM569\n'); run('0.2')
regs('m569', (0,))
cmd('sysbus.tmc SetDrvFlags 0 0x1')
mark('m913'); send('M913 X50\nM913\n'); run('0.2')
regs('m913', (0,))
mark('m122'); send('M122\n'); run('0.3')
cmd('sysbus.tmc SetDrvFlags 0 0x0')
cmd('sysbus.tmc PowerCycle 1')
regs('y_lost', (1,))
mark('y_reset'); run('1.2')
regs('y_back', (1,))
cmd('sysbus.tmc SetPresent 3 false')
mark('e_gone'); run('1.2')
mark('e_m122'); send('M122\n'); run('0.3')
cmd('sysbus.tmc PowerCycle 3'); cmd('sysbus.tmc SetPresent 3 true')
mark('e_back'); run('1.2')
regs('e_back', (3,))
mark('m81'); send('M81\n'); run('0.3')
for a in range(4):
    cmd('sysbus.tmc PowerCycle %d' % a)
mark('move'); send('G92 X0\nG1 X1 F600\n'); run('0.7')           # 500 ms PS_ON delay
regs('move', (0, 3))
run('0.3')
mark('rep'); send('M503\n'); run('0.3')
mark('st'); send('M500\nM502\nM906\nM569\n'); run('0.3')
regs('m502', (0,))
mark('st2'); send('M501\nM906\n'); run('0.3')
regs('m501', (0,))
mark('end')
cmd('quit')

script = '/tmp/teacup_tmc_%d.resc' % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.run([RENODE, '--console', '--disable-gui', script], capture_output=True, text=True)
out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout + proc.stderr)
open('/tmp/teacup_tmc.log', 'w').write(out)
os.remove(script); os.remove(REPL)

sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def uart(sec):
    r = [re.sub(r'^.*\] ', '', l) for l in sections.get(sec, []) if 'usart2: [host' in l]
    return [l.replace('echo:', '').strip() for l in r if not re.match(r'^ok( N-?\d+)? P\d+ B\d+$', l)]
R = {}
for m in re.finditer(r'@@REG (\S+) (\d) (\S+)\s*\n(?:.*\n)*?\s*(0x[0-9A-Fa-f]+)\s*\n', out):
    R[(m.group(1), int(m.group(2)), m.group(3))] = int(m.group(4), 16)
reg = lambda tag, a, name: R.get((tag, a, name))

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-50s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1

RS = 0.11
def cs_vsense(ma):                      # Marlin's current calculation
    cs = 32 * 1.41421356 * ma / 1000 * (RS + 0.02) / 0.325 - 1
    vs = 0
    if cs < 16:
        vs = 1
        cs = 32 * 1.41421356 * ma / 1000 * (RS + 0.02) / 0.180 - 1
    return max(0, min(31, int(cs))), vs
def ihold_irun(ma):
    cs, vs = cs_vsense(ma)
    return int(cs * 0.5) | cs << 8 | 10 << 16
def chopconf(ma):
    return 3 | 2 << 7 | 1 << 15 | cs_vsense(ma)[1] << 17 | 3 << 24 | 1 << 28
GC = 0x1C0                              # pdn_disable, mstep_reg_select, multistep_filt
cur = {0: 800, 1: 800, 2: 800, 3: 600}

check('before M80: X driver untouched (GCONF reset value)', reg('boot', 0, 'GCONF') == 1, reg('boot', 0, 'GCONF'))
ok = all(reg('on', a, 'GCONF') == (GC | (4 if a == 3 else 0)) and reg('on', a, 'IHOLD_IRUN') == ihold_irun(cur[a]) and
         reg('on', a, 'CHOPCONF') == chopconf(cur[a]) and reg('on', a, 'PWMCONF') == 0xC80D0E24 and
         reg('on', a, 'TPOWERDOWN') == 128 and reg('on', a, 'GSTAT') == 0 for a in range(4))
check('M80: all four configured (800 / 600 mA, 1/32, modes)', ok,
      ['%d: %s' % (a, ' '.join('%s=%X' % (n, reg('on', a, n) or 0) for n in REGS)) for a in range(4)]
      + ['expect IHOLD_IRUN %X / %X CHOPCONF %X' % (ihold_irun(800), ihold_irun(600), chopconf(800))])
check('M906 X1000: IRUN without vsense, reported',
      reg('m906', 0, 'IHOLD_IRUN') == ihold_irun(1000) and reg('m906', 0, 'CHOPCONF') == chopconf(1000) and
      'M906 X1000 Y800 Z800 E600' in uart('m906'),
      (hex(reg('m906', 0, 'IHOLD_IRUN') or 0), hex(ihold_irun(1000)), uart('m906')))
check('M569 S0 X: spreadCycle on X, report',
      reg('m569', 0, 'GCONF') == GC | 4 and uart('m569')[-2:] == ['M569 S0 X E', 'M569 S1 Y Z'],
      (reg('m569', 0, 'GCONF'), uart('m569')))
def thrs(mm_s, spm):                    # Marlin _tmc_thrs(), 1/32 microsteps
    return int(12650000 * 32 / (256 * mm_s * spm))
check('TPWMTHRS: hybrid X100 Y100 Z3 E30 mm/s',
      [reg('on', a, 'TPWMTHRS') for a in range(4)] == [thrs(100, 160), thrs(100, 160), thrs(3, 8000), thrs(30, 1672)],
      ([reg('on', a, 'TPWMTHRS') for a in range(4)], [thrs(100, 160), thrs(100, 160), thrs(3, 8000), thrs(30, 1672)]))
check('M913 X50: TPWMTHRS, report', reg('m913', 0, 'TPWMTHRS') == thrs(50, 160) and
      'M913 X50 Y100 Z3 E30' in uart('m913'), (reg('m913', 0, 'TPWMTHRS'), uart('m913')))
m122 = uart('m122')
check('M122: X TMC2209 ok 994 mA 1/32 spreadCycle, warning',
      any(l.startswith('TMC X (address 0): TMC2209, ok, 994 mA (IRUN 17, IHOLD 8), 1/32, spreadCycle') for l in m122) and
      any(l.startswith('CS_ACTUAL 17, standstill, overtemperature warning') for l in m122) and
      sum(1 for l in m122 if ', ok, ' in l) == 4 and
      any(l.startswith('TMC Y') and l.endswith('stealthChop up to 100 mm/s') for l in m122), m122)
check('driver Y lost power: reset seen, configured again',
      reg('y_lost', 1, 'GCONF') == 1 and 'TMC Y reset, configured' in uart('y_reset') and
      reg('y_back', 1, 'IHOLD_IRUN') == ihold_irun(800) and reg('y_back', 1, 'GCONF') == GC,
      (uart('y_reset'), reg('y_back', 1, 'IHOLD_IRUN')))
check('driver E missing: "not responding", M122 too',
      'TMC E not responding' in uart('e_gone') and 'TMC E (address 3): not responding' in uart('e_m122'),
      (uart('e_gone'), [l for l in uart('e_m122') if 'TMC E' in l]))
check('driver E back: found, configured',
      'TMC E found, configured' in uart('e_back') and reg('e_back', 3, 'IHOLD_IRUN') == ihold_irun(600),
      (uart('e_back'), reg('e_back', 3, 'IHOLD_IRUN')))
check('M81, drivers reset: first move configures them',
      reg('move', 0, 'IHOLD_IRUN') == ihold_irun(1000) and reg('move', 0, 'GCONF') == GC | 4 and
      reg('move', 3, 'IHOLD_IRUN') == ihold_irun(600),
      (reg('move', 0, 'IHOLD_IRUN'), reg('move', 3, 'IHOLD_IRUN')))
rep = uart('rep')
check('M503: M906 / M569', 'M906 X1000 Y800 Z800 E600' in rep and 'M569 S0 X E' in rep and 'M569 S1 Y Z' in rep,
      [l for l in rep if l.startswith('M906') or l.startswith('M569')])
check('M502: defaults, written to the drivers',
      'M906 X800 Y800 Z800 E600' in uart('st') and 'M569 S1 X Y Z' in uart('st') and
      reg('m502', 0, 'IHOLD_IRUN') == ihold_irun(800) and reg('m502', 0, 'GCONF') == GC,
      (uart('st'), reg('m502', 0, 'IHOLD_IRUN')))
check('M501: stored values back', 'M906 X1000 Y800 Z800 E600' in uart('st2') and
      reg('m501', 0, 'IHOLD_IRUN') == ihold_irun(1000), (uart('st2'), reg('m501', 0, 'IHOLD_IRUN')))
stream = [re.sub(r'^.*\] ', '', l) for l in out.splitlines() if 'usart2: [host' in l]
check('no error, no write error', not any(l.startswith('Error') or 'write error' in l for l in stream),
      [l for l in stream if 'rror' in l])
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
