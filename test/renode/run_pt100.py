#!/usr/bin/env python3
"""Renode test of a PT100 hotend sensor through a MAX31865 (HOTEND_MAX31865)
on the P3 Steel configuration:

  make CHIP=F401 TEST=0 EXTRA="-DHOTEND_MAX31865" BUILD_SUFFIX=_pt

A MAX31865 model (models/TeacupMAX31865.cs) on SPI1, chip select PC15,
takes the place of the SPI flash (Renode's STM32 SPI holds one device).
The test sets its sensor resistance and reads the temperature (M105).

Checks:
  - configuration at startup: bias, automatic conversion, 50 Hz, 2 wire,
    fault clear
  - 25, 100, 200, 250 C (Callendar-Van Dusen) read within 0.25 C, below
    0 C reads 0
  - the sensor is read every ~100 ms
  - a fault (open sensor) while heating: no readings, the thermal
    protection halts ("Temperature sensor timeout")

Usage: run_pt100.py [renode] [teacup.elf] [platform.repl]
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0_pt/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')

OVERSAMPLE, SLOTS, ADC_MAX = 16, 1, 4095     # only the bed is analog now
_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
ADCBUF = int(re.search(r'^([0-9a-f]+) \S adc_buffer$', _nm, re.M).group(1), 16)

# Bed thermistor at 25 C (see run_p3steel.py).
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

# PT100, IEC 60751.
R0, A, B, C = 100.0, 3.9083e-3, -5.775e-7, -4.183e-12
def pt_r(t):
    return R0 * (1 + A * t + B * t * t + (C * (t - 100) * t ** 3 if t < 0 else 0))

lines = []
def cmd(c): lines.append(c)
def mark(name): cmd('echo "@@MARK %s"' % name)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def pin(port, n, level): cmd('sysbus.gpioPort%s OnGPIO %d %s' % (port, n, 'true' if level else 'false'))
def adc(bed):
    for i in range(OVERSAMPLE):
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 0), bed))
def prop(tag, name):
    cmd('echo "@@%s"' % tag); cmd('sysbus.spi1.max31865 %s' % name)

repl2 = '/tmp/teacup_pt100_%d.repl' % os.getpid()
open(repl2, 'w').write('max31865: SPI.TeacupMAX31865 @ spi1\n\ngpioPortC:\n    15 -> max31865@0\n')
for _m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs', 'TeacupMAX31865.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', _m))
cmd('mach create "p3"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('machine LoadPlatformDescription @%s' % repl2)
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('sysbus.gpioPortA OnGPIO 0 true')    # filament present
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
for port, n in (('B', 10), ('B', 3), ('B', 15)):
    pin(port, n, True)
cmd('sysbus.spi1.max31865 Resistance %.4f' % pt_r(25.0))
mark('boot')
run('0.02'); adc(ROOM)
run('0.5')
prop('CONFIG', 'Config')
TEMPS = [25.0, 100.0, 200.0, 250.0, -10.0]
for t in TEMPS:
    cmd('sysbus.spi1.max31865 Resistance %.4f' % pt_r(t)); run('0.3')
    mark('t_%d' % int(t)); send('M105\n'); run('0.05')
prop('READS0', 'Reads'); run('1.0'); prop('READS1', 'Reads')
# Heating at 200 C, then the sensor opens.
cmd('sysbus.spi1.max31865 Resistance %.4f' % pt_r(200.0)); run('0.3')
send('M104 S200\n'); run('1.0')
mark('fault'); cmd('sysbus.spi1.max31865 Fault true'); run('5.5')
mark('end')
cmd('quit')

script = '/tmp/teacup_pt100_%d.resc' % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script], stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout.read()); proc.wait()
open('/tmp/teacup_pt100.log', 'w').write(out)

sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def uart(sec):
    r = [re.sub(r'^.*\] ', '', l) for l in sections.get(sec, []) if 'usart2: [host' in l]
    return ['ok' if re.match(r'^ok( N-?\d+)? P\d+ B\d+$', l) else l for l in r]
v = {}
for m in re.finditer(r'@@(\w+)\s*\n(?:.*\n)*?\s*(0x[0-9A-Fa-f]+|\d+)\s*\n', out):
    v.setdefault(m.group(1), int(m.group(2), 0))

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-44s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1

u = uart('boot')
check('boot', bool(u) and u[0] == 'start', u)
check('config: bias, auto, 50 Hz, 2 wire, clear (0xC3)', v.get('CONFIG') == 0xC3, v.get('CONFIG'))
for t in TEMPS:
    got = None
    for l in uart('t_%d' % int(t)):
        m = re.match(r'ok T:(-?[\d.]+)/', l)
        if m: got = float(m.group(1))
    exp = max(t, 0.0)
    check('PT100 %6.1f C (R %.3f Ohm)' % (t, pt_r(t)), got is not None and abs(got - exp) <= 0.25, got)
n = v.get('READS1', 0) - v.get('READS0', 0)
check('read every ~100 ms (8..11 per s)', 8 <= n <= 11, n)
uf = uart('fault')
check('open sensor while heating: sensor timeout, halted',
      any('Temperature sensor timeout' in l for l in uf) and any('Printer halted' in l for l in uf), uf)
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
