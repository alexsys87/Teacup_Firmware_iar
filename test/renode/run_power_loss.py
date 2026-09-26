#!/usr/bin/env python3
"""Renode test of the power loss recovery (POWER_LOSS_RECOVERY, M413,
M1000) on the P3 Steel build with a W25Q64 (models/TeacupW25Q.cs) and the
mechanics model (models/p3_model.py):

  make CHIP=F401 TEST=0

A 6 layer file is uploaded to the SPI flash (M28 / M29) and printed (M23,
M24). In the middle the printer restarts (M112, M999: RAM is lost, the
SPI flash and the mechanics keep their state). After the restart:

  - the firmware reports the interrupted print with the Z of the move
    that was executing (not of the queue end, many moves are read ahead)
  - M1000 heats up, lifts Z, homes X and Y, primes, goes back down and
    prints the rest of the file: the physical end position is right, the
    filament fed covers the whole file (nothing missing) plus the prime
    plus at most the layer since the last record
  - a finished print leaves no record behind

Usage: run_power_loss.py [renode] [teacup.elf] [platform.repl]
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, 'models')
sys.path.insert(0, MODELS)
import p3_model

RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma_flash.repl')

OVERSAMPLE, SLOTS, ADC_MAX = 16, 2, 4095
RP, PTS = 4700.0, [(25.0, 100000.0), (150.0, 1641.9), (250.0, 226.15)]
def sh_coef():
    (t1, r1), (t2, r2), (t3, r3) = PTS
    l1, l2, l3 = map(math.log, (r1, r2, r3))
    y1, y2, y3 = (1 / (t + 273.15) for t in (t1, t2, t3))
    g2, g3 = (y2 - y1) / (l2 - l1), (y3 - y1) / (l3 - l1)
    c = (g3 - g2) / (l3 - l2) / (l1 + l2 + l3)
    b = g2 - c * (l1 * l1 + l1 * l2 + l2 * l2)
    a = y1 - (b + l1 * l1 * c) * l1
    return a, b, c
A, B, C = sh_coef()
def adc_to_c(adc):
    r = RP * adc / (ADC_MAX - adc)
    l = math.log(r)
    return 1 / (A + B * l + C * l ** 3) - 273.15
def c_to_adc(t):
    lo, hi = 1, ADC_MAX - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if adc_to_c(mid) > t: lo = mid
        else: hi = mid
    return hi

_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
ADCBUF = int(re.search(r'^([0-9a-f]+) \S adc_buffer$', _nm, re.M).group(1), 16)

lines = []
def cmd(c): lines.append(c)
TAGS = []
def mark(name):
    """Start a section. Renode's logger (UART output) is asynchronous and
    falls behind the monitor's echo with the Python hooks, so sections
    can't be cut by time. Instead "M73 P<n>" + "M73" puts a tag
    ("echo:Progress: n%") into the UART stream itself: a section is the
    output between two tags, in command order."""
    TAGS.append(name)
    cmd('echo "@@MARK %s"' % name)
    send('M73 P%d R0\nM73\n' % len(TAGS))
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def pin(port, n, level): cmd('sysbus.gpioPort%s OnGPIO %d %s' % (port, n, 'true' if level else 'false'))
def adc(hotend, bed):
    for i in range(OVERSAMPLE):
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 0), hotend))
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 1), bed))
def model(call): cmd('python "import p3_model; p3_model.%s"' % call)
def report(tag): model("report('%s')" % tag)
def hook(addr, fn):
    cmd('sysbus AddWatchpointHook 0x%08X 4 Write "import p3_model; p3_model.%s(self.Machine, value)"' % (addr, fn))

for _m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupW25Q.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(MODELS, _m))
cmd('mach create "p3"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
cmd('python "import sys; sys.path.append(\'%s\'); import p3_model"' % MODELS)
# Start state of the inputs, matching the model: switches open (high),
# probe signal low, filament present (pull-up).
pin('B', 10, True); pin('B', 3, True); pin('B', 15, False); pin('A', 0, True)
model("pins.update({('B', 10): True, ('B', 3): True, ('B', 15): False})")
hook(0x40010000, 'step_x')        # TIM1 CR1
hook(0x40000000, 'step_y')        # TIM2 CR1
hook(0x40000400, 'step_e')        # TIM3 CR1
hook(0x40020418, 'gpiob_bsrr')    # Z step (PB12)
hook(0x40014034, 'servo_ccr')     # TIM9 CCR1: servo pulse width
hook(0x40020818, 'gpioc_bsrr')    # PC13 servo signal
ROOM = c_to_adc(25.0)
HOT, BED = c_to_adc(200.0), c_to_adc(60.0)
def cs(t):
    c = 0
    for ch in t: c ^= ord(ch)
    return '%s*%d' % (t, c)
def boot(name):
    mark(name); run('0.02'); adc(HOT, BED); run('0.4')
def reset(name):
    send('M112\n'); run('0.1'); mark(name); send('M999\n'); run('0.02'); adc(HOT, BED)
    # The reset clears the GPIO inputs: switches open again (the nozzle is
    # away from them), probe low, filament present.
    pin('B', 10, True); pin('B', 3, True); pin('B', 15, False); pin('A', 0, True)
    model("pins.update({('B', 10): True, ('B', 3): True, ('B', 15): False})")
    run('0.6')

boot('boot')
send('M80\n'); run('0.7')
SPM = 'M92 X40 Y40 Z400 E100\nM500\n'
mark('setup'); send(SPM); run('1.0')

LAYERS, LH = 6, 0.2
FILE = ['M83', 'M104 S200', 'M140 S60', 'G28 X Y', 'G92 Z0 E0']
for k in range(LAYERS):
    FILE.append('G1 Z%.1f F600' % (LH * (k + 1)))
    FILE += ['G1 X30 Y10 E1 F1200', 'G1 X30 Y30 E1', 'G1 X10 Y30 E1', 'G1 X10 Y10 E1']
E_FILE = 4.0 * LAYERS
mark('upload'); send(cs('N-1 M110') + '\n'); run('0.02')
n = 0
send(cs('N%d M28 PLR.GCO' % n) + '\n'); run('0.1'); n += 1
for l in FILE:
    send(cs('N%d %s' % (n, l)) + '\n'); run('0.01'); n += 1
send(cs('N%d M29' % n) + '\n'); run('0.2')
model("pos.update({'x': 50.0, 'y': 60.0, 'z': 8.0, 'z2': 8.0, 'e': 0.0})")
mark('print'); send('M23 PLR.GCO\nM24\n'); run('11.0'); report('cut')
reset('restart')
mark('m413'); send('M413\n'); run('0.1')
mark('resume'); send('M1000\n'); run('60.0'); report('end')
mark('m114'); send('M114\n'); run('0.1')
reset('restart2')
mark('m413b'); send('M413\n'); run('0.1')
mark('discard'); send('M1000 C\n'); run('0.1')
cmd('quit')

script = '/tmp/teacup_plr_%d.resc' % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script],
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, text=True)
out = proc.stdout.read()
proc.wait()
out = re.sub(r'\x1b\[[0-9;]*m', '', out)
open('/tmp/teacup_plr.log', 'w').write(out)

STREAM = [re.sub(r'^.*\] ', '', l) for l in out.splitlines() if 'usart2: [host' in l]
TAGRE = re.compile(r'^echo:Progress: (\d+)%, remaining 0 min$')
sections, cur = {'pre': []}, 'pre'
for l in STREAM:
    mm = TAGRE.match(l)
    if mm and 1 <= int(mm.group(1)) <= len(TAGS):
        cur = TAGS[int(mm.group(1)) - 1]; sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def uart(sec):
    return ['ok' if re.match(r'^ok( N-?\d+)? P\d+ B\d+$', l) else l for l in sections.get(sec, [])]
def allu():
    return STREAM
MODEL = {}
for l in out.splitlines():
    mm = re.search(r'@@MODEL (\S+) (.*)', l)
    if mm:
        d = {}
        for kv in mm.group(2).split():
            k, v = kv.split('=', 1)
            try: d[k] = float(v)
            except ValueError: d[k] = v
        MODEL[mm.group(1)] = d
def pos(sec):
    for l in uart(sec):
        mm = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+) E:(-?[\d.]+) Count', l)
        if mm: return tuple(float(v) for v in mm.groups())
    return None

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-46s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1
def near(a, b, tol):
    return a is not None and b is not None and abs(a - b) <= tol

Z0 = 13.0                                # physical Z of G92 Z0 (8 + 5 mm
                                         # lift of G28, Z_HOMING_HEIGHT)
m0, m1 = MODEL.get('cut', {}), MODEL.get('end', {})
# Boot messages come before the section tag (sent after the reset).
u = uart('print') + uart('restart')
rep = [l for l in u if l.startswith('echo:Power loss recovery: PLR.GCO')]
check('restart: interrupted print reported', len(rep) == 1, u)
zr = float(re.search(r'Z(-?[\d.]+)\.', rep[0]).group(1)) if rep else None
# The executing move is an XY move of the current layer: its line starts
# at the physical Z (not at the Z of the queue end, which is higher).
check('  Z of the executing line', zr is not None and m0 and abs(zr - (m0['z'] - Z0)) <= 0.001, (zr, m0.get('z')))
check('  printing was in the middle', m0 and 2.0 < m0.get('e', 0) < E_FILE - 2.0, m0.get('e'))
check('M413: on, record', any('Power-loss recovery ON' in l for l in uart('m413')), uart('m413'))
ur = uart('resume')
check('M1000: resumed, file finished', 'echo:Print resumed' in ur and 'SD file done.' in ur, ur)
check('  end position X10 Y10 Z%.1f' % (LH * LAYERS), m1 and abs(m1['x'] - 10) <= 0.2 and abs(m1['y'] - 10) <= 0.2
      and abs(m1['z'] - Z0 - LH * LAYERS) <= 0.003, (m1.get('x'), m1.get('y'), m1.get('z')))
p = pos('m114')
check('  M114 X10 Y10', p is not None and p[:2] == (10.0, 10.0), p)
e = m1.get('e', 0) - 3.0                 # minus the prime
# Resumed from the start of the layer (the last record): what was printed
# of it before the power loss is printed again, at most one layer.
check('  filament: whole file, at most one layer twice', E_FILE - 0.01 <= e <= E_FILE + 4.01, (e, E_FILE))
check('finished print: no record left', any('No interrupted print' in l for l in uart('m413b'))
      and not any('Power loss recovery:' in l for l in uart('m114') + uart('restart2')), uart('restart2') + uart('m413b'))
check('no error', not any(l.startswith('Error') for l in allu() if 'kill' not in l and 'M112' not in l and 'halted' not in l),
      [l for l in allu() if l.startswith('Error')])
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
