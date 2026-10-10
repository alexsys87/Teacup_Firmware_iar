#!/usr/bin/env python3
"""Renode test of an inductive Z probe (INDUCTIVE_PROBE instead of BLTOUCH)
on the P3 Steel configuration:

  make CHIP=F401 TEST=0 EXTRA="-DINDUCTIVE_PROBE" BUILD_SUFFIX=_ind

The mechanics model (models/p3_model.py) drives PB15 as an NPN NO sensor:
low while the sensing face is within its switching distance of the
(tilted, bumped) bed, high otherwise; a level, no servo.

Checks:
  - no servo pulses on PC13, M115 reports the probe
  - G28: X, Y by switches, Z with the sensor at the bed center, 2 samples
  - G30 and the 9 points of G29 = bed heights (+-8 um)
  - M401 with the sensor already near the bed: error, no probing

Usage: run_inductive.py [renode] [teacup.elf] [platform.repl]
"""
import math, os, re, subprocess, sys

from renode_common import artifact_path, monitor_command, run_renode

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, 'models')
sys.path.insert(0, MODELS)
import p3_model

RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0_ind/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')

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
def cmd(c): lines.append(monitor_command(c))
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

for _m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(MODELS, _m))
cmd('mach create "p3"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
cmd('python "import sys; sys.path.append(\'%s\'); import p3_model"' % MODELS)
# Start state of the inputs, matching the model: switches open (high),
# probe signal low, filament present (pull-up).
# Inductive sensor, NPN NO: high (pull-up) while away from the bed.
model("PROBE.update({'kind': 'inductive', 'active_high': False})")
pin('B', 10, True); pin('B', 3, True); pin('B', 15, True); pin('A', 0, True)
model("pins.update({('B', 10): True, ('B', 3): True, ('B', 15): True})")
hook(0x40010000, 'step_x')        # TIM1 CR1
hook(0x40000000, 'step_y')        # TIM2 CR1
hook(0x40000400, 'step_e')        # TIM3 CR1
hook(0x40020418, 'gpiob_bsrr')    # Z step (PB12)
hook(0x40014034, 'servo_ccr')     # TIM9 CCR1: servo pulse width
hook(0x40020818, 'gpioc_bsrr')    # PC13 servo signal
ROOM = c_to_adc(25.0)
mark('boot')
run('0.02'); adc(ROOM, ROOM)
run('0.4')

SPM = 'M92 X40 Y40 Z400 E100\n'
mark('caps'); send('M115\n' + SPM); run('0.2')
mark('m851'); send('M851 X-30 Y-10 Z-1.5\n'); run('0.1')
mark('g28'); send('G28\n'); run('18.0'); report('g28')
mark('g28_pos'); send('M114\n'); run('0.1')
mark('g30'); send('G30 X50 Y50\n'); run('11.0'); report('g30')
mark('g29'); send('G29\n'); run('85.0'); report('g29')
mark('mesh'); send('M503\n'); run('0.3')
# Sensor near the bed (nozzle 0.5 above it at the center): no probing.
send('M420 S0\nG1 X140 Y100 Z0.5 F3000\nM400\n'); run('3.0')
mark('m401_low'); send('M401\n'); run('0.2')
send('G1 Z10 F600\nM400\n'); run('3.0')
mark('m401_ok'); send('M401\nM402\n'); run('0.2'); report('end')
cmd('quit')

script = artifact_path('teacup_inductive_%d.resc') % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
out = run_renode(RENODE, script)
out = re.sub(r'\x1b\[[0-9;]*m', '', out)
open(artifact_path('teacup_inductive.log'), 'w').write(out)

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

bed = p3_model.bed
XYTOL = 0.2
PC = (110.0, 90.0)                       # probe position when homing Z
ZREF = bed(*PC)                          # logical Z 0 = nozzle on the bed there

u = uart('caps')
check('M115: Z_PROBE capability', 'Cap:Z_PROBE:1' in u, u[-6:])
m = MODEL.get('g28', {})
p = pos('g28_pos')
check('G28: ok, no error', 'ok' in uart('g28') and not any('rror' in l for l in uart('g28')), uart('g28'))
check('G28: nozzle at X140 Y100, Z at clearance 5', p is not None and p[0] == 140.0 and p[1] == 100.0 and near(p[2], 5.0, 0.001), p)
check('G28 Z: sensor at the bed center', near(m.get('x', 0) - 30, PC[0], XYTOL) and near(m.get('y', 0) - 10, PC[1], XYTOL), (m.get('x'), m.get('y')))
check('G28 Z: nozzle 5 mm above the bed there', near(m.get('pgap'), 5.0, 0.01), m.get('pgap'))
check('G28 Z: 2 samples', m.get('trig') == 2, m.get('trig'))
check('no servo: no pulses on PC13', MODEL.get('end', {}).get('pset', -1) == 0 and MODEL.get('end', {}).get('servo') == '',
      (MODEL.get('end', {}).get('pset'), MODEL.get('end', {}).get('servo')))

u = uart('g30')
g30 = None
for l in u:
    mm = re.match(r'Bed X: ([\d.]+) Y: ([\d.]+) Z: (-?[\d.]+)', l)
    if mm: g30 = float(mm.group(3))
exp30 = bed(50, 50) - ZREF
check('G30 X50 Y50: bed height', near(g30, exp30, 0.008), (g30, round(exp30, 4), u))

u = uart('g29')
mesh = {}
for l in uart('mesh'):
    mm = re.match(r'echo:  M421 I(\d) J(\d) Z(-?[\d.]+)', l)
    if mm: mesh[(int(mm.group(1)), int(mm.group(2)))] = float(mm.group(3))
errs = []
for (i, j), z in sorted(mesh.items()):
    x, y = 15 + 87.5 * i, 15 + 75.0 * j
    errs.append(round(z - (bed(x, y) - ZREF), 4))
check('G29: completes, leveling on', 'echo:Bed Leveling ON' in u and 'ok' in u and not any('rror' in l for l in u), u[:3])
check('G29: 18 samples', MODEL.get('g29', {}).get('trig') == 22, MODEL.get('g29', {}).get('trig'))
check('G29: 9 points = bed heights (+-8 um)', len(errs) == 9 and all(abs(e) <= 0.008 for e in errs), errs)
check('M401 near the bed: error', any(l.startswith('Error:Probe triggered before probing') for l in uart('m401_low')), uart('m401_low'))
check('M401 / M402 away from it: ok', not any('rror' in l for l in uart('m401_ok')) and uart('m401_ok').count('ok') >= 2, uart('m401_ok'))
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
