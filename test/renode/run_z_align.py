#!/usr/bin/env python3
"""Renode test of G34 (independent Z alignment, Z_STEPPER_ALIGN) on the P3
Steel configuration with BLTouch and Z2 STEP on PC15:

  make CHIP=F401 TEST=0 EXTRA="-DBLTOUCH -DZ_STEPPER_ALIGN" BUILD_SUFFIX=_g34

The mechanics model (models/p3_model.py) runs with two lead screws: Z
(PB12) at X = -35 and Z2 (PC15) at X = 255, DIR shared (PB13). The nozzle
height is on the line between them. Z2 starts 0.6 mm lower than Z.

Checks:
  - Z and Z2 step together on normal moves (G28)
  - G34 probes both points, raises the lower screw alone, repeats until
    the difference is within 0.02 mm, homes Z again
  - afterwards the nozzle has the same gap to the bed at both probe
    points, i.e. the gantry is parallel to the bed

Usage: run_z_align.py [renode] [teacup.elf] [platform.repl]
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, 'models')
sys.path.insert(0, MODELS)
import p3_model

RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0_g34/teacup.elf')
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
pin('B', 10, True); pin('B', 3, True); pin('B', 15, False); pin('A', 0, True)
model("pins.update({('B', 10): True, ('B', 3): True, ('B', 15): False})")
hook(0x40010000, 'step_x')        # TIM1 CR1
hook(0x40000000, 'step_y')        # TIM2 CR1
hook(0x40000400, 'step_e')        # TIM3 CR1
hook(0x40020418, 'gpiob_bsrr')    # Z step (PB12)
hook(0x40014034, 'servo_ccr')     # TIM9 CCR1: servo pulse width
hook(0x40020818, 'gpioc_bsrr')    # PC13 servo signal
ROOM = c_to_adc(25.0)
model("DUAL.update({'on': True})")
# Z starts at 8.0. Z2 lower: the gantry tilts against the tilt of the
# model's bed (0.002 * x), 0.73 mm between the probe points.
model("pos.update({'z2': 7.4})")
model("PROBE.update({'dx': 0.0, 'dy': 0.0})")
mark('boot')
run('0.02'); adc(ROOM, ROOM)
run('0.4')

SPM = 'M92 X40 Y40 Z400 E100\n'
mark('setup'); send(SPM + 'M851 X0 Y0 Z-1.5\n'); run('0.2')
mark('g28'); send('G28\n'); run('18.0'); report('g28')
mark('g34'); send('G34\n'); run('120.0'); report('g34')
mark('g34_pos'); send('M114\n'); run('0.1')
cmd('quit')

script = '/tmp/teacup_zalign_%d.resc' % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
# Output straight into the log file, it can be watched while running.
with open('/tmp/teacup_zalign.log', 'w') as logf:
    proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script],
                            stdin=subprocess.PIPE, stdout=logf,
                            stderr=subprocess.STDOUT, text=True)
    proc.wait()
out = re.sub(r'\x1b\[[0-9;]*m', '', open('/tmp/teacup_zalign.log').read())

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
S1, S2 = -35.0, 255.0
def gap(md, x, y=90.0):
    z = md['z1'] + (md['z2'] - md['z1']) * (x - S1) / (S2 - S1)
    return z - bed(x, y)

m0 = MODEL.get('g28', {})
check('G28: Z and Z2 step together', m0.get('zup', -1) == m0.get('zup2', -2) and m0.get('zdown', -1) == m0.get('zdown2', -2)
      and m0.get('zup', 0) > 0, (m0.get('zup'), m0.get('zup2'), m0.get('zdown'), m0.get('zdown2')))
d0 = gap(m0, 20.0) - gap(m0, 200.0) if m0 else None
check('  before G34: gantry tilted', d0 is not None and abs(d0) > 0.3, d0)
u = uart('g34')
it = [l for l in u if l.startswith('echo:G34 #')]
check('G34: aligned message', 'echo:G34: Z steppers aligned, run G29 again' in u, u)
check('  iterations reported, last within 0.02', len(it) >= 2 and abs(float(re.search(r'difference (-?[\d.]+)', it[-1]).group(1))) <= 0.02, it)
m1 = MODEL.get('g34', {})
d1 = gap(m1, 20.0) - gap(m1, 200.0) if m1 else None
check('  after G34: same gap at both probe points (+-0.02)', d1 is not None and abs(d1) <= 0.02, d1)
check('  Z2 raised alone', m1.get('zup2', 0) > m1.get('zup', 0),
      (m1.get('zup'), m1.get('zup2')))
p = pos('g34_pos')
check('  Z homed again: nozzle at clearance above the center', p is not None and abs(p[2] - 5.0) <= 0.01 and abs(m1.get('pgap', 0) - 5.0) <= 0.01,
      (p, m1.get('pgap')))
check('no error / reset', not any('rror' in l for l in allu()), [l for l in allu() if 'rror' in l])
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
