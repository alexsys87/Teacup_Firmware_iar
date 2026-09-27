#!/usr/bin/env python3
"""Renode test of the mesh subdivision (MESH_SUBDIVISIONS), G26 and the test
prints M9910 (linear advance pattern), M9911 (input shaping tower), M9912
(tuning tower) on the P3 Steel build (make CHIP=F401 TEST=0).

No homing: the test sets axes_homed in RAM (address from nm), positions
come from G92. E steps are TIM3 one pulse starts.

Usage: run_calibration.py [renode] [teacup.elf] [platform.repl]

Checks:
  - M421 mesh (3x3, a bump in the middle), M420 S1 Z0: the Z motor follows
    the Catmull-Rom subdivision (3 per cell) at several places, +-1 um,
    visibly different from the plain bilinear mesh
  - G26: starts, reports the circles, M410 stops it ("Test print
    stopped"), host E unchanged, leveling state restored
  - M9910 A0 B0.02 C0.01: three lines with K 0, 0.01, 0.02 reported, E
    extruded as computed, K restored, host E unchanged
  - M9911 A20 B40 H0.4: two layers, frequency reported, M593 restored
  - M9912 P1 A0 B0.01: G1 Z1 sets K 0.01 and reports it, C band, P0 off
  - without homing: refused
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
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

_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
def sym(name):
    return int(re.search(r'^([0-9a-f]+) \S %s$' % name, _nm, re.M).group(1), 16)
ADCBUF = sym('adc_buffer')
HOMED = sym('axes_homed')
SP_STEPS = sym('startpoint_steps')          # TARGET: axis[X], [Y], [Z], [E], F

lines = []
def cmd(c): lines.append(c)
def run(t): cmd('emulation RunFor "%s"' % t)
def mark(name): cmd('echo "@@MARK %s"' % name)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def adc(hotend, bed):
    for i in range(OVERSAMPLE):
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 0), hotend))
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 1), bed))

for m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', m))
cmd('mach create "cal"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('sysbus.gpioPortA OnGPIO 0 true')
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2'); cmd('logLevel -1 sysbus.timer3')
for n in (10, 3, 15):
    cmd('sysbus.gpioPortB OnGPIO %d true' % n)
run('0.02'); adc(c_to_adc(25), c_to_adc(25)); run('0.4')
send('M80\nG92 X100 Y100 Z5 E0\nM82\n'); run('0.8')

# 1. Not homed: refused.
mark('nohome'); send('M9910\n'); run('0.1')

# 2. Mesh subdivision.
MESH = [[0.1, 0, 0], [0, 0.3, 0], [0, 0, 0]]          # [y][x] mm
send('M420 Z0\n')
for j in range(3):
    for i in range(3):
        send('M421 I%d J%d Z%.3f\n' % (i, j, MESH[j][i]))
send('M420 S1\n'); run('0.3')
PTS_XY = [(40.0, 40.0), (62.5, 90.0), (110.0, 52.5), (150.0, 140.0), (46.667, 15.0), (205.0, 165.0), (5.0, 5.0)]
for k, (x, y) in enumerate(PTS_XY):
    mark('sub%d' % k); send('G1 X%.3f Y%.3f Z5 F6000\nM400\nM114\n' % (x, y)); run('3.0')
    cmd('echo "@@ZSTEPS %d"' % k); cmd('sysbus ReadDoubleWord 0x%08X' % (SP_STEPS + 8))
send('M420 S0\n'); run('0.2')

# Homed from here on, hotend hot.
cmd('sysbus WriteByte 0x%08X 7' % HOMED)
# Target first: a jump to 200 C with the heater off is "heater off but
# temperature rising" for the thermal protection.
send('M104 S230\n'); run('0.1'); adc(c_to_adc(200), c_to_adc(25)); run('0.5')
send('M104 S200\n'); run('0.3')
send('G92 X100 Y100 Z5 E0\n'); run('0.1')
# 3. G26, stopped with M410 during the first circle.
mark('g26'); send('G26 H200 B0 P1\n'); run('8.0')
mark('g26_stop'); send('M410\n'); run('1.0')
cmd('sysbus LogPeripheralAccess sysbus.timer3 true')
mark('g26_after'); send('M114\nM420\n'); run('0.3')

# 4. M9910: 3 lines.
# G26 switched the heaters off at the end (no K).
send('G92 X100 Y100 Z5 E0\nM900 K0.05\nM104 S200\n'); run('0.5')
mark('la'); send('M9910 A0 B0.02 C0.01\n'); run('25.0')    # prime 3 s + 3 lines of ~4 s
mark('la_after'); send('M114\nM900\n'); run('0.3')

# 5. M9911: two layers.
send('G92 X100 Y100 Z5 E0\nM593 F33\n'); run('0.2')
cmd('sysbus LogPeripheralAccess sysbus.timer3 false')
mark('is'); send('M9911 A20 B40 H0.4\n'); run('30.0')     # first layer 200 mm at 20 mm/s
mark('is_after'); send('M114\nM593\n'); run('0.3')

# 6. M9912 tuning tower.
send('G92 X100 Y100 Z0.2 E0\n'); run('0.1')
mark('tw'); send('M9912 P1 A0 B0.01\nG1 Z1 F600\nM400\nM900\n'); run('0.5')
mark('tw_band'); send('M9912 P1 A0 B0.01 C2\nG1 Z3.5\nM400\nM900\n'); run('0.8')
mark('tw_off'); send('M9912 P0\nG1 Z6\nM400\nM900\n'); run('1.0')
mark('end')
cmd('quit')

script = '/tmp/cal_test_%d.resc' % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.run([RENODE, '--console', '--disable-gui', script], capture_output=True, text=True)
out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout + proc.stderr)
open('/tmp/cal_test.log', 'w').write(out)
os.remove(script)

sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def uart(sec):
    r = [re.sub(r'^.*\] ', '', l) for l in sections.get(sec, []) if 'usart2: [host' in l]
    return [l.replace('echo:', '').strip() for l in r if not re.match(r'^ok( N-?\d+)? P\d+ B\d+$', l)]
def e_pulses(sec):
    return sum(1 for l in sections.get(sec, [])
               if re.search(r'timer3: .*WriteUInt32 to 0x0 .*value 0x9\b', l))
def m114(sec):
    for l in uart(sec):
        m = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+) E:(-?[\d.]+) Count X:(-?\d+) Y:(-?\d+) Z:(-?\d+)', l)
        if m: return [float(v) for v in m.groups()]

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-56s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1

check('not homed: refused', 'Home X, Y and Z first (G28)' in uart('nohome'), uart('nohome'))

# Firmware model: Catmull-Rom virtual grid, rounded to um, bilinear.
X0, Y0, DX, DY, SUB = 15000, 15000, 95000, 75000, 3
Z = [[v * 1000 for v in row] for row in MESH]
def mp(i, j):
    if i < 0: return 2 * mp(0, j) - mp(1, j)
    if i >= 3: return 2 * mp(2, j) - mp(1, j)
    if j < 0: return 2 * mp(i, 0) - mp(i, 1)
    if j >= 3: return 2 * mp(i, 2) - mp(i, 1)
    return Z[j][i]
def cr(p0, p1, p2, p3, t):
    return 0.5 * (2 * p1 + (p2 - p0) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
                  (3 * (p1 - p2) + p3 - p0) * t ** 3)
NV = 2 * SUB + 1
VZ = [[0] * NV for _ in range(NV)]
for vy in range(NV):
    iy, ty = vy // SUB, (vy % SUB) / SUB
    if iy > 1: iy, ty = 1, 1.0
    for vx in range(NV):
        ix, tx = vx // SUB, (vx % SUB) / SUB
        if ix > 1: ix, tx = 1, 1.0
        row = [cr(mp(ix - 1, iy - 1 + k), mp(ix, iy - 1 + k), mp(ix + 1, iy - 1 + k), mp(ix + 2, iy - 1 + k), tx)
               for k in range(4)]
        VZ[vy][vx] = math.floor(cr(*row, ty) + 0.5)
def bil(grid, n, vdx, vdy, x, y):
    fx = min(max((x - X0) / vdx, 0), n - 1)
    fy = min(max((y - Y0) / vdy, 0), n - 1)
    ix, iy = min(int(fx), n - 2), min(int(fy), n - 2)
    tx, ty = fx - ix, fy - iy
    z0 = grid[iy][ix] * (1 - tx) + grid[iy][ix + 1] * tx
    z1 = grid[iy + 1][ix] * (1 - tx) + grid[iy + 1][ix + 1] * tx
    return z0 * (1 - ty) + z1 * ty
ZS = {}
for m in re.finditer(r'@@ZSTEPS (\d+)\s*\n(?:.*\n)*?\s*(0x[0-9A-Fa-f]+)\s*\n', out):
    v = int(m.group(2), 16)
    ZS[int(m.group(1))] = v - (1 << 32) if v >= 1 << 31 else v
res, worst, diff_max = [], 0, 0
for k, (x, y) in enumerate(PTS_XY):
    p = m114('sub%d' % k)
    exp = math.floor(bil(VZ, NV, DX / SUB, DY / SUB, x * 1000, y * 1000) + 0.5)
    lin = bil(Z, 3, DX, DY, x * 1000, y * 1000)
    if p is None or k not in ZS:
        res.append(None); worst = 999; continue
    got = ZS[k] / 8.0 - p[2] * 1000           # motor um - logical um
    res.append((round(got, 1), exp, round(lin, 1)))
    worst = max(worst, abs(got - exp))
    diff_max = max(diff_max, abs(exp - lin))
check('subdivision: Z motor = Catmull-Rom surface (+-1 um)', worst <= 1.0, res)
check('  differs from the plain bilinear mesh (> 10 um somewhere)', diff_max > 10, diff_max)

g26 = uart('g26')
check('G26: circles reported', 'G26: 3x3 circles' in g26, g26)
p = m114('g26_after')
check('  M410: "Test print stopped", host E unchanged, leveling off again',
      'Test print stopped' in uart('g26_stop') + uart('g26_after') and p is not None and p[3] == 0.0 and
      'Bed Leveling OFF' in uart('g26_after'), (uart('g26_stop'), p, uart('g26_after')[:3]))

la = [l for l in uart('la') if l.startswith('LA pattern')]
check('M9910: lines K0 / 0.01 / 0.02, done',
      la == ['LA pattern line 1: K0.0000', 'LA pattern line 2: K0.0100', 'LA pattern line 3: K0.0200',
             'LA pattern done, lines from the front'], la)
e_line = 0.2 * 0.4 / (math.pi / 4 * 1.75 ** 2)            # mm filament per mm
# Prime 60 mm at double flow, 3 x 80 mm; 4 travels retract + prime 1 mm
# each, one final retract. Linear advance adds steps forth and back.
e_min = (60 * 2 * e_line + 3 * 80 * e_line + 4 * 2 + 1) * 1672
check('  E steps >= computed (%.0f)' % e_min, e_pulses('la') >= e_min - 5, e_pulses('la'))
p = m114('la_after')
check('  host E unchanged, K back to 0.05', p is not None and p[3] == 0.0 and
      any(l.startswith('M900 K0.05') for l in uart('la_after')), (p, uart('la_after')))

isr = [l for l in uart('is') if l.startswith('IS tower')]
check('M9911: frequency reported, done', 'IS tower Z0.200 F20.00' in isr and 'IS tower done' in isr, isr)
check('  M593 back to 33 Hz', any('F33.00' in l for l in uart('is_after')), uart('is_after'))

check('M9912 P1 A0 B0.01: G1 Z1 -> K0.01', 'Tuning tower Z1.000: 10/1000' in uart('tw') and
      any(l.startswith('M900 K0.01') for l in uart('tw')), uart('tw'))
check('  C2: band middle Z3.000 -> K0.03', 'Tuning tower Z3.000: 30/1000' in uart('tw_band') and
      any(l.startswith('M900 K0.03') for l in uart('tw_band')), uart('tw_band'))
check('  P0: off, K unchanged', 'Tuning tower off' in uart('tw_off') and
      any(l.startswith('M900 K0.03') for l in uart('tw_off')), uart('tw_off'))
stream = [re.sub(r'^.*\] ', '', l) for l in out.splitlines() if 'usart2: [host' in l]
check('no error', not any(l.startswith('Error') for l in stream), [l for l in stream if 'rror' in l])
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
