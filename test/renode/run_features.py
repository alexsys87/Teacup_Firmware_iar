#!/usr/bin/env python3
"""Renode test of the filament runout sensor / M600, babystepping (M290),
BLTouch (G28 Z, G30, G29) and mesh bed leveling (M420, M421) on the P3
Steel configuration with BLTOUCH:

  make CHIP=F401 TEST=0 EXTRA="-DBLTOUCH" BUILD_SUFFIX=_bl

A mechanics model (models/p3_model.py) runs in Renode's Python, fed by
watchpoint hooks on the step outputs: it tracks the physical nozzle
position, closes the X/Y endstops, answers the BLTouch servo commands and
triggers the probe signal when the probe tip reaches the bed. The bed is
tilted and has a bump that is bilinear in the mesh cells, so the leveled
nozzle keeps a constant distance to it only if moves are split at the grid
lines. Steps/mm are reduced with M92 (fewer hook calls).

Usage: run_features.py [renode] [teacup.elf] [platform.repl]
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, 'models')
sys.path.insert(0, MODELS)
import p3_model

RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0_bl/teacup.elf')
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
mark('boot')
run('0.02'); adc(ROOM, ROOM)
run('0.4')

SPM = 'M92 X40 Y40 Z400 E100\n'
mark('caps'); send('M115\nM119\n' + SPM); run('0.2')
mark('m851'); send('M851 X-30 Y-10 Z-1.5\n'); run('0.1')

# ---- G28: X, Y at the switches, Z with the probe at the bed center ----
mark('g28'); send('G28\n'); run('18.0')
mark('g28_pos'); send('M114\n'); run('0.1'); report('g28')

# ---- G30 at one point ----
mark('g30'); send('G30 X50 Y50\n'); run('11.0'); report('g30')
# ---- M423 X twist: G30 at the same point with the interpolated correction ----
mark('tw_set'); send('M423 X0 Z0.1\nM423 X1 Z0.02\nM423 X2 Z-0.06\nM423\n'); run('0.2')
mark('tw_g30'); send('G30 X50 Y50\n'); run('11.0'); report('tw_g30')
mark('tw_off'); send('M423 R\n'); run('0.1')

# ---- G29: 3x3 mesh ----
mark('g29'); send('G29\n'); run('85.0'); report('g29')
mark('g29_state'); send('M420\n'); run('0.2')

# ---- Leveled moves: first layer height 0.2 everywhere ----
mark('lv_start'); send('G1 X20 Y20 Z0.2 F3000\nM400\n'); run('7.0'); report('lv_start')
model('track_start(0.2)')
mark('lv_diag'); send('G1 X185 Y160 F3000\nM400\n'); run('6.5')
model('track_stop()'); report('lv_diag')
mark('lv_pos'); send('M114\n'); run('0.1')

# ---- Babystepping ----
mark('bs_up'); send('M290 Z0.1\nM400\nM114\n'); run('0.6'); report('bs_up')
model('track_start(0.3)')
mark('bs_move'); send('G1 X100 Y90 F3000\nM400\n'); run('4.0')
model('track_stop()'); report('bs_move')
# During a move: 0.05 down while travelling.
mark('bs_during'); send('G1 X30 Y40 F1200\n'); run('1.0')
send('M290 Z-0.05\n'); run('5.0'); report('bs_during')
mark('bs_back'); send('M290 Z-0.05\nM400\nM290\n'); run('0.6'); report('bs_back')

# ---- Fade height, M420 S0 / S1 ----
mark('fade'); send('G1 Z10 F600\nM400\n'); run('4.0'); report('fade')
send('G1 Z0.2\nM400\n'); run('3.5')
mark('m420_off'); send('M420 S0\nM114\n'); run('0.2'); report('m420_off')
mark('m420_on'); send('M420 S1\nM114\n'); run('0.2')

# ---- Settings ----
mark('st_save'); send('M500\n'); run('0.3')
mark('st_defaults'); send('M502\nM503\n'); run('0.4')
mark('st_load'); send('M501\nM503\n'); run('0.4')

# ---- Filament runout during a "print" ----
# Heat up like a real hotend: the heater runs at full power while the
# (forced) reading climbs, the protection sees a plausible curve.
mark('ro_heat'); send('M104 S200\n'); run('0.6')
for t in (50, 80, 110, 140, 170, 190, 200):
    adc(c_to_adc(float(t)), ROOM); run('0.4')
send('M82\nG92 E0\nG1 X40 Y90 Z0.2 F3000\nM400\n'); run('3.0')
report('ro_start')
pin('A', 0, False)                          # filament gone
run('0.2')
mark('ro_print'); send('G1 X60 E10 F1200\nG1 X80 E20\nG1 X100 E30\nG1 X120 E40\nM400\nM114\n')
run('16.0'); report('ro_parked')
mark('ro_wait'); run('3.0')
pin('A', 0, True)                           # new filament in
mark('ro_resume'); send('M108\n'); run('25.0'); report('ro_resumed')
mark('ro_pos'); send('M114\n'); run('0.1')

# ---- M600 by hand, continued by the host prompt answer ----
mark('m600'); send('M600 Z5 U5 L5\nM114\n'); run('6.0'); report('m600_parked')
mark('m600_go'); send('M876 S0\n'); run('8.0'); report('m600_done')

# ---- M412 S0: no pause, M119 shows the sensor ----
pin('A', 0, False); run('0.1')
mark('ro_off'); send('M412 S0\nM412\nM119\nG1 X60 E80 F1200\nM400\nM114\n'); run('6.0')
send('M412 S1\n'); pin('A', 0, True); run('0.1')
mark('end'); run('0.05')
cmd('python "import time; time.sleep(3)"')
cmd('quit')

script = '/tmp/teacup_features_%d.resc' % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script],
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, text=True)
out = proc.stdout.read()
proc.wait()
out = re.sub(r'\x1b\[[0-9;]*m', '', out)
open('/tmp/teacup_features.log', 'w').write(out)

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
GX, GY = (15.0, 102.5, 190.0), (15.0, 90.0, 165.0)
def chord_range(a, b, split=True, n=4000):
    """Min/max of (straight Z between split points) - bed along the path
    a -> b: what a leveled nozzle does. Splitting at the grid lines (like
    Marlin's bilinear leveling) leaves only the chord error inside a cell,
    which is not 0 on a diagonal (bilinear is quadratic along it)."""
    ts = [0.0, 1.0]
    if split:
        for i, g in ((0, GX), (1, GY)):
            for v in g:
                if (a[i] - v) * (b[i] - v) < 0:
                    ts.append((v - a[i]) / (b[i] - a[i]))
    ts.sort()
    pt = lambda t: (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
    lo, hi = 1e9, -1e9
    for k in range(n + 1):
        t = k / n
        j = max(i for i in range(len(ts) - 1) if ts[i] <= t) if t < 1 else len(ts) - 2
        t0, t1 = ts[j], ts[j + 1]
        z0, z1 = bed(*pt(t0)), bed(*pt(t1))
        d = z0 + (z1 - z0) * (t - t0) / (t1 - t0) - bed(*pt(t))
        lo, hi = min(lo, d), max(hi, d)
    return lo, hi
# X/Y homing stops a little after the switch opened (back off, debounce,
# deceleration): the physical position is up to ~0.15 mm off the logical.
XYTOL = 0.2
PC = (110.0, 90.0)                       # probe position when homing Z
ZREF = bed(*PC)                          # logical Z 0 = nozzle on the bed there

u = uart('caps')
check('M115 capabilities', all(('Cap:%s:1' % c) in u for c in
      ('Z_PROBE', 'LEVELING_DATA', 'BABYSTEPPING', 'FILAMENT_RUNOUT', 'PROMPT_SUPPORT')), u[-8:])
check('M119: probe open, filament present', any('z_min:open' in l and 'filament:open' in l for l in u), [l for l in u if 'min' in l])
check('M851 sets the offset', 'echo:Probe Offset X-30.000 Y-10.000 Z-1.500' in uart('m851'), uart('m851'))

m = MODEL.get('g28', {})
p = pos('g28_pos')
check('G28: ok, no error', 'ok' in uart('g28') and not any('rror' in l for l in uart('g28')), uart('g28'))
# Nozzle at the Z homing position: probe (offset X-30 Y-10) at the center.
check('G28: nozzle at X140 Y100, Z at clearance 5', p is not None and p[0] == 140.0 and p[1] == 100.0 and near(p[2], 5.0, 0.001), p)
check('G28 Z: probe at the bed center', near(m.get('x', 0) - 30, PC[0], XYTOL) and near(m.get('y', 0) - 10, PC[1], XYTOL), (m.get('x'), m.get('y')))
check('G28 Z: nozzle 5 mm above the bed there', near(m.get('pgap'), 5.0 + 0.0, 0.01), m.get('pgap'))
# 1500 us (init), stow, then deploy / deploy / stow, no alarm reset (2193).
check('G28 Z: 2 samples, deploy x2, stow, no reset', m.get('trig') == 2 and m.get('servo') == '1500,1472,647,647,1472', (m.get('trig'), m.get('servo')))
check('servo pulses on PC13 (TIM9)', m.get('pset', 0) > 100 and abs(m.get('pset', 0) - m.get('preset', 0)) <= 2, (m.get('pset'), m.get('preset')))

u = uart('g30')
g30 = None
for l in u:
    mm = re.match(r'Bed X: ([\d.]+) Y: ([\d.]+) Z: (-?[\d.]+)', l)
    if mm: g30 = float(mm.group(3))
exp30 = bed(50, 50) - ZREF
check('G30 X50 Y50: bed height', near(g30, exp30, 0.008), (g30, round(exp30, 4), u))
tw = [l.replace('echo:', '').strip() for l in uart('tw_set') if 'M423' in l]
check('M423: points at X15 / 110 / 205 reported',
      tw == ['M423 X0 Z0.100 ; at X15.000', 'M423 X1 Z0.020 ; at X110.000', 'M423 X2 Z-0.060 ; at X205.000'], tw)
g30t = None
for l in uart('tw_g30'):
    mm = re.match(r'Bed X: ([\d.]+) Y: ([\d.]+) Z: (-?[\d.]+)', l)
    if mm: g30t = float(mm.group(3))
twist50 = 0.1 + (50 - 15) / 95.0 * (0.02 - 0.1)
check('M423: G30 X50 corrected by %.4f' % twist50,
      g30 is not None and g30t is not None and near(g30t - g30, twist50, 0.003), (g30t, g30))

u = uart('g29')
mesh = {}
st = uart('st_load')
for l in st:
    mm = re.match(r'echo:  M421 I(\d) J(\d) Z(-?[\d.]+)', l)
    if mm: mesh[(int(mm.group(1)), int(mm.group(2)))] = float(mm.group(3))
errs = []
for (i, j), z in sorted(mesh.items()):
    x, y = 15 + 87.5 * i, 15 + 75.0 * j
    errs.append(round(z - (bed(x, y) - ZREF), 4))
check('G29: completes, leveling on', 'echo:Bed Leveling ON' in u and 'ok' in u and not any('rror' in l for l in u), u[:3])
check('G29: 18 samples, no alarm reset', MODEL.get('g29', {}).get('trig') == 22 and '2193' not in str(MODEL.get('g29', {}).get('servo')),
      (MODEL.get('g29', {}).get('trig'), MODEL.get('g29', {}).get('servo')))
check('G29: 9 points = bed heights (+-8 um)', len(errs) == 9 and all(abs(e) <= 0.008 for e in errs), errs)
check('M420 reports ON, fade 10', 'echo:Bed Leveling ON' in uart('g29_state') and 'echo:Fade Height 10.000' in uart('g29_state'), uart('g29_state')[:3])

m = MODEL.get('lv_start', {})
check('G1 Z0.2 at X20 Y20: 0.2 above the bed', near(m.get('gap'), 0.2, 0.01), m.get('gap'))
m = MODEL.get('lv_diag', {})
lo, hi = chord_range((20, 20), (185, 160))
nlo, nhi = chord_range((20, 20), (185, 160), split=False)
check('diagonal over the bump: split at grid lines', m.get('tn', 0) > 1000 and near(m.get('tmin'), 0.2 + lo, 0.006)
      and near(m.get('tmax'), 0.2 + hi, 0.006),
      (m.get('tmin'), m.get('tmax'), 'expected %.4f..%.4f, unsplit %.4f..%.4f' % (0.2 + lo, 0.2 + hi, 0.2 + nlo, 0.2 + nhi)))
check('  logical Z stays 0.2', pos('lv_pos') is not None and pos('lv_pos')[2] == 0.2, pos('lv_pos'))

m = MODEL.get('bs_up', {})
check('M290 Z0.1: nozzle up at once', near(m.get('gap'), 0.3, 0.01), m.get('gap'))
check('  M114 unchanged, offset reported', pos('bs_up') is not None and pos('bs_up')[2] == 0.2 and 'echo:Z offset 0.100' in uart('bs_up'),
      (pos('bs_up'), uart('bs_up')))
m = MODEL.get('bs_move', {})
lo, hi = chord_range((185, 160), (100, 90))
check('  next move keeps +0.1', near(m.get('tmin'), 0.3 + lo, 0.006) and near(m.get('tmax'), 0.3 + hi, 0.006),
      (m.get('tmin'), m.get('tmax'), 'expected %.4f..%.4f' % (0.3 + lo, 0.3 + hi)))
m = MODEL.get('bs_during', {})
check('M290 during a move: applied, move finished', near(m.get('gap'), 0.25, 0.01) and near(m.get('x'), 30, XYTOL), (m.get('gap'), m.get('x')))
m = MODEL.get('bs_back', {})
check('M290 back to offset 0', near(m.get('gap'), 0.2, 0.01) and 'echo:Z offset 0.000' in uart('bs_back'), (m.get('gap'), uart('bs_back')))

m = MODEL.get('fade', {})
check('Z10 (fade height): no correction', near(m.get('z', 0) - ZREF, 10.0, 0.005), round(m.get('z', 0) - ZREF, 4))
m0 = MODEL.get('m420_off', {})
p = pos('m420_off')
exp = 0.2 + bed(30, 40) - ZREF
check('M420 S0: motors stay, logical Z = motor Z', near(m0.get('gap'), 0.2, 0.01) and p is not None and near(p[2], exp, 0.01), (m0.get('gap'), p, round(exp, 3)))
p = pos('m420_on')
check('M420 S1: logical Z back to 0.2', p is not None and near(p[2], 0.2, 0.0015), p)

u = uart('st_save')
check('M500 stores', any('Settings Stored' in l for l in u), u)
u = uart('st_defaults')
check('M502: no mesh, leveling off, probe offset 0', 'echo:  M420 S0 Z10.00' in u and not any('M421' in l for l in u) and 'echo:  M851 X0.00 Y0.00 Z0.00' in u,
      [l for l in u if 'M42' in l or 'M851' in l])
u = uart('st_load')
check('M501: mesh, M420 S1, M851, M412 back', 'echo:  M420 S1 Z10.00' in u and len(mesh) == 9 and 'echo:  M851 X-30.00 Y-10.00 Z-1.50' in u
      and 'echo:  M412 S1 D25.00' in u, [l for l in u if 'M42' in l or 'M851' in l or 'M412' in l][:4])

e0 = MODEL.get('ro_start', {}).get('e', 0)
u = uart('ro_print')
check('runout: pause after the buffered moves', 'echo:Filament runout' in u and '//action:out_of_filament T0' in u
      and '//action:prompt_begin Insert filament' in u, [l for l in u if 'ction' in l or 'unout' in l])
m = MODEL.get('ro_parked', {})
check('  parked at X10 Y170, Z +20', near(m.get('x'), 10, XYTOL) and near(m.get('y'), 170, XYTOL)
      and near(m.get('z', 0) - ZREF, 20.2 + 0.0, 0.1), (m.get('x'), m.get('y'), round(m.get('z', 0) - ZREF, 3)))
check('  retract 2 + unload 50 mm', near(m.get('e', 0) - e0, 40 - 2 - 50, 0.05), round(m.get('e', 0) - e0, 3))
# ro_wait's tag waits in the command queue until the pause ends: the
# busy messages of the pause are in ro_print.
check('  busy: paused for user', 'echo:busy: paused for user' in uart('ro_print'), [l for l in uart('ro_print') if 'busy' in l][:3])
m = MODEL.get('ro_resumed', {})
p = pos('ro_pos')
check('M108: load, back, prime, E logical unchanged', p is not None and p[0] == 120.0 and p[1] == 90.0 and p[2] == 0.2 and p[3] == 40.0, p)
check('  nozzle back 0.2 above the bed', near(m.get('gap'), 0.2, 0.01) and near(m.get('x'), 120, XYTOL), (m.get('gap'), m.get('x')))
check('  filament: +40 - 12 (change)', near(m.get('e', 0) - e0, 40 - 12, 0.05), round(m.get('e', 0) - e0, 3))
u = uart('m600')
m = MODEL.get('m600_parked', {})
check('M600: prompt, parked', '//action:prompt_show' in u and near(m.get('x'), 10, XYTOL), (u[-4:], m.get('x')))
m = MODEL.get('m600_done', {})
check('M876 S0 continues', 'echo:Resuming' in uart('m600') + uart('m600_go') and near(m.get('x'), 120, XYTOL) and near(m.get('gap'), 0.2, 0.01)
      and near(m.get('e', 0) - e0, 40 - 12 - 2, 0.05), (uart('m600_go')[-3:], m.get('x'), m.get('gap'), round(m.get('e', 0) - e0, 3)))
u = uart('ro_off')
check('M412 S0: no pause', not any('unout' in l and 'OFF' not in l for l in u) and pos('ro_off') is not None and pos('ro_off')[3] == 80.0,
      [l for l in u if 'ilament' in l] + [pos('ro_off')])
check('M119: filament triggered', any('filament:triggered' in l for l in u), [l for l in u if 'min' in l])
alls = allu()
check('no reset, no kill', sum(1 for l in alls if l == 'start') == 1 and not any('halted' in l for l in alls), '')

print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
