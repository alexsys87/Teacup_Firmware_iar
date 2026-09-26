#!/usr/bin/env python3
"""Renode test of linear advance (M900 K) on the P3 Steel build
(make CHIP=F401 TEST=0).

A CPU hook on dda_start() logs the virtual time and the E positions of the
E generator: la_e_nominal (Bresenham) and la_e_actual (E motor). E step
pulses are timer one pulse starts of TIM3 (STEP_TIMER_PULSES), the E
direction is PB5 (inverted, E_INVERT_DIR).

Usage: run_linear_advance.py [renode] [teacup.elf] [platform.repl]

Checks, for a polygon of 60 G1 moves of 0.35 mm with E at 100 mm/s
(0.035 mm filament per mm):
  - K = 0: no advance, no backward E steps
  - K = 0.05: the extruder runs ahead by K * extrusion speed at cruise
    speed, takes it back while decelerating (backward E steps)
  - E pulses forward - backward = nominal E steps, E motor at its nominal
    position after the moves (M400 waits for it), M114 E right
and further:
  - retract and prime (E only) don't get advance
  - M900 K sets, M503 reports, M500 / M502 / M501 store and load it,
    out of range values are refused
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')

F = 6000                                  # mm/min
V = F / 60.0                              # mm/s
E_STEPS = 1672.0                          # E steps per mm
SEG, N_POLY = 0.35, 60
E_PER_MM = 0.035                          # filament per mm of path
K = 0.05

_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
def sym(name):
    return int(re.search(r'^([0-9a-f]+) \S %s$' % name, _nm, re.M).group(1), 16)
ADCBUF = sym('adc_buffer')
DDA_START = sym('dda_start')
E_NOM = sym('la_e_nominal')
E_ACT = sym('la_e_actual')

lines = []
def cmd(c): lines.append(c)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))

for m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', m))
cmd('mach create "la"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('sysbus.gpioPortA OnGPIO 0 true')    # filament present
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
for _p in ('gpioPortB', 'timer3'):
    cmd('logLevel -1 sysbus.%s' % _p)
for n in (10, 3, 15):                    # endstops X, Y, Z
    cmd('sysbus.gpioPortB OnGPIO %d true' % n)
run('0.02')
for i in range(32):
    cmd('sysbus WriteWord 0x%08X 3911' % (ADCBUF + 2 * i))       # 25 C
run('0.4')
send('M80\nG92 X100 Y100 Z5 E0\nM83\n'); run('0.8')
cmd('sysbus.cpu AddHook 0x%08X "self.ErrorLog(\'LA %%.9f %%d %%d\' %% '
    '(machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds, '
    'machine.SystemBus.ReadDoubleWord(0x%08X), '
    'machine.SystemBus.ReadDoubleWord(0x%08X)))"' % (DDA_START, E_NOM, E_ACT))
for _p in ('gpioPortB', 'timer3'):
    cmd('sysbus LogPeripheralAccess sysbus.%s true' % _p)

# Moves of a scenario, then a short Z move as end marker, queued right
# behind them, then a pause. Move starts are grouped by these pauses.
def finish(z):
    send('M400\nG1 Z%.2f F240\nM400\nM114\n' % z)
    run('1.2')

r_poly = SEG / (2 * math.sin(math.pi / 60))
cx, cy = 100 - r_poly, 100
def polygon():
    send('G1 F%d\n' % F)
    for k in range(1, N_POLY + 1):
        a = 2 * math.pi * k / 60
        send('G1 X%.3f Y%.3f E%.5f\n' % (cx + r_poly * math.cos(a),
             cy + r_poly * math.sin(a), SEG * E_PER_MM))
        run('0.002')

# 1. Without linear advance.
send('M900 K0\n'); run('0.02')
polygon()
finish(5.02)
# 2. With linear advance.
send('M900 K%.2f\n' % K); run('0.02')
polygon()
finish(5.04)
# 3. Retract and prime with linear advance on: no advance.
send('G1 E-2 F1500\nG1 E2 F1500\n')
finish(5.06)
cmd('sysbus LogPeripheralAccess sysbus.gpioPortB false')
cmd('sysbus LogPeripheralAccess sysbus.timer3 false')
# 4. Settings.
send('M503\n'); run('0.2')
send('M900 K11\nM900 K0.123\nM500\nM502\nM503\nM501\nM503\nM900 K0\nM500\n')
run('0.8')
cmd('quit')

script = '/tmp/la_test_%d.resc' % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script], stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout.read()); proc.wait()
open('/tmp/la_test.log', 'w').write(out)

def s32(v):
    v = int(v)
    return v - (1 << 32) if v >= (1 << 31) else v

# Walk the log in order: move starts (grouped by pauses) and E pulses.
HOOK_RE = re.compile(r'cpu: LA ([\d.]+) (\d+) (\d+)')
PULSE_RE = re.compile(r'timer3: .*WriteUInt32 to 0x0 .*value 0x9\b')
DIR_RE = re.compile(r'gpioPortB: .*WriteUInt32 to 0x18 \(BitSet\), value 0x([0-9A-F]+)')
groups = []                               # [events, fwd, back]
fwd_dir = None                            # E direction pin: forward?
for l in out.splitlines():
    m = HOOK_RE.search(l)
    if m:
        t, nom, act = float(m.group(1)), s32(m.group(2)), s32(m.group(3))
        if not groups or t - groups[-1][0][-1][0] > 0.3:
            groups.append([[], 0, 0])
        groups[-1][0].append((t, nom, act))
        continue
    m = DIR_RE.search(l)
    if m:
        v = int(m.group(1), 16)
        if v & (1 << 5): fwd_dir = False            # inverted: high = back
        if v & (1 << (5 + 16)): fwd_dir = True
        continue
    if PULSE_RE.search(l) and groups:
        if fwd_dir: groups[-1][1] += 1
        else: groups[-1][2] += 1

uart = [re.sub(r'^.*\] ', '', l) for l in out.splitlines() if 'usart2: [host' in l]
positions = []
for l in uart:
    m = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+) E:(-?[\d.]+)', l)
    if m:
        positions.append(tuple(float(v) for v in m.groups()))

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-58s %s' % ('PASS' if cond else 'FAIL', name, info if not cond else ''))
    if not cond: fails += 1

check('three groups of moves', len(groups) == 3, [len(g[0]) for g in groups])
expected_adv = K * V * E_PER_MM * E_STEPS
if len(groups) == 3:
    def scenario(name, g, k):
        ev, fwd, back = g
        moves = ev[:-1]                   # the last one is the Z marker
        nom0, act0 = moves[0][1], moves[0][2]
        nom1, act1 = ev[-1][1], ev[-1][2]
        adv = [a - n for _, n, a in moves]
        mid = adv[len(adv) // 4: len(adv) - len(adv) // 4]
        mean = sum(mid) / len(mid) if mid else 0
        print('--- %s: %d moves, %d E steps nominal, pulses +%d -%d, '
              'advance at cruise %.1f steps (min %d max %d)'
              % (name, len(moves), nom1 - nom0, fwd, back, mean,
                 min(mid) if mid else 0, max(mid) if mid else 0))
        check('%s: E pulses forward - backward = nominal' % name,
              fwd - back == nom1 - nom0, (fwd, back, nom1 - nom0))
        check('%s: E motor at its nominal position before and after' % name,
              act0 == nom0 and act1 == nom1, ((nom0, act0), (nom1, act1)))
        return mean, back, nom1 - nom0

    mean0, back0, n0 = scenario('K 0', groups[0], 0)
    check('K 0: no advance', abs(mean0) <= 1, mean0)
    check('K 0: no backward E steps', back0 == 0, back0)
    # E per move as parsed: whole um.
    e_um = round(float('%.5f' % (SEG * E_PER_MM)) * 1000)
    n_expected = N_POLY * e_um / 1000.0 * E_STEPS
    check('K 0: E steps of the polygon (%.0f)' % n_expected,
          abs(n0 - n_expected) <= 1, n0)

    mean1, back1, n1 = scenario('K %.2f' % K, groups[1], K)
    print('    expected advance K * v_e = %.1f steps' % expected_adv)
    check('K %.2f: advance at cruise = K * extrusion speed (+-8 %%)' % K,
          abs(mean1 - expected_adv) <= 0.08 * expected_adv, '%.1f' % mean1)
    check('K %.2f: advance taken back while decelerating' % K, back1 > 0, back1)
    check('K %.2f: same nominal E as without' % K, n1 == n0, (n1, n0))

    ev, fwd, back = groups[2]
    adv = [a - n for _, n, a in ev[:-1]]
    print('--- retract / prime: pulses +%d -%d, advance at move starts %s'
          % (fwd, back, adv))
    # At a move start the pulse of the last E step of the move before
    # may still be due (+-1).
    check('retract / prime: no advance', all(abs(a) <= 1 for a in adv), adv)
    check('retract / prime: 2 mm back, 2 mm forward',
          fwd == back and abs(back - 2 * E_STEPS) <= 1, (fwd, back))

check('M114 after all moves: X100 Y100 Z5.06', len(positions) >= 3 and
      max(abs(a - b) for a, b in zip(positions[2][:3], (100, 100, 5.06))) < 0.01,
      positions[2:3])

m900 = [l for l in uart if l.startswith('echo:  M900 K')]
print('--- M900 lines: %s' % m900)
check('M503 reports M900 K%.4f' % K, len(m900) >= 1 and m900[0].endswith('M900 K%.4f' % K), m900[:1])
check('M900 K11 refused', any('M900 K out of range' in l for l in uart))
check('M502: K back to the default (0)', len(m900) >= 2 and m900[1].endswith('M900 K0.0000'), m900[1:2])
check('M501: stored K 0.123 loaded', len(m900) >= 3 and m900[2].endswith('M900 K0.1230'), m900[2:3])

print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
