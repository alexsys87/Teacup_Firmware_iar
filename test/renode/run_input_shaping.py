#!/usr/bin/env python3
"""Renode test of input shaping and S-curve smoothing (M593) on the P3
Steel build (make CHIP=F401 TEST=0).

CPU hooks log each X/Y step of the Bresenham algorithm (shaper_step(): axis,
direction, compare time of the step timer) and each step pulse of the
shaper (shaper_pulse(): axis, timer count, direction pins PA9 / PA10).

From the nominal steps and the settings the test computes the shaped
position, sum of A[k] * x(t - T[k]) averaged over the S-curve window, and
compares it with the motor position at every pulse: it must never be more
than one step away. Copies as in motion/input_shaping.c (Klipper formulas).

Usage: run_input_shaping.py [renode] [teacup.elf] [platform.repl]

Checks:
  - shaping off by default: X/Y steps don't go through the shaper
  - M593 F doesn't change the feedrate of the following moves
  - X ZV 40 Hz, Y MZV 30 Hz, then with S-curve 10 ms: a polygon of short
    moves and a zig-zag with reversals: motor position = shaped position
    (+-1 step) at every pulse, pulses forward - backward = nominal steps,
    M114 right, no history overflows
  - mean delay of the motor behind the nominal steps
  - M593 reports, M500 / M502 / M501 store and load, bad values refused
"""
import bisect, math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')

F_CPU = 84000000
F = 6000

_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
def sym(name):
    return int(re.search(r'^([0-9a-f]+) \S %s$' % name, _nm, re.M).group(1), 16)
ADCBUF = sym('adc_buffer')
SHAPER_STEP = sym('shaper_step')
SHAPER_PULSE = sym('shaper_pulse')
TIM5_CNT, TIM5_CCR1, GPIOA_ODR = 0x40000C24, 0x40000C34, 0x40020014

lines = []
def cmd(c): lines.append(c)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))

for m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', m))
# A fine time quantum: with the default one Renode's timer model now and
# then misses a compare set just ahead of the counter (the firmware has a
# safety net for it, hal/timer.c, but the step is up to 2 ms late then).
cmd('emulation SetGlobalQuantum "0.000001"')
cmd('mach create "is"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('sysbus.gpioPortA OnGPIO 0 true')    # filament present
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
for n in (10, 3, 15):                    # endstops X, Y, Z
    cmd('sysbus.gpioPortB OnGPIO %d true' % n)
run('0.02')
for i in range(32):
    cmd('sysbus WriteWord 0x%08X 3911' % (ADCBUF + 2 * i))       # 25 C
run('0.4')
send('M80\nG92 X100 Y100 Z5 E0\nG90\n'); run('0.8')
cmd('sysbus.cpu AddHook 0x%08X "self.ErrorLog(\'SS %%d %%d %%d\' %% '
    '(self.GetRegisterUnsafe(0).RawValue, self.GetRegisterUnsafe(1).RawValue, '
    'machine.SystemBus.ReadDoubleWord(0x%08X)))"' % (SHAPER_STEP, TIM5_CCR1))
cmd('sysbus.cpu AddHook 0x%08X "self.ErrorLog(\'SP %%d %%d %%d\' %% '
    '(self.GetRegisterUnsafe(0).RawValue, machine.SystemBus.ReadDoubleWord(0x%08X), '
    'machine.SystemBus.ReadDoubleWord(0x%08X)))"' % (SHAPER_PULSE, TIM5_CNT, GPIOA_ODR))

def pause():
    send('M400\n')
    run('1.0')

# Groups of moves, each ends with the shaper idle. Config per group:
# (freq, damping, type) for X and Y, S-curve ms.
GROUPS = []
send('M114\n'); run('0.05')
# 0. Shaping off: no steps through the shaper.
send('G1 X105 F%d\n' % F); pause()
send('G1 X100\n'); pause()

r_poly = 0.35 / (2 * math.sin(math.pi / 60))
def polygon():
    cx, cy = 100 - r_poly, 100
    for k in range(1, 61):
        a = 2 * math.pi * k / 60
        send('G1 X%.3f Y%.3f\n' % (cx + r_poly * math.cos(a), cy + r_poly * math.sin(a)))
        run('0.002')
def zigzag():
    for k in range(12):
        send('G1 X%.3f Y%.3f\n' % (101 if k % 2 == 0 else 100, 100 + 0.5 * (k + 1)))
        run('0.002')
    send('G1 X100 Y100\n')

# 1. Input shaping. The F of M593 is a frequency, the next move without F
#    must run at F6000 (10 mm: 0.2 s), not at 40 mm/min.
CFG_A = ([(4000, 100, 0), (3000, 100, 1)], 0)
send('M593 X F40 D0.1 T0\nM593 Y F30 D0.1 T1\n'); run('0.1')
send('G1 X110\n'); pause(); GROUPS.append(('X 10 mm', CFG_A))
send('G1 X100\n'); pause(); GROUPS.append(('X back', CFG_A))
polygon(); pause(); GROUPS.append(('polygon, shaping', CFG_A))
zigzag(); pause(); GROUPS.append(('zig-zag, shaping', CFG_A))
# 2. Plus S-curve.
CFG_B = ([(4000, 100, 0), (3000, 100, 1)], 10)
send('M593 S10\n'); run('0.1')
polygon(); pause(); GROUPS.append(('polygon, shaping + S-curve', CFG_B))
zigzag(); pause(); GROUPS.append(('zig-zag, shaping + S-curve', CFG_B))
send('G1 X110 Y105\n'); pause(); GROUPS.append(('diagonal, shaping + S-curve', CFG_B))
send('G1 X100 Y100\n'); pause(); GROUPS.append(('diagonal back', CFG_B))
send('M114\n'); run('0.05')
# 3. Settings.
send('M593\n'); run('0.1')
send('M593 T2\nM593 D1.5\nM500\nM502\nM593\nM501\nM593\n'); run('0.5')
send('M593 F0 S0\nM500\n'); run('0.3')
send('M9001\n'); run('0.1')
cmd('quit')

script = '/tmp/is_test_%d.resc' % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script], stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout.read()); proc.wait()
open('/tmp/is_test.log', 'w').write(out)

nominal = [[], []]                        # (tick, sign)
pulses = [[], []]                         # (tick, sign)
for l in out.splitlines():
    m = re.search(r'cpu: SS (\d+) (\d+) (\d+)', l)
    if m:
        a = int(m.group(1))
        nominal[a].append((int(m.group(3)) & ~1, 1 if int(m.group(2)) else -1))
        continue
    m = re.search(r'cpu: SP (\d+) (\d+) (\d+)', l)
    if m:
        a = int(m.group(1))
        odr = int(m.group(3))
        fwd = (odr >> (9 if a == 0 else 10)) & 1
        pulses[a].append((int(m.group(2)), 1 if fwd else -1))

uart = [re.sub(r'^.*\] ', '', l) for l in out.splitlines() if 'usart2: [host' in l]
positions = []
for l in uart:
    m = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+) E:(-?[\d.]+)', l)
    if m:
        positions.append(tuple(float(v) for v in m.groups()))

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-60s %s' % ('PASS' if cond else 'FAIL', name, info if not cond else ''))
    if not cond: fails += 1

def copies(freq_centi, damp_milli, typ, s_ms):
    """Weights and delays (ticks) as in make_config()."""
    f, z = freq_centi / 100.0, damp_milli / 1000.0
    if f > 0:
        df = math.sqrt(1 - z * z)
        td = 1 / (f * df)
        if typ == 0:
            k = math.exp(-z * math.pi / df)
            a, t = [1, k], [0, 0.5 * td]
        else:
            k = math.exp(-0.75 * z * math.pi / df)
            a1 = 1 - 1 / math.sqrt(2)
            a, t = [a1, (math.sqrt(2) - 1) * k, a1 * k * k], [0, 0.375 * td, 0.75 * td]
    elif s_ms:
        a, t = [1], [0]
    else:
        return [], 0
    s = sum(a)
    return [(ai / s, int(ti * F_CPU + 0.5)) for ai, ti in zip(a, t)], s_ms * 1000 * (F_CPU // 1000000)

class Shaped:
    """Shaped position of a list of nominal steps at any time."""
    def __init__(self, events, cps, win):
        self.t = [e[0] for e in events]
        self.cs = [0]                     # prefix sums of signs
        self.cst = [0]                    # prefix sums of sign * time
        for t, s in events:
            self.cs.append(self.cs[-1] + s)
            self.cst.append(self.cst[-1] + s * t)
        self.cps, self.win = cps, win
    def at(self, t):
        g = 0.0
        for w, d in self.cps:
            full = bisect.bisect_right(self.t, t - d - self.win)
            x = self.cs[full]
            if self.win:
                inside = bisect.bisect_right(self.t, t - d)
                n = self.cs[inside] - self.cs[full]
                st = self.cst[inside] - self.cst[full]
                x += (n * (t - d) - st) / self.win
            g += w * x
        return g

# Group the nominal steps by pauses (> 0.1 s = 8.4 M ticks, within a
# group they are a few ms apart at most).
def groups_of(ev):
    gs = []
    for e in ev:
        if not gs or e[0] - gs[-1][-1][0] > 8400000:
            gs.append([])
        gs[-1].append(e)
    return gs

names = [g[0] for g in GROUPS]
for a, axis in enumerate('XY'):
    gs = groups_of(nominal[a])
    # Y doesn't move in the first two X moves.
    expected = [n for n, (name, cfg) in enumerate(GROUPS)
                if not (axis == 'Y' and name.startswith('X'))]
    # Before M593 (shaping off) no steps go through the shaper.
    check('%s: %d groups of shaped steps, none before M593' % (axis, len(expected)),
          len(gs) == len(expected),
          (len(gs), len(expected)))
    if len(gs) != len(expected):
        continue
    pos = 0                               # motor position before the group
    for gi, n in enumerate(expected):
        name, (axcfg, s_ms) = GROUPS[n]
        cps, win = copies(*axcfg[a], s_ms)
        ev = gs[gi]
        t0 = ev[0][0]
        t1 = gs[gi + 1][0][0] if gi + 1 < len(gs) else 1 << 62
        pl = [p for p in pulses[a] if t0 <= p[0] < t1]
        sh = Shaped(ev, cps, win)
        act, worst, lags = 0, 0.0, []
        for t, s in pl:
            act += s
            dev = abs(sh.at(t) - act)
            worst = max(worst, dev)
        net = sum(s for _, s in ev)
        # Mean delay: the i-th forward motor step after the i-th nominal one.
        fn = [t for t, s in ev if s > 0]
        fp = [t for t, s in pl if s > 0]
        if fn and len(fn) == len(fp):
            lags = [(p - q) / F_CPU * 1000 for p, q in zip(fp, fn)]
        exp_lag = (sum(w * d for w, d in cps) + win / 2) / F_CPU * 1000
        print('--- %s %s: %d steps, pulses +%d -%d, worst %.2f steps from the '
              'shaped position, mean delay %s ms (expected %.2f)'
              % (axis, name, len(ev), sum(1 for p in pl if p[1] > 0),
                 sum(1 for p in pl if p[1] < 0), worst,
                 '%.2f' % (sum(lags) / len(lags)) if lags else '-', exp_lag))
        check('%s %s: motor = shaped position (+-1 step)' % (axis, name), worst <= 1.0,
              '%.2f' % worst)
        check('%s %s: pulses forward - backward = nominal' % (axis, name),
              sum(s for _, s in pl) == net, (sum(s for _, s in pl), net))
        if lags and name.startswith('X 10'):
            mean = sum(lags) / len(lags)
            check('%s %s: mean delay (+-3 %%)' % (axis, name),
                  abs(mean - exp_lag) <= 0.03 * exp_lag, '%.3f' % mean)

# The 10 mm move at F6000 (F of M593 isn't a feedrate): about 0.2 s.
gx = groups_of(nominal[0])
if gx:
    dur = (gx[0][-1][0] - gx[0][0][0]) / F_CPU
    print('--- X 10 mm after M593 F40: %.3f s' % dur)
    check('M593 F leaves the feedrate alone (10 mm at 100 mm/s)', 0.15 < dur < 0.25, dur)

check('M114 at the end: X100 Y100', len(positions) >= 2 and
      abs(positions[-1][0] - 100) < 0.01 and abs(positions[-1][1] - 100) < 0.01,
      positions[-1:])
m593 = [l for l in uart if l.startswith('echo:  M593')]
print('--- M593 lines: %s' % m593)
check('M593 report', m593[:3] == ['echo:  M593 X F40.00 D0.100 T0',
                                  'echo:  M593 Y F30.00 D0.100 T1',
                                  'echo:  M593 S10.00'], m593[:3])
check('bad T and D refused', any('M593 T is 0' in l for l in uart) and
      any('M593 D out of range' in l for l in uart))
check('M502: defaults (off)', m593[3:6] == ['echo:  M593 X F0.00 D0.100 T1',
                                           'echo:  M593 Y F0.00 D0.100 T1',
                                           'echo:  M593 S0.00'], m593[3:6])
check('M501: stored values back', m593[6:9] == m593[:3], m593[6:9])
check('no history overflows', not any('overflows' in l for l in m593), m593)
st = [l for l in uart if l.startswith('echo:Step IRQ')]
m = re.search(r'late (\d+)', st[-1]) if st else None
check('no late steps (M9001)', bool(m) and m.group(1) == '0', st[-1:])

print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
