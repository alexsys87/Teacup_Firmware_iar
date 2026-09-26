#!/usr/bin/env python3
"""Renode test of the look-ahead planner and the accelerations on the P3
Steel build (make CHIP=F401 TEST=0): many short moves must run at the
programmed feedrate, without slowing down at each junction; retracts use
M204 R, diagonals accelerate like axis-aligned moves.

With look-ahead over two moves only, each move had to be able to stop
within itself: 0.35 mm at 1000 mm/s^2 gave at most ~26 mm/s, arc segments
of 0.63 mm ~35 mm/s, whatever the feedrate.

A CPU hook on dda_start() logs the virtual time of each move start, so the
average speed of each move is known. A short Z move after M400 marks the
end of the last move.

Usage: run_lookahead.py [renode] [teacup.elf] [platform.repl]

Checks, for a G2 circle (r 5 mm, segments of ~0.63 mm) and a polygon of
0.35 mm G1 moves (sent like a host at 115200 baud would), both at 100 mm/s:
  - total time close to one acceleration, cruise, one deceleration
  - cruise speed reached, not exceeded, no speed dips between moves
  - acceleration between moves within the limit
  - M114 = end point
and for single moves:
  - a 3 mm retract (E only) accelerates with ACCELERATION_RETRACT
  - a 20 mm move at 45 degrees takes as long as a 20 mm move along X: the
    acceleration applies along the path, not to the fast axis
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')
# Seconds between two polygon lines, 25 characters at 115200 baud.
PACE = float(os.environ.get('LOOKAHEAD_PACE', '0.002'))

F = 6000                                  # mm/min
V = F / 60.0                              # mm/s
ACC = 1000.0                              # mm/s^2, ACCELERATION
ACC_RETRACT = 5000.0                      # mm/s^2, ACCELERATION_RETRACT
# Acceleration applies along the path. Margin for averaging over moves:
# the speed is updated once per millisecond only. With the acceleration on
# the fast axis (before), diagonals gave ~1440 mm/s^2 here.
ACC_MAX = ACC * 1.2

_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
def sym(name):
    return int(re.search(r'^([0-9a-f]+) \S %s$' % name, _nm, re.M).group(1), 16)
ADCBUF = sym('adc_buffer')
DDA_START = sym('dda_start')

lines = []
def cmd(c): lines.append(c)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))

for m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', m))
cmd('mach create "lookahead"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
# Filament runout input PA0: pull-up on the board = filament present.
cmd('sysbus.gpioPortA OnGPIO 0 true')
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
for n in (10, 3, 15):                    # endstops X, Y, Z
    cmd('sysbus.gpioPortB OnGPIO %d true' % n)
run('0.02')
for i in range(32):
    cmd('sysbus WriteWord 0x%08X 3911' % (ADCBUF + 2 * i))       # 25 C
run('0.4')
send('M80\nG92 X100 Y100 Z5 E0\n'); run('0.8')
cmd('sysbus.cpu AddHook 0x%08X "self.ErrorLog(\'LA %%.9f\' %% '
    'machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds)"' % DDA_START)

# Moves of a scenario, then a short Z move as end marker, queued right
# behind them, then a pause. Move starts are grouped by these pauses.
def finish(z):
    send('M400\nG1 Z%.2f F240\nM400\nM114\n' % z)
    run('1.2')

# 1. Full circle, r 5 mm: segments of sqrt(8 * r * ARC_TOLERANCE) = 0.63 mm.
R_ARC = 5.0
send('G2 X100 Y100 I%.1f J0 F%d\n' % (R_ARC, F))
finish(5.02)

# 2. Polygon of 0.35 mm moves, 6 degrees each (below the jerk limit at
#    100 mm/s), two turns, sent line by line.
SEG, N_POLY = 0.35, 120
r_poly = SEG / (2 * math.sin(math.pi / 60))
cx, cy = 100 - r_poly, 100
send('G1 F%d\n' % F)
for k in range(1, N_POLY + 1):
    a = 2 * math.pi * k / 60
    send('G1 X%.3f Y%.3f\n' % (cx + r_poly * math.cos(a), cy + r_poly * math.sin(a)))
    run(str(PACE))
finish(5.04)

# 3. Retract, E only, at the E feedrate limit (25 mm/s).
L_RETRACT, F_RETRACT = 3.0, 1500
send('G1 E-%.1f F%d\n' % (L_RETRACT, F_RETRACT))
finish(5.06)

# 4. 20 mm at 45 degrees, then 20 mm along X.
L_LINE = 20.0
D45 = L_LINE / math.sqrt(2)
send('G1 X%.3f Y%.3f F%d\n' % (100 + D45, 100 + D45, F))
finish(5.08)
send('G1 X%.3f F%d\n' % (100 + D45 - L_LINE, F))
finish(5.10)
cmd('quit')

script = '/tmp/lookahead_test_%d.resc' % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script], stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout.read()); proc.wait()
open('/tmp/lookahead_test.log', 'w').write(out)

starts = [float(v) for v in re.findall(r'cpu: LA ([\d.]+)', out)]
groups = []
for t in starts:
    if not groups or t - groups[-1][-1] > 0.3:
        groups.append([])
    groups[-1].append(t)
uart = [re.sub(r'^.*\] ', '', l) for l in out.splitlines() if 'usart2: [host' in l]
positions = []
for l in uart:
    m = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+) E:(-?[\d.]+)', l)
    if m:
        positions.append(tuple(float(v) for v in m.groups()))

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-52s %s' % ('PASS' if cond else 'FAIL', name, info if not cond else ''))
    if not cond: fails += 1

def ideal_time(length, v=V, acc=ACC):
    """One acceleration to v, cruise, one deceleration."""
    if length >= v * v / acc:
        return length / v + v / acc
    return 2 * math.sqrt(length / acc)

def analyse(name, times, seg_len, length, slack):
    """times: move starts plus the start of the end marker."""
    n = len(times) - 1
    dts = [times[i + 1] - times[i] for i in range(n)]
    speeds = [seg_len / dt for dt in dts]
    total = times[-1] - times[0]
    ideal = ideal_time(length)
    print('--- %s: %d moves of %.3f mm, %.3f s (ideal %.3f s), top %.1f mm/s'
          % (name, n, seg_len, total, ideal, max(speeds)))
    print('    mm/s: ' + ' '.join('%.0f' % v for v in speeds))
    check('%s: total time <= %.2f x ideal + %.2f s' % (name, slack[0], slack[1]),
          total <= ideal * slack[0] + slack[1], '%.3f s' % total)
    check('%s: cruise speed reached (>= 95 %%)' % name, max(speeds) >= 0.95 * V,
          '%.1f' % max(speeds))
    check('%s: feedrate not exceeded (<= 102 %%)' % name, max(speeds) <= 1.02 * V,
          '%.1f' % max(speeds))
    mid = speeds[n // 4: n - n // 4]
    check('%s: no speed dips in the middle half' % name,
          mid and min(mid) >= 0.95 * max(mid), '%.1f .. %.1f' % (min(mid), max(mid)))
    # Over spans of 4 moves: speeds are updated once per millisecond only
    # (dda_clock()), so single moves of 3..6 ms are noisy.
    acc = max(abs(speeds[i + 4] ** 2 - speeds[i] ** 2) / (2 * 4 * seg_len)
              for i in range(n - 4))
    check('%s: acceleration <= %.0f mm/s^2' % (name, ACC_MAX), acc <= ACC_MAX, '%.0f' % acc)

u = [l for l in uart if l.startswith('ok')]
check('move queue: 63 free slots (ADVANCED_OK)', any(l.startswith('ok P63') for l in u), u[:3])
check('five groups of moves', len(groups) == 5, [len(g) for g in groups])
if len(groups) == 5:
    n_arc = len(groups[0]) - 1
    seg_arc = 2 * R_ARC * math.sin(math.pi / n_arc)
    analyse('G2 circle r 5', groups[0], seg_arc, n_arc * seg_arc, (1.15, 0.02))
    check('G1 polygon: %d moves' % N_POLY, len(groups[1]) - 1 == N_POLY, len(groups[1]) - 1)
    # The first move starts right away and has to stop, the next ones
    # can't join it while it runs.
    analyse('G1 polygon 0.35 mm', groups[1], SEG, N_POLY * SEG, (1.15, 0.06))

    # Single moves: one start plus the end marker. Step timing adds ~1 %.
    t_ret = groups[2][-1] - groups[2][0]
    v_ret = F_RETRACT / 60.0
    ideal_r = ideal_time(L_RETRACT, v_ret, ACC_RETRACT)
    ideal_r_old = ideal_time(L_RETRACT, v_ret, ACC)
    print('--- retract %.1f mm at %.0f mm/s: %.4f s (%.4f s at %.0f mm/s^2, %.4f s at %.0f)'
          % (L_RETRACT, v_ret, t_ret, ideal_r, ACC_RETRACT, ideal_r_old, ACC))
    check('retract: single move', len(groups[2]) == 2, len(groups[2]))
    check('retract: accelerates with M204 R (%.0f mm/s^2)' % ACC_RETRACT,
          ideal_r * 0.99 <= t_ret <= ideal_r * 1.03 + 0.002, '%.4f s' % t_ret)

    t_diag = groups[3][-1] - groups[3][0]
    t_axis = groups[4][-1] - groups[4][0]
    ideal_l = ideal_time(L_LINE)
    print('--- 20 mm: diagonal %.4f s, along X %.4f s, ideal %.4f s '
          '(%.4f s with sqrt(2) on the diagonal)'
          % (t_diag, t_axis, ideal_l, ideal_time(L_LINE, V, ACC * math.sqrt(2))))
    check('along X: ideal time', ideal_l * 0.99 <= t_axis <= ideal_l * 1.03,
          '%.4f s' % t_axis)
    check('diagonal: same time as along X (+-1.5 %)',
          abs(t_diag - t_axis) <= 0.015 * t_axis, '%.4f s' % t_diag)
check('M114 after the circle', len(positions) >= 1 and
      max(abs(a - b) for a, b in zip(positions[0][:3], (100, 100, 5.02))) < 0.01,
      positions[:1])
check('M114 after the polygon', len(positions) >= 2 and
      max(abs(a - b) for a, b in zip(positions[1][:3], (100, 100, 5.04))) < 0.01,
      positions[1:2])
check('M114 after the retract', len(positions) >= 3 and
      abs(positions[2][3] + L_RETRACT) < 0.01, positions[2:3])
check('M114 after the lines', len(positions) >= 5 and
      max(abs(a - b) for a, b in zip(positions[4][:3],
          (100 + D45 - L_LINE, 100 + D45, 5.10))) < 0.01, positions[4:5])

print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
