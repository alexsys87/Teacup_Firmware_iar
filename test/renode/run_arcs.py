#!/usr/bin/env python3
"""Renode test of G2/G3 arcs on the P3 Steel build (make CHIP=F401 TEST=0).

The path is reconstructed from the X/Y step pulses (TIM1 / TIM2 one pulse
starts, STEP_TIMER_PULSES) and direction pins (PA9 / PA10), 160 steps/mm, and compared with the exact circle.

Usage: run_arcs.py [renode] [teacup.elf] [platform.repl]

Checks:
  - G2 / G3 with I J: half circle, full circle (start = end)
  - G2 with R (half circle), G3 with negative R (the 270 degree arc)
  - helix: Z and E along a full circle, end values exact
  - relative mode G91 with an arc
  - every step position within tolerance + step size of the circle, the
    right direction of rotation and the right sweep angle
  - M114 after each arc = the programmed end point
  - bad parameters: message, no move
  - M410 during a long arc (P10 full turns): stops, nothing queued after
  - small fast arcs (r 0.5 mm, 100 mm/s): segments come fast enough
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')

STEPS = 160.0                             # X, Y steps per mm
# ARC_TOLERANCE (chord sagitta, inwards) + 2 steps: segment end points are
# rounded to whole steps, and within a segment each axis steps on its own,
# a step position can be off the ideal line by almost one step per axis.
TOL = 0.010 + 2.0 / STEPS

_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
ADCBUF = int(re.search(r'^([0-9a-f]+) \S adc_buffer$', _nm, re.M).group(1), 16)
ROOM_ADC = 3911                           # 25 C: 100k NTC, 4.7k pull-up

lines = []
def cmd(c): lines.append(c)
def mark(name): cmd('echo "@@MARK %s"' % name)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))

for m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', m))
cmd('mach create "arcs"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
# Filament runout input PA0: pull-up on the board = filament present.
cmd('sysbus.gpioPortA OnGPIO 0 true')
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
for _p in ('gpioPortA', 'timer1', 'timer2', 'timer3', 'timer5'):
    cmd('logLevel -1 sysbus.%s' % _p)
def trace_log(on):
    # X/Y/E steps are timer one pulse starts (STEP_TIMER_PULSES), X/Y
    # directions are PA9 / PA10. TIM5 (step timing) separates the step
    # interrupts: X and Y stepped in one interrupt are one point.
    for _p in ('gpioPortA', 'timer1', 'timer2', 'timer3', 'timer5'):
        cmd('sysbus LogPeripheralAccess sysbus.%s %s' % (_p, 'true' if on else 'false'))
for n in (10, 3, 15):                    # endstops X, Y, Z
    cmd('sysbus.gpioPortB OnGPIO %d true' % n)
mark('boot')
run('0.02')
for i in range(32):
    cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * i, ROOM_ADC))
run('0.4')
send('M80\nG92 X100 Y100 Z5 E0\nM82\n'); run('0.8')
trace_log(True)

# name, G-code, start (x, y), center, end, direction (+1 CCW, -1 CW), sweep deg
ARCS = []
def arc(name, gcode, start, center, end, direction, sweep, wait=2.0):
    ARCS.append((name, start, center, end, direction, sweep))
    mark(name); send(gcode + '\nM400\nM114\n'); run(str(wait))

arc('a_g2_half', 'G2 X110 Y100 I5 J0 F3000', (100, 100), (105, 100), (110, 100), -1, 180)
arc('a_g3_full', 'G3 X110 Y100 I-5 J0', (110, 100), (105, 100), (110, 100), +1, 360, 3.0)
arc('a_g2_r', 'G2 X100 Y100 R5', (110, 100), (105, 100), (100, 100), -1, 180)
arc('a_g3_rneg', 'G3 X105 Y95 R-5', (100, 100), (100, 95), (105, 95), +1, 270, 2.5)
arc('a_helix', 'G2 X105 Y95 I-5 J0 Z6 E3', (105, 95), (100, 95), (105, 95), -1, 360, 4.0)
send('G91\n'); run('0.05')
arc('a_rel', 'G3 X-10 Y0 I-5 J0', (105, 95), (100, 95), (95, 95), +1, 180)
send('G90\n'); run('0.05')

mark('a_bad'); send('G2 X120 Y120\nM400\nM114\n'); run('0.2')

# M410 during a long arc: 10 full turns, r 20 mm, 20 s at 60 mm/s.
mark('a_long'); send('G92 X100 Y100\nG2 X100 Y100 I20 J0 P9 F3600\n'); run('1.5')
mark('a_stop'); send('M410\n'); run('0.3')
mark('a_after_stop'); send('M114\n'); run('1.0')
mark('a_quiet'); run('1.0')

# Many small fast arcs: r 0.5 mm full circles at 100 mm/s.
trace_log(False)
mark('a_fast'); send('G92 X100 Y100\n')
send(''.join('G2 X100 Y100 I0.5 J0 F6000\n' for _ in range(10)) + 'M400\nM114\n')
run('3.0')

# Like PrusaSlicer's arc fitting: fractional I/J, end point rounded to
# 3 decimals (not exactly on the circle), relative E (M83).
_c = (100 - 2.345, 100 + 1.5)
_r = math.hypot(2.345, 1.5)
_th = math.atan2(100 - _c[1], 100 - _c[0]) + math.radians(100)
_end = (round(_c[0] + _r * math.cos(_th), 3), round(_c[1] + _r * math.sin(_th), 3))
trace_log(True)
send('M83\n'); run('0.05')
arc('a_slicer', 'G3 X%.3f Y%.3f I-2.345 J1.5 E0.5 F1800' % _end, (100, 100), _c, _end, +1, 100)
mark('end')
cmd('quit')

script = '/tmp/arcs_test_%d.resc' % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script], stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout.read()); proc.wait()
open('/tmp/arcs_test.log', 'w').write(out)

sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def uart(sec):
    return [re.sub(r'^.*\] ', '', l) for l in sections.get(sec, []) if 'usart2: [host' in l]
def pos(sec):
    for l in uart(sec):
        m = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+) E:(-?[\d.]+)', l)
        if m: return tuple(float(v) for v in m.groups())

STEP_RE = re.compile(r'(timer[123]): .*WriteUInt32 to 0x0 .*value 0x9\b')
DIR_RE = re.compile(r'gpioPortA: .*WriteUInt32 to 0x18 \(BitSet\), value 0x([0-9A-F]+)')
def trace(sec, start):
    """Step positions (mm): TIM1 / TIM2 pulse starts, PA9 / PA10 direction.
    The direction pins are only written when they change, so their state
    is followed through the whole log, not just this section. Steps of one
    step interrupt (between TIM5 accesses) make one point: X and Y pulses
    start within a few cycles of each other."""
    x, y = start
    dx = dy = 1
    pts = [(x, y)]
    pending = False
    cur = 'pre'
    for l in out.splitlines():
        m = re.search(r'@+MARK (\S+)', l)
        if m and 'echo' not in l:
            cur = m.group(1)
            continue
        m = DIR_RE.search(l)
        if m:
            v = int(m.group(1), 16)
            if v & (1 << 9): dx = 1
            if v & (1 << (9 + 16)): dx = -1
            if v & (1 << 10): dy = 1
            if v & (1 << (10 + 16)): dy = -1
            continue
        if cur != sec:
            continue
        if 'timer5:' in l:
            if pending:
                pts.append((x, y))
                pending = False
            continue
        m = STEP_RE.search(l)
        if m:
            if m.group(1) == 'timer1': x += dx / STEPS
            elif m.group(1) == 'timer2': y += dy / STEPS
            else: continue
            pending = True
    if pending:
        pts.append((x, y))
    return pts

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-52s %s' % ('PASS' if cond else 'FAIL', name, info if not cond else ''))
    if not cond: fails += 1

u = uart('boot')
check('boot', u[:1] == ['start'], u[:3])

for name, start, center, end, direction, sweep in ARCS:
    pts = trace(name, start)
    r = math.hypot(start[0] - center[0], start[1] - center[1])
    dev = max(abs(math.hypot(px - center[0], py - center[1]) - r) for px, py in pts)
    # Unwrapped angle along the path.
    total, prev = 0.0, math.atan2(pts[0][1] - center[1], pts[0][0] - center[0])
    back = 0
    for px, py in pts[1:]:
        a = math.atan2(py - center[1], px - center[0])
        d = (a - prev + math.pi) % (2 * math.pi) - math.pi
        if d * direction < -1e-9: back += 1
        total += d; prev = a
    p = pos(name)
    print('--- %s: %d steps, max deviation %.4f mm, sweep %.1f deg' % (name, len(pts), dev, math.degrees(total)))
    check('%s: on the circle (<= %.3f mm)' % (name, TOL), dev <= TOL, '%.4f' % dev)
    check('%s: sweep %d deg, never backwards' % (name, direction * sweep),
          abs(math.degrees(total) - direction * sweep) < 1.0 and back == 0,
          (round(math.degrees(total), 2), back))
    check('%s: M114 = end point' % name, p is not None and abs(p[0] - end[0]) < 1e-6 and abs(p[1] - end[1]) < 1e-6,
          (p, end))
    check('%s: step count ends at the end point' % name,
          abs(pts[-1][0] - end[0]) < 0.5 / STEPS + 1e-9 and abs(pts[-1][1] - end[1]) < 0.5 / STEPS + 1e-9,
          (pts[-1], end))

p = pos('a_helix')
check('helix: Z6 E3 exact', p is not None and p[2] == 6.0 and p[3] == 3.0, p)
ne = 0
for l in sections.get('a_slicer', []):
    if re.search(r'timer3: .*WriteUInt32 to 0x0 .*value 0x9\b', l): ne += 1
check('slicer-like arc: relative E 0.5 mm = 836 steps (TIM3)', ne == 836, ne)
b = uart('a_bad')
check('bad parameters: message, no move', 'echo:G2/G3 bad parameters' in b and pos('a_bad')[:2] == (95.0, 95.0)
      and trace('a_bad', (95, 95)) == [(95, 95)], b)

pa = pos('a_after_stop')
steps_quiet = len(trace('a_quiet', (0, 0))) - 1
check('M410 during a 10-turn arc: position on the circle', pa is not None and
      abs(math.hypot(pa[0] - 120, pa[1] - 100) - 20) < 0.05, pa)
check('M410: no moves afterwards', steps_quiet == 0, steps_quiet)

def uart_deltas(sec):
    r = []
    for l in sections.get(sec, []):
        m = re.search(r'usart2: \[host.*virt:[^(]*\(\+([\d.]+)(ks|s|ms|µs)\)\] (.*)$', l)
        if m:
            r.append((float(m.group(1)) * {'ks': 1000, 's': 1, 'ms': 1e-3, 'µs': 1e-6}[m.group(2)], m.group(3)))
    return r
d = uart_deltas('a_fast')
# From the ok of G92 to the M114 answer after M400.
fast_time = None
for k in range(1, len(d)):
    if d[k][1].startswith('X:'):
        fast_time = sum(t for t, _ in d[1:k + 1])
        break
print('--- 10 arcs r 0.5 mm: %s s' % (None if fast_time is None else round(fast_time, 3)))
pf = pos('a_fast')
okf = [l for l in uart('a_fast') if l.startswith('ok')]
check('10 small fast arcs: done, back at start', pf is not None and pf[:2] == (100.0, 100.0), (pf, len(okf)))
check('10 small fast arcs: < 2.5 s (planner keeps up)', fast_time is not None and fast_time < 2.5, fast_time)

print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
