#!/usr/bin/env python3
"""Renode test: the feedrate must not depend on the direction of a move.

On the P3 Steel build (make CHIP=F401 TEST=0), 150 mm moves at F3000
(50 mm/s) in different directions in the XY plane, plus two with a small Z
part (3D distance). The duration of each move is taken from the virtual
time between the "ok" of the G1 and the "ok" of the following M400. With
a correct planner all durations are equal (same length, same speed, same
acceleration).

The old integer distance approximation of Teacup (approx_distance(), from
AVR times) was off by -3 % .. +4 % depending on the direction, so was the
speed.

Usage: run_speed.py [renode] [teacup.elf] [platform.repl]
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')

_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
ADCBUF = int(re.search(r'^([0-9a-f]+) \S adc_buffer$', _nm, re.M).group(1), 16)

lines = []
def cmd(c): lines.append(c)
def mark(name): cmd('echo "@@MARK %s"' % name)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))

for m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', m))
cmd('mach create "speed"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
# Filament runout input PA0: pull-up on the board = filament present.
cmd('sysbus.gpioPortA OnGPIO 0 true')
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
for n in (10, 3, 15):                    # endstops X, Y, Z
    cmd('sysbus.gpioPortB OnGPIO %d true' % n)
mark('boot')
run('0.02')
for i in range(32):
    cmd('sysbus WriteWord 0x%08X 3911' % (ADCBUF + 2 * i))       # 25 C
run('0.4')
send('M80\nG92 X10 Y10 Z5 E0\n'); run('0.8')

L, F, ACC = 150.0, 3000, 1000.0
MOVES = []
for deg in (0, 10, 20, 26.565, 30, 40, 45, 50, 60, 70, 80, 90):
    MOVES.append(('xy_%s' % str(deg).replace('.', '_'), deg, 0.0))
MOVES.append(('xyz_0', 0, 0.5))               # 3D: small Z part
MOVES.append(('xyz_45', 45, 0.5))
for name, deg, dz in MOVES:
    a = math.radians(deg)
    dx, dy = L * math.cos(a), L * math.sin(a)
    mark(name)
    send('G1 X%.3f Y%.3f Z%.3f F%d\nM400\n' % (10 + dx, 10 + dy, 5 + dz, F))
    run('3.6')
    mark(name + '_back')
    send('G1 X10 Y10 Z5 F9000\nM400\n')
    run('1.8')
mark('end')
cmd('quit')

script = '/tmp/speed_test_%d.resc' % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script], stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout.read()); proc.wait()
open('/tmp/speed_test.log', 'w').write(out)

sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
UNIT = {'ks': 1000.0, 's': 1.0, 'ms': 1e-3, 'µs': 1e-6}
def uart_timed(sec):
    """(absolute virtual time, text) of the UART lines of a section."""
    r = []
    for l in sections.get(sec, []):
        m = re.search(r'usart2: \[host.*virt: *([\d.]+)(ks|s|ms|µs) .*?\] (.*)$', l)
        if m:
            r.append((float(m.group(1)) * UNIT[m.group(2)], m.group(3)))
    return r

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-46s %s' % ('PASS' if cond else 'FAIL', name, info if not cond else ''))
    if not cond: fails += 1

dur = {}
for name, deg, dz in MOVES:
    u = uart_timed(name)
    # "ok" of the G1 (queued) and of M400 (move done). Keepalive lines
    # ("busy: processing") may come in between.
    oks = [t for t, txt in u if txt.startswith('ok')]
    dur[name] = oks[1] - oks[0] if len(oks) >= 2 else None

v = F / 60.0
print('expected about %.3f s (L/v + v/a); the log gives 10 ms resolution' % (L / v + v / ACC))
print('%-10s %9s %8s' % ('move', 'time s', 'speed %'))
ref = dur.get('xy_0')
for name, deg, dz in MOVES:
    t = dur[name]
    length = math.sqrt(L * L + dz * dz)
    rel = (ref / t * length / L * 100.0) if (t and ref) else None
    print('%-10s %9s %8s' % (name, '%.3f' % t if t else '-', '%.1f' % rel if rel else '-'))

times = [t for t in dur.values() if t]
check('all moves timed', len(times) == len(MOVES), dur)
if times:
    spread = (max(times) - min(times)) / min(times) * 100.0
    print('spread of the durations: %.2f %%' % spread)
    check('speed independent of direction (spread < 1 %)', spread < 1.0, '%.2f %%' % spread)

print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
