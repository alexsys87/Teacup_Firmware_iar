#!/usr/bin/env python3
"""Renode test of junction deviation (M205 J), slowdown (M205 B) and
volumetric extrusion / speed limit (M200 D S L) on the P3 Steel build
(make CHIP=F401 TEST=0).

A CPU hook on dda_start() logs the virtual time of each move start, E
steps are TIM3 one pulse starts.

Usage: run_quality.py [renode] [teacup.elf] [platform.repl]

Checks:
  - J0.02: a circle r 5 mm at 100 mm/s runs at sqrt(a * r) = 70.7 mm/s
    (centripetal acceleration = M204), J0: at 100 mm/s
  - square of 20 mm sides: time as computed with the corner speed
    sqrt(a * J * s / (1 - s)), s = sin(45 deg) (J0.02: 7 mm/s), and with
    jerk 20 mm/s (J0)
  - B100000: a burst of short moves into an empty queue gets stretched,
    B0 not
  - M200 D1.75 L5: G1 X20 E1 runs at 5 mm^3/s (41.7 mm/s), travel and
    L0 at full speed; M200 D1.75: E in mm^3 (2.405 mm^3 = 1 mm filament),
    S0 back to mm
  - M852 J K: Z moves shift the X / Y motors by Z * factor, M114 as is
  - reports, M500 / M502 / M501
"""
import math, os, re, subprocess, sys

from renode_common import artifact_path, monitor_command, run_renode

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')

F = 6000
V = F / 60.0
ACC = 1000.0
E_SPM = 1672

_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
def sym(name):
    return int(re.search(r'^([0-9a-f]+) \S %s$' % name, _nm, re.M).group(1), 16)
ADCBUF = sym('adc_buffer')
DDA_START = sym('dda_start')

lines = []
def cmd(c): lines.append(monitor_command(c))
def run(t): cmd('emulation RunFor "%s"' % t)
def mark(name): cmd('echo "@@MARK %s"' % name)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))

for m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', m))
cmd('mach create "quality"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('sysbus.gpioPortA OnGPIO 0 true')
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
for _t in ('timer1', 'timer2', 'timer3'):
    cmd('logLevel -1 sysbus.%s' % _t)
for n in (10, 3, 15):
    cmd('sysbus.gpioPortB OnGPIO %d true' % n)
run('0.02')
for i in range(32):
    cmd('sysbus WriteWord 0x%08X 3911' % (ADCBUF + 2 * i))
run('0.4')
send('M80\nG92 X100 Y100 Z5 E0\nM83\n'); run('0.8')
cmd('sysbus.cpu AddHook 0x%08X "self.ErrorLog(\'LA %%.9f\' %% '
    'machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds)"' % DDA_START)
cmd('sysbus LogPeripheralAccess sysbus.timer3 true')

z = [5.0]
def finish(pause='1.2'):
    z[0] += 0.02
    send('M400\nG1 Z%.2f F240\nM400\nM114\n' % z[0])
    run(pause)

# 1. Circle r 5 with J0.02 and J0.
R_ARC = 5.0
mark('circle_jd'); send('M205 J0.02 B0\nG2 X100 Y100 I%.1f J0 F%d\n' % (R_ARC, F)); finish()
mark('circle_j0'); send('M205 J0\nG2 X100 Y100 I%.1f J0 F%d\n' % (R_ARC, F)); finish()

# 2. Square, 20 mm sides.
SQ = 20.0
def square():
    send('G1 X%.1f Y100 F%d\nG1 X%.1f Y%.1f\nG1 X100 Y%.1f\nG1 X100 Y100\n'
         % (100 + SQ, F, 100 + SQ, 100 + SQ, 100 + SQ))
mark('sq_jd'); send('M205 J0.02\n'); square(); finish('1.6')
mark('sq_j0'); send('M205 J0\n'); square(); finish('1.6')

# 3. Slowdown: six 1 mm moves at once into the empty queue.
def burst():
    send(''.join('G1 X%d F%d\n' % (101 + k, F) for k in range(6)))
mark('sd_off'); send('M205 J0.02 B0\n'); run('0.1'); burst(); finish()
send('G1 X100 F%d\n' % F); run('0.3')
mark('sd_on'); send('M205 B100000\n'); run('0.1'); burst(); finish('1.6')
send('M205 B0\nG1 X100 F%d\n' % F); run('0.3')

# 4. Volumetric speed limit.
mark('vl_on'); send('M200 D1.75 S0 L5\nG1 X120 E1 F%d\n' % F); finish('1.2')
mark('vl_travel'); send('G1 X100 F%d\n' % F); finish()
mark('vl_off'); send('M200 L0\nG1 X120 E1 F%d\n' % F); finish()

# 5. Volumetric E.
mark('ve_on'); send('M200 D1.75\nG1 E2.405 F300\n'); finish()
mark('ve_off'); send('M200 S0\nG1 E1 F300\n'); finish()
# 6. Skew XZ / YZ: Z moves shift the X and Y motors.
cmd('sysbus LogPeripheralAccess sysbus.timer1 true')
cmd('sysbus LogPeripheralAccess sysbus.timer2 true')
mark('sk_set'); send('M852 I0 J0.1 K0.1\nM852\n'); run('0.1')
mark('sk_up'); send('G1 Z%.2f F600\nM400\nM114\n' % (z[0] + 1)); run('1.0')
mark('sk_down'); send('G1 Z%.2f\nM400\nM114\n' % z[0]); run('1.0')
mark('sk_off'); send('M852 J0 K0\nG1 Z%.2f\nM400\n' % (z[0] + 1)); run('1.0')
cmd('sysbus LogPeripheralAccess sysbus.timer1 false')
cmd('sysbus LogPeripheralAccess sysbus.timer2 false')
mark('rep'); send('M200 D1.75 S1 L5\nM205 J0.05 B30000\nM200\nM503\n'); run('0.3')
mark('st'); send('M500\nM502\nM200\nM501\nM200\n'); run('0.3')
mark('end')
cmd('quit')

script = artifact_path('quality_test_%d.resc') % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
# stdin stays open: at EOF on stdin Renode's console queues input events
# without end and runs out of memory.
out = re.sub(r'\x1b\[[0-9;]*m', '', run_renode(RENODE, script))
open(artifact_path('quality_test.log'), 'w').write(out)
os.remove(script)

sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def starts(sec):
    return [float(v) for l in sections.get(sec, []) for v in re.findall(r'cpu: LA ([\d.]+)', l)]
def e_pulses(sec):
    return sum(1 for l in sections.get(sec, [])
               if re.search(r'timer3: .*WriteUInt32 to 0x0 .*value 0x9\b', l))
def uart(sec):
    r = [re.sub(r'^.*\] ', '', l) for l in sections.get(sec, []) if 'usart2: [host' in l]
    return [l.replace('echo:', '').strip() for l in r if not re.match(r'^ok( N-?\d+)? P\d+ B\d+$', l)]

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-52s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1

def seg_time(L, v0, v1, vmax, a=ACC):
    """Time of one move from v0 to v1, at most vmax, acceleration a."""
    vp = math.sqrt((2 * a * L + v0 * v0 + v1 * v1) / 2)   # peak if no cruise
    if vp <= vmax:
        return (vp - v0) / a + (vp - v1) / a
    d_acc = (vmax * vmax - v0 * v0) / (2 * a)
    d_dec = (vmax * vmax - v1 * v1) / (2 * a)
    return (vmax - v0) / a + (vmax - v1) / a + (L - d_acc - d_dec) / vmax
def path_time(Ls, vj, vmax):
    vs = [0.0] + [vj] * (len(Ls) - 1) + [0.0]
    return sum(seg_time(L, vs[i], vs[i + 1], vmax) for i, L in enumerate(Ls))

def circle_speed(sec):
    t = starts(sec)                          # moves + end marker
    n = len(t) - 1                           # moves, the last ends at the marker
    if n < 10:
        return None, None
    seg = 2 * R_ARC * math.sin(math.pi / n)
    sp = [seg / (t[i + 1] - t[i]) for i in range(n)]
    mid = sp[n // 4: n - n // 4]
    return max(sp), (min(mid), max(mid))
v_jd, mid_jd = circle_speed('circle_jd')
v_c = math.sqrt(ACC * R_ARC)
# Junctions at sqrt(a * r), in between each 0.63 mm segment accelerates a
# bit (half of it up, half down): v^2 + a * segment at the most.
v_c_max = math.sqrt(v_c ** 2 + ACC * 2 * R_ARC * math.sin(math.pi / 50))
check('J0.02: circle r5 at sqrt(a*r) = %.1f mm/s (<= %.1f)' % (v_c, v_c_max),
      v_jd is not None and 0.98 * v_c <= v_jd <= 1.01 * v_c_max and mid_jd[0] >= 0.98 * v_c,
      (v_jd, mid_jd))
v_j0, _ = circle_speed('circle_j0')
check('J0: circle at 100 mm/s (jerk)', v_j0 is not None and v_j0 >= 0.95 * V, v_j0)

def sq_time(sec):
    t = starts(sec)
    return t[4] - t[0] if len(t) >= 5 else None
s = math.sin(math.radians(45))
vj_jd = math.sqrt(ACC * 0.02 * s / (1 - s))
exp_jd = path_time([SQ] * 4, vj_jd, V)
exp_j0 = path_time([SQ] * 4, 20.0, V)
t_jd, t_j0 = sq_time('sq_jd'), sq_time('sq_j0')
check('square J0.02: corners at %.1f mm/s (%.3f s)' % (vj_jd, exp_jd),
      t_jd is not None and abs(t_jd - exp_jd) <= 0.03 * exp_jd, t_jd)
check('square J0: corners at 20 mm/s jerk (%.3f s)' % exp_j0,
      t_j0 is not None and abs(t_j0 - exp_j0) <= 0.03 * exp_j0, t_j0)

def burst_time(sec):
    t = starts(sec)
    return t[6] - t[0] if len(t) >= 7 else None
t_off, t_on = burst_time('sd_off'), burst_time('sd_on')
check('B0: burst of 6 x 1 mm at full speed (< 0.25 s)', t_off is not None and t_off < 0.25, t_off)
check('B100000: burst stretched (> 0.1 s longer)', t_on is not None and t_off is not None and
      t_on > t_off + 0.1, t_on)

def single_time(sec):
    t = starts(sec)
    return t[1] - t[0] if len(t) >= 2 else None
area = math.pi / 4 * 1.75 ** 2
v_lim = 5.0 / (1.0 / 20.0 * area)            # mm/s
exp_lim = seg_time(20.0, 0, 0, v_lim)
exp_full = seg_time(20.0, 0, 0, V)
t_lim, t_tr, t_off2 = single_time('vl_on'), single_time('vl_travel'), single_time('vl_off')
check('M200 L5: G1 X20 E1 at %.1f mm/s (%.3f s)' % (v_lim, exp_lim),
      t_lim is not None and abs(t_lim - exp_lim) <= 0.03 * exp_lim, t_lim)
check('  travel without E at full speed (%.3f s)' % exp_full,
      t_tr is not None and abs(t_tr - exp_full) <= 0.03 * exp_full, t_tr)
check('  L0: full speed', t_off2 is not None and abs(t_off2 - exp_full) <= 0.03 * exp_full, t_off2)
check('M200 D1.75: E2.405 mm^3 = 1672 steps', abs(e_pulses('ve_on') - E_SPM) <= 1, e_pulses('ve_on'))
check('M200 S0: E1 = 1672 steps', abs(e_pulses('ve_off') - E_SPM) <= 1, e_pulses('ve_off'))
def xy_pulses(sec, tim):
    return sum(1 for l in sections.get(sec, [])
               if re.search(r'%s: .*WriteUInt32 to 0x0 .*value 0x9\b' % tim, l))
def pos(sec):
    for l in uart(sec):
        m = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+)', l)
        if m: return tuple(float(v) for v in m.groups())
check('M852 J0.1 K0.1 reported', 'M852 I0.000000 J0.100000 K0.100000' in uart('sk_set'), uart('sk_set'))
check('  Z +1 mm: X and Y motors 0.1 mm (16 steps) each',
      xy_pulses('sk_up', 'timer1') == 16 and xy_pulses('sk_up', 'timer2') == 16,
      (xy_pulses('sk_up', 'timer1'), xy_pulses('sk_up', 'timer2')))
check('  M114 unchanged X120 Y100', pos('sk_up') is not None and pos('sk_up')[:2] == (120.0, 100.0), pos('sk_up'))
check('  Z back: 16 steps back each',
      xy_pulses('sk_down', 'timer1') == 16 and xy_pulses('sk_down', 'timer2') == 16,
      (xy_pulses('sk_down', 'timer1'), xy_pulses('sk_down', 'timer2')))
check('  J0 K0: no X / Y steps on Z moves',
      xy_pulses('sk_off', 'timer1') == 0 and xy_pulses('sk_off', 'timer2') == 0,
      (xy_pulses('sk_off', 'timer1'), xy_pulses('sk_off', 'timer2')))
rep = uart('rep')
check('reports: M200, M205 B J', 'M200 S1 D1.750 L5.000' in rep and 'M205 B30000 J0.050' in rep,
      [l for l in rep if l.startswith('M200') or l.startswith('M205')])
st = [l for l in uart('st') if l.startswith('M200')]
check('M500 / M502 / M501', st == ['M200 S0 D1.750 L0.000', 'M200 S1 D1.750 L5.000'], st)
stream = [re.sub(r'^.*\] ', '', l) for l in out.splitlines() if 'usart2: [host' in l]
check('no error', not any(l.startswith('Error') or 'out of range' in l for l in stream),
      [l for l in stream if 'rror' in l or 'range' in l])
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
