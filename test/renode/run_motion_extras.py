#!/usr/bin/env python3
"""Renode test of skew correction (M852), Z backlash compensation (M425)
and their storage on the P3 Steel build (make CHIP=F401 TEST=0).

X, Y, E steps are timer one pulse starts (STEP_TIMER_PULSES: TIM1, TIM2,
TIM3), Z steps GPIO pulses on PB12.

Usage: run_motion_extras.py [renode] [teacup.elf] [platform.repl]

Checks:
  - M852 I0.01: G1 Y10 moves the X motor by -0.1 mm (16 steps), M114
    still reports X0 Y10; back to Y0 the X motor returns
  - M852 without parameters and M503 report the factor
  - M425 Z0.1: Z reversals get 0.1 mm (800 steps) extra, moves in the
    same direction none, M114 Z unaffected; M425 F0.5 half of it
  - M500, M502, M501 store and load M852 and M425
"""
import os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')

OVERSAMPLE, SLOTS = 16, 2
ROOM = 2048                              # thermistor ADC about 25 C
_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
ADCBUF = int(re.search(r'^([0-9a-f]+) \S adc_buffer$', _nm, re.M).group(1), 16)

lines = []
def cmd(c): lines.append(c)
def mark(name): cmd('echo "@@MARK %s"' % name)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def pin(port, n, level): cmd('sysbus.gpioPort%s OnGPIO %d %s' % (port, n, 'true' if level else 'false'))
def adc(hotend, bed):
    for i in range(OVERSAMPLE):
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 0), hotend))
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 1), bed))

for _m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', _m))
cmd('mach create "p3"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('sysbus.gpioPortA OnGPIO 0 true')    # filament present
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
cmd('logLevel -1 sysbus.gpioPortA'); cmd('logLevel -1 sysbus.gpioPortB')
for _t in ('timer1', 'timer2', 'timer3'):
    cmd('logLevel -1 sysbus.%s' % _t)
for port, n in (('B', 10), ('B', 3), ('B', 15)):
    pin(port, n, True)                   # endstops released (active low)
mark('boot')
run('0.02'); adc(ROOM, ROOM)
run('0.4')
cmd('sysbus LogPeripheralAccess sysbus.gpioPortB true')
for _t in ('timer1', 'timer2', 'timer3'):
    cmd('sysbus LogPeripheralAccess sysbus.%s true' % _t)
send('M80\n'); run('0.7')                # supply on, no wait on the first move

# ---- M852 skew ----
send('G92 X0 Y0 Z0 E0\n'); run('0.05')
mark('sk_set'); send('M852 I0.01\nM852\n'); run('0.1')
mark('sk_y10'); send('G1 Y10 F3000\n'); run('0.6')
mark('sk_pos'); send('M114\n'); run('0.05')
mark('sk_x10'); send('G1 X10\n'); run('0.6')
mark('sk_y0'); send('G1 Y0\n'); run('0.6')
mark('sk_pos0'); send('M114\n'); run('0.05')
mark('sk_off'); send('M852 I0\nG1 Y10\n'); run('0.6')
mark('sk_back'); send('G1 X0 Y0\n'); run('1.0')

# ---- M425 Z backlash ----
mark('bl_set'); send('M425 Z0.1\n'); run('0.05')
mark('bl_z1'); send('G1 Z1 F240\n'); run('1.0')       # first move: no reversal known
mark('bl_z2'); send('G1 Z2\n'); run('1.0')            # same direction
mark('bl_z15'); send('G1 Z1.5\n'); run('1.0')         # reversal
mark('bl_z1b'); send('G1 Z1\n'); run('1.0')           # same direction
mark('bl_z2b'); send('G1 Z2\n'); run('1.0')           # reversal
mark('bl_pos'); send('M114\n'); run('0.05')
mark('bl_half'); send('M425 F0.5\nG1 Z1\n'); run('1.0')
mark('bl_report'); send('M425\n'); run('0.1')

# ---- storage ----
mark('st_save'); send('M852 I-0.0025\nM500\n'); run('2.5')
mark('st_def'); send('M502\nM503\n'); run('0.2')
mark('st_load'); send('M501\nM503\n'); run('0.2')
mark('end')
cmd('quit')

script = '/tmp/motion_extras_test.resc'
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script], stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout.read()); proc.wait()
open('/tmp/motion_extras_test.log', 'w').write(out)

sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def uart(sec):
    r = [re.sub(r'^.*\] ', '', l) for l in sections.get(sec, []) if 'usart2: [host' in l]
    return ['ok' if re.match(r'^ok( N-?\d+)? P\d+ B\d+$', l) else l for l in r]
TIMER_OF = {'X': 'timer1', 'Y': 'timer2', 'E': 'timer3'}
def pulses(sec, axis):
    if axis in TIMER_OF:
        return sum(1 for l in sections.get(sec, [])
                   if re.search(r'%s: .*WriteUInt32 to 0x0 .*value 0x9\b' % TIMER_OF[axis], l))
    n = 0
    for l in sections.get(sec, []):             # Z: PB12
        m = re.search(r'gpioPortB: .*WriteUInt32 to 0x18 \(BitSet\), value 0x([0-9A-F]+)', l)
        if m and int(m.group(1), 16) & (1 << 12): n += 1
    return n
def pos(sec):
    for l in uart(sec):
        m = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+) E:(-?[\d.]+) Count', l)
        if m: return tuple(float(v) for v in m.groups())
def line(sec, prefix):
    for l in uart(sec):
        l = l.replace('echo:', '').strip()
        if l.startswith(prefix): return l

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-44s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1

u = uart('boot')
check('boot', bool(u) and u[0] == 'start', u)
check('M852 I0.01 reported', line('sk_set', 'M852') == 'M852 I0.010000', uart('sk_set'))
check('skew: G1 Y10 -> X motor 16 steps', pulses('sk_y10', 'X') == 16 and pulses('sk_y10', 'Y') == 1600,
      (pulses('sk_y10', 'X'), pulses('sk_y10', 'Y')))
check('  M114 X0 Y10', pos('sk_pos') is not None and pos('sk_pos')[:2] == (0.0, 10.0), pos('sk_pos'))
check('  G1 X10: 1600 X steps', pulses('sk_x10', 'X') == 1600, pulses('sk_x10', 'X'))
check('  G1 Y0: X motor back 16 steps', pulses('sk_y0', 'X') == 16, pulses('sk_y0', 'X'))
check('  M114 X10 Y0', pos('sk_pos0') is not None and pos('sk_pos0')[:2] == (10.0, 0.0), pos('sk_pos0'))
check('M852 I0: no X steps on Y moves', pulses('sk_off', 'X') == 0 and pulses('sk_off', 'Y') == 1600,
      (pulses('sk_off', 'X'), pulses('sk_off', 'Y')))
zs = [pulses(s, 'Z') for s in ('bl_z1', 'bl_z2', 'bl_z15', 'bl_z1b', 'bl_z2b')]
check('backlash: extra 800 steps on reversals only', zs == [8000, 8000, 4800, 4000, 8800], zs)
check('  M114 Z2', pos('bl_pos') is not None and pos('bl_pos')[2] == 2.0, pos('bl_pos'))
check('  M425 F0.5: 400 extra', pulses('bl_half', 'Z') == 8400, pulses('bl_half', 'Z'))
check('  M425 reported', line('bl_report', 'M425') == 'M425 F0.50 Z0.10', uart('bl_report'))
check('M500 stores', any(l.startswith('echo:Settings Stored') for l in uart('st_save')), uart('st_save'))
check('M502: defaults', line('st_def', 'M852') == 'M852 I0.000000' and line('st_def', 'M425') == 'M425 F1.00 Z0.00',
      (line('st_def', 'M852'), line('st_def', 'M425')))
check('M501: loaded', line('st_load', 'M852') == 'M852 I-0.002500' and line('st_load', 'M425') == 'M425 F0.50 Z0.10',
      (line('st_load', 'M852'), line('st_load', 'M425')))
check('no error / reset', 'Error:' not in out and out.count('] start') == 1, out.count('] start'))
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
