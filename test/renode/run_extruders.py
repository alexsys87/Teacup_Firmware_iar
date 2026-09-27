#!/usr/bin/env python3
"""Renode test of the second extruder (EXTRUDERS 2, T0 / T1, M218) on the
P3 Steel configuration, E1 STEP on PC15, E1 DIR on PC14:

  make CHIP=F401 TEST=0 EXTRA="-DEXTRUDERS=2" BUILD_SUFFIX=_e2
  run_extruders.py [renode] [teacup.elf]

E0 steps are TIM3 one pulse starts (STEP_TIMER_PULSES), E1 steps GPIO
pulses (GPIOC BSRR bit 15).

Checks:
  - T0: E moves step E0 only
  - M218 T1 X10 Y-5 Z0.5, report; T1: "Active Extruder: 1", same
    physical position, M114 in T1 coordinates (X10 Y-5 Z0.5)
  - T1: E moves step E1 only, E1 DIR follows the E direction (not
    inverted, E0 has E_INVERT_DIR), XY moves in T1 coordinates
  - linear advance (M900 K) steps the active extruder only
  - T0: coordinates back, E0 steps again
  - M218 while T1 is active shifts right away
  - T2 and M218 T0 rejected
  - M503 reports M218; M500 / M502 / M501 store and load it
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0_e2/teacup.elf')
REPL = os.path.join(HERE, 'stm32f401_dma.repl')

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
ROOM = c_to_adc(25.0)
_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
ADCBUF = int(re.search(r'^([0-9a-f]+) \S adc_buffer$', _nm, re.M).group(1), 16)

lines = []
def cmd(c): lines.append(c)
def mark(name): cmd('echo "@@MARK %s"' % name)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def adc(hotend, bed):
    for i in range(OVERSAMPLE):
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 0), hotend))
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 1), bed))
def odr(tag):
    cmd('echo "@@ODRB %s"' % tag); cmd('sysbus ReadDoubleWord 0x40020414')   # GPIOB ODR
    cmd('echo "@@ODRC %s"' % tag); cmd('sysbus ReadDoubleWord 0x40020814')   # GPIOC ODR

for _m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', _m))
cmd('mach create "p3"')
cmd('machine LoadPlatformDescription @%s' % REPL)
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
for _p in ('gpioPortA', 'gpioPortB', 'gpioPortC', 'timer1', 'timer2', 'timer3'):
    cmd('logLevel -1 sysbus.%s' % _p)
cmd('sysbus.gpioPortA OnGPIO 0 true')    # filament present
for port, n in (('B', 10), ('B', 3), ('B', 15)):
    cmd('sysbus.gpioPort%s OnGPIO %d true' % (port, n))
run('0.02'); adc(ROOM, ROOM); run('0.4')
for _p in ('gpioPortC', 'timer1', 'timer2', 'timer3'):
    cmd('sysbus LogPeripheralAccess sysbus.%s true' % _p)
send('M80\n'); run('0.7')
send('G92 X0 Y0 Z0 E0\n'); run('0.05')

mark('e0'); send('G1 E1 F600\n'); run('0.3')                      # 1 mm E, T0
mark('m218'); send('M218 T1 X10 Y-5 Z0.5\n'); run('0.05')
mark('t1'); send('T1\nM114\n'); run('0.1')
mark('e1'); send('G1 E2 F600\n'); run('0.3'); odr('e1_fwd')       # 1 mm E, T1
mark('e1r'); send('G1 E1.5\n'); run('0.3'); odr('e1_back')        # 0.5 mm back
mark('xy1'); send('G1 X20 Y5 F3000\n'); run('0.6')               # 10 mm X, 10 mm Y
mark('la1'); send('M900 K0.05\nG1 X30 E2.5 F3000\n'); run('0.8')  # 10 mm X, 1 mm E, LA
mark('la_off'); send('M900 K0\n'); run('0.05')
mark('t0'); send('T0\nM114\n'); run('0.1')
mark('e0b'); send('G1 E3.5 F600\n'); run('0.3')                  # 1 mm E, T0
mark('live'); send('T1\nM218 T1 X12\nM114\n'); run('0.1')        # X 20 -> 22
mark('bad'); send('T2\nM218 T0 X1\n'); run('0.1')
mark('rep'); send('M503\n'); run('0.3')
mark('st'); send('M500\nM502\nM218\nM501\nM218\n'); run('0.3')
mark('end')
cmd('quit')

script = '/tmp/teacup_extruders_%d.resc' % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.run([RENODE, '--console', '--disable-gui', script], capture_output=True, text=True)
out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout + proc.stderr)
open('/tmp/teacup_extruders.log', 'w').write(out)
os.remove(script)

sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def uart(sec):
    r = [re.sub(r'^.*\] ', '', l) for l in sections.get(sec, []) if 'usart2: [host' in l]
    return [l.replace('echo:', '').strip() for l in r]
TIMER_OF = {'X': 'timer1', 'Y': 'timer2', 'E0': 'timer3'}
def pulses(sec, axis):
    if axis in TIMER_OF:
        return sum(1 for l in sections.get(sec, [])
                   if re.search(r'%s: .*WriteUInt32 to 0x0 .*value 0x9\b' % TIMER_OF[axis], l))
    n = 0
    for l in sections.get(sec, []):             # E1: PC15
        m = re.search(r'gpioPortC: .*WriteUInt32 to 0x18 \(BitSet\), value 0x([0-9A-F]+)', l)
        if m and int(m.group(1), 16) & (1 << 15): n += 1
    return n
def pos(sec):
    for l in uart(sec):
        m = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+) E:(-?[\d.]+) Count', l)
        if m: return tuple(float(v) for v in m.groups())
def lines_with(sec, prefix):
    return [l for l in uart(sec) if l.startswith(prefix)]
V = {}
for m in re.finditer(r'@@(ODR[BC]) (\S+)\s*\n(?:.*\n)*?\s*(0x[0-9A-Fa-f]+)\s*\n', out):
    V[(m.group(1), m.group(2))] = int(m.group(3), 16)
def bit(reg, tag, b):
    v = V.get((reg, tag))
    return None if v is None else (v >> b) & 1

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-50s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1

E = 1672                                    # steps per mm
check('T0: G1 E1 -> 1672 E0 steps, no E1', pulses('e0', 'E0') == E and pulses('e0', 'E1') == 0,
      (pulses('e0', 'E0'), pulses('e0', 'E1')))
check('M218 T1 X10 Y-5 Z0.5 reported', lines_with('m218', 'M218') == ['M218 T1 X10.000 Y-5.000 Z0.500'],
      lines_with('m218', 'M218'))
check('T1: "Active Extruder: 1", M114 in T1 coordinates',
      lines_with('t1', 'Active Extruder') == ['Active Extruder: 1'] and pos('t1') == (10.0, -5.0, 0.5, 1.0),
      (lines_with('t1', 'Active'), pos('t1')))
check('  T1 itself moves nothing', pulses('t1', 'X') + pulses('t1', 'Y') + pulses('t1', 'E0') + pulses('t1', 'E1') == 0)
check('T1: G1 E2 -> 1672 E1 steps, no E0', pulses('e1', 'E1') == E and pulses('e1', 'E0') == 0,
      (pulses('e1', 'E1'), pulses('e1', 'E0')))
check('  E1 DIR high forward (E0 DIR inverted: low)', bit('ODRC', 'e1_fwd', 14) == 1 and bit('ODRB', 'e1_fwd', 5) == 0,
      (bit('ODRC', 'e1_fwd', 14), bit('ODRB', 'e1_fwd', 5)))
check('  back 0.5 mm: 836 E1 steps, E1 DIR low', pulses('e1r', 'E1') == E // 2 and bit('ODRC', 'e1_back', 14) == 0,
      (pulses('e1r', 'E1'), bit('ODRC', 'e1_back', 14)))
check('T1: G1 X20 Y5 -> 10 mm on X and Y', pulses('xy1', 'X') == 1600 and pulses('xy1', 'Y') == 1600,
      (pulses('xy1', 'X'), pulses('xy1', 'Y')))
check('linear advance on T1: E1 only, >= 1672 steps', pulses('la1', 'E1') >= E and pulses('la1', 'E0') == 0,
      (pulses('la1', 'E1'), pulses('la1', 'E0')))
check('T0: M114 in T0 coordinates (X20 Y10 Z0)',
      lines_with('t0', 'Active Extruder') == ['Active Extruder: 0'] and pos('t0') == (20.0, 10.0, 0.0, 2.5),
      (lines_with('t0', 'Active'), pos('t0')))
check('T0: G1 E3.5 -> 1672 E0 steps, no E1', pulses('e0b', 'E0') == E and pulses('e0b', 'E1') == 0,
      (pulses('e0b', 'E0'), pulses('e0b', 'E1')))
check('M218 X12 with T1 active: X shifts to 32', pos('live') == (32.0, 5.0, 0.5, 3.5), pos('live'))
check('T2 and M218 T0 rejected', lines_with('bad', 'T0 or T1 only') and lines_with('bad', 'M218: T1 only'),
      uart('bad'))
check('M503 reports M218', 'M218 T1 X12.00 Y-5.00 Z0.50' in uart('rep'), lines_with('rep', 'M218'))
st = lines_with('st', 'M218 T1')
check('M500 / M502 / M501: stored, defaults, loaded',
      st == ['M218 T1 X0.000 Y0.000 Z0.000', 'M218 T1 X12.000 Y-5.000 Z0.500'], st)
stream = [l for l in out.splitlines() if 'usart2: [host' in l]
check('no error', not any(re.sub(r'^.*\] ', '', l).startswith('Error') for l in stream),
      [l for l in stream if 'Error' in l])
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
