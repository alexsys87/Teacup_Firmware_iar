#!/usr/bin/env python3
"""Renode test of skew correction (M852), Z backlash compensation (M425),
firmware retract (G10, G11, M207, M208), host lost (M86) and their storage on the P3 Steel build (make CHIP=F401 TEST=0).

X, Y, E steps are timer one pulse starts (STEP_TIMER_PULSES: TIM1, TIM2,
TIM3), Z steps GPIO pulses on PB12.

Usage: run_motion_extras.py [renode] [teacup.elf] [platform.repl]

Checks:
  - M852 I0.01: G1 Y10 moves the X motor by -0.1 mm (16 steps), M114
    still reports X0 Y10; back to Y0 the X motor returns
  - M852 without parameters and M503 report the factor
  - M425 Z0.1: Z reversals get 0.1 mm (800 steps) extra, moves in the
    same direction none, M114 Z unaffected; M425 F0.5 half of it
  - M425 X / Y: the same on X and Y; S2 spreads the correction over 2 mm
    of moves (share distance / S per move), S0 all in the first move
  - G10 / G11 (M207 S F Z, M208 S F): E retract and prime, Z lift kept
    for travel moves, G-code coordinates unchanged, repeated G10 ignored
  - G28 X / G28 Y lift Z to Z_HOMING_HEIGHT first, only once
  - M86 S3: no line for 3 s while printing (armed by a move with E) ->
    retract, Z up, park, hotend off; a long G4 doesn't count
  - M500, M502, M501 store and load M852, M425 and M207 / M208
  - hotend fan (PCF8574 P6) on at 50 C, controller fan (P7) while the
    drivers are enabled and 60 s afterwards (models/TeacupPCF8574.cs)
"""
import math, os, re, subprocess, sys

from renode_common import artifact_path, monitor_command, run_renode

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')

OVERSAMPLE, SLOTS, ADC_MAX = 16, 2, 4095
RP, PTS = 4700.0, [(25.0, 100000.0), (150.0, 1641.9), (250.0, 226.15)]
def sh_coef():                           # Steinhart-Hart, as the board config
    (t1, r1), (t2, r2), (t3, r3) = PTS
    l1, l2, l3 = map(math.log, (r1, r2, r3))
    y1, y2, y3 = (1 / (t + 273.15) for t in (t1, t2, t3))
    g2, g3 = (y2 - y1) / (l2 - l1), (y3 - y1) / (l3 - l1)
    c = (g3 - g2) / (l3 - l2) / (l1 + l2 + l3)
    b = g2 - c * (l1 * l1 + l1 * l2 + l2 * l2)
    return y1 - (b + l1 * l1 * c) * l1, b, c
A, B, C = sh_coef()
def adc_to_c(adc):
    l = math.log(RP * adc / (ADC_MAX - adc))
    return 1 / (A + B * l + C * l ** 3) - 273.15
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
def cmd(c): lines.append(monitor_command(c))
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

for _m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs', 'TeacupPCF8574.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', _m))
cmd('mach create "p3"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
# PCF8574 at 0x20: hotend fan P6, controller fan P7 (board file).
cmd('machine LoadPlatformDescriptionFromString "pcf: I2C.TeacupPCF8574 @ i2c1 0x20"')
def pcf(tag): cmd('echo "@@PCF %s"' % tag); cmd('sysbus.i2c1.pcf Output')
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
pcf('boot')
send('M80\n'); run('0.7')                # supply on, no wait on the first move

# ---- M852 skew ----
send('G92 X0 Y0 Z0 E0\n'); run('0.05')
mark('sk_set'); send('M852 I0.01\nM852\n'); run('0.1')
mark('sk_y10'); send('G1 Y10 F3000\n'); run('0.6')
mark('sk_pos'); send('M114\n'); run('0.05'); pcf('moving')
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

# ---- M425 X / Y backlash, smoothing (F0.5 still: 0.1 mm -> 8 steps) ----
mark('bx_x10'); send('M425 Z0 X0.1 S2\nG1 X10 F3000\n'); run('1.0')  # reversal, 10 mm >= S: all 8
mark('bx_x9'); send('G1 X9\n'); run('0.4')           # reversal, 1 mm of S2: half (4)
mark('bx_x8'); send('G1 X8\n'); run('0.4')           # half of the rest (2)
mark('bx_x4'); send('G1 X4\n'); run('0.6')           # 4 mm: the rest (2)
mark('bx_s0'); send('M425 S0\nG1 X10\n'); run('0.8')  # reversal, no smoothing: all 8
mark('bx_pos'); send('M114\n'); run('0.05')
mark('by_y5'); send('M425 X0 Y0.1\nG1 Y5\n'); run('0.6')   # Y reversal: 8
mark('by_y0'); send('G1 Y0\n'); run('0.6')           # reversal: 8
mark('bx_report'); send('M425\n'); run('0.1')
mark('bx_off'); send('M425 Y0\nG1 X0\n'); run('0.8')   # off: no extra

# ---- G10 / G11 firmware retract ----
send('M425 Z0\nG92 E0\n'); run('0.1')
mark('rt_set'); send('M207 S1.5 F1500 Z0.2\nM208 S0 F1200\nM207\n'); run('0.2')
mark('rt_g10'); send('G10\n'); run('0.6')
mark('rt_pos'); send('M114\n'); run('0.05')
mark('rt_travel'); send('G1 X5 F3000\n'); run('0.6')
mark('rt_g10b'); send('G10\n'); run('0.3')
mark('rt_g11'); send('G11\n'); run('0.6')
mark('rt_pos2'); send('M114\n'); run('0.05')
mark('rt_extra'); send('M208 S0.1\nG10\nG11\n'); run('1.2')
mark('rt_print'); send('G1 X10 E1 F1200\n'); run('0.6')
mark('rt_pos3'); send('M114\n'); run('0.05')
mark('rt_report'); send('M208\n'); run('0.1')

# ---- M86 host lost ----
# Hotend at 200 C (thermistor ADC), X and Y homed for parking.
HOT = c_to_adc(200.0)
# The simulated temperature jumps from 25 to 200 C. With the heater off
# (or its PID output at 0 from the derivative of that jump) this is what
# the thermal protection calls "heater off but temperature rising" (stuck
# MOSFET), one check period (20 s) later. So the heater runs at full power
# during the jump (target 230, beyond PID_FUNCTIONAL_RANGE), then the
# target is 200: the protection starts watching from 200 C.
send('M104 S230\n'); run('0.1')
adc(HOT, ROOM); run('0.5')
send('M104 S200\n'); run('0.3')
# The first G28 lifts Z from 1 to 5 mm first (Z_HOMING_HEIGHT, ~1.2 s).
mark('hl_home')
for port, n, ax in (('B', 10, 'X'), ('B', 3, 'Y')):
    pin(port, n, False)
    send('G28 %s\n' % ax); run('2.0')
    pin(port, n, True); run('1.0')
mark('hl_homed'); send('M114\n'); run('0.05')
# Fewer Z steps for the 10 mm lift (faster simulation).
mark('hl_set'); send('M92 Z400\nM86 S3 E0\nM86\n'); run('0.2')
mark('hl_g4'); send('G1 X20 Y20 E0.5 F3000\nG4 P5000\n'); run('6.0')
mark('hl_wait'); run('1.5')                   # 1.5 s after G4: not yet
mark('hl_lost'); run('9.0')                   # 3 s: lost, park
mark('hl_pos'); send('M114\nM105\n'); run('0.1')

# ---- storage ----
mark('st_save'); send('M852 I-0.0025\nM500\n'); run('2.5')
mark('st_def'); send('M502\nM503\n'); run('0.2')
mark('st_load'); send('M501\nM503\n'); run('0.2')
# ---- hotend fan (P6, >= 50 C), controller fan (P7) ----
send('M104 S60\n'); adc(c_to_adc(60.0), ROOM); run('0.6'); pcf('hot60')
send('M104 S0\n'); adc(c_to_adc(45.0), ROOM); run('0.6'); pcf('cool45')
send('M84\n'); run('2.0'); pcf('idle2')
run('60.0'); pcf('idle62')
mark('end')
cmd('quit')

script = artifact_path('motion_extras_test.resc')
open(script, 'w').write('\n'.join(lines) + '\n')
out = re.sub(r'\x1b\[[0-9;]*m', '', run_renode(RENODE, script))
open(artifact_path('motion_extras_test.log'), 'w').write(out)

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
check('  M425 reported', line('bl_report', 'M425') == 'M425 F0.50 S0.00 X0.00 Y0.00 Z0.10', uart('bl_report'))
xs = [pulses(s, 'X') for s in ('bx_x10', 'bx_x9', 'bx_x8', 'bx_x4', 'bx_s0')]
check('backlash X, S2: 8, 4 + 2 + 2, then S0: 8', xs == [1608, 164, 162, 642, 968], xs)
check('  M114 X10', pos('bx_pos') is not None and pos('bx_pos')[0] == 10.0, pos('bx_pos'))
ys = [pulses(s, 'Y') for s in ('by_y5', 'by_y0')]
check('backlash Y: 8 extra on both reversals', ys == [808, 808], ys)
check('  M425 reported', line('bx_report', 'M425') == 'M425 F0.50 S0.00 X0.00 Y0.10 Z0.00', uart('bx_report'))
check('  off: no extra', pulses('bx_off', 'X') == 1600, pulses('bx_off', 'X'))
check('M207 reported', line('rt_set', 'M207') == 'M207 S1.50 F1500.00 Z0.20', uart('rt_set'))
check('G10: 1.5 mm E back, 0.2 mm Z up', pulses('rt_g10', 'E') == 2508 and pulses('rt_g10', 'Z') == 1600,
      (pulses('rt_g10', 'E'), pulses('rt_g10', 'Z')))
check('  M114: E, Z coordinates unchanged', pos('rt_pos') is not None and pos('rt_pos')[2:] == (1.0, 0.0), pos('rt_pos'))
check('  travel keeps the lift (no Z steps)', pulses('rt_travel', 'X') == 800 and pulses('rt_travel', 'Z') == 0,
      (pulses('rt_travel', 'X'), pulses('rt_travel', 'Z')))
check('  second G10 ignored', pulses('rt_g10b', 'E') == 0 and pulses('rt_g10b', 'Z') == 0,
      (pulses('rt_g10b', 'E'), pulses('rt_g10b', 'Z')))
check('G11: Z down, 1.5 mm E primed', pulses('rt_g11', 'E') == 2508 and pulses('rt_g11', 'Z') == 1600,
      (pulses('rt_g11', 'E'), pulses('rt_g11', 'Z')))
check('  M114 X5 Z1 E0', pos('rt_pos2') == (5.0, 0.0, 1.0, 0.0), pos('rt_pos2'))
check('M208 S0.1: 0.1 mm more primed', abs(pulses('rt_extra', 'E') - (2508 + 2508 + 167)) <= 1,
      pulses('rt_extra', 'E'))
check('  absolute E continues (E1 = 1672 steps)', pulses('rt_print', 'E') == 1672 and pos('rt_pos3') == (10.0, 0.0, 1.0, 1.0),
      (pulses('rt_print', 'E'), pos('rt_pos3')))
check('  M208 reported', line('rt_report', 'M208') == 'M208 S0.10 F1200.00', uart('rt_report'))
check('G28 X: Z lifted to 5 mm first (Z_HOMING_HEIGHT), once', pulses('hl_home', 'Z') == 32000
      and pos('hl_homed') is not None and pos('hl_homed')[:3] == (0.0, 0.0, 5.0), (pulses('hl_home', 'Z'), pos('hl_homed')))
check('M86 reported', line('hl_set', 'M86') == 'M86 S3 E0', uart('hl_set'))
check('host lost: not during G4 P5000 nor 1.5 s after', not any('Host lost' in l for l in uart('hl_g4') + uart('hl_wait')),
      uart('hl_g4') + uart('hl_wait'))
ul = uart('hl_lost')
check('host lost after 3 s without lines', 'echo:No line from the host for 3 s' in ul and 'echo:Host lost, parking' in ul
      and 'echo:Parked, hotend temperature lowered' in ul, ul)
check('  retract 2 mm, Z up 10 mm', pulses('hl_lost', 'E') == 3344 and pulses('hl_lost', 'Z') == 4000,
      (pulses('hl_lost', 'E'), pulses('hl_lost', 'Z')))
p = pos('hl_pos')
check('  parked at X10 Y170, Z + 10', p is not None and p[:3] == (10.0, 170.0, 15.0), p)
check('  hotend target 0', any(re.match(r'ok T:[\d.]+/0\.0 ', l) for l in uart('hl_pos')), uart('hl_pos'))
check('M500 stores', any(l.startswith('echo:Settings Stored') for l in uart('st_save')), uart('st_save'))
check('M502: defaults', line('st_def', 'M852') == 'M852 I0.000000' and line('st_def', 'M425') == 'M425 F1.00 S0.00 X0.00 Y0.00 Z0.00',
      (line('st_def', 'M852'), line('st_def', 'M425')))
check('M501: loaded', line('st_load', 'M852') == 'M852 I-0.002500' and line('st_load', 'M425') == 'M425 F0.50 S0.00 X0.00 Y0.00 Z0.00'
      and line('st_load', 'M207') == 'M207 S1.50 F1500.00 Z0.20',
      (line('st_load', 'M852'), line('st_load', 'M425')))
PCF = {}
for m in re.finditer(r'@@PCF (\w+)\s*\n(?:.*\n)*?\s*(0x[0-9A-Fa-f]+|\d+)\s*\n', out):
    PCF[m.group(1)] = int(m.group(2), 0)
fan = lambda t, bit: (PCF.get(t, -1) >> bit) & 1 if t in PCF else None
check('PCF8574: both fans off at boot, P0..P5 high', PCF.get('boot') == 0x3F, PCF)
check('controller fan on while moving', fan('moving', 7) == 1, PCF.get('moving'))
check('hotend fan on at 60 C', fan('hot60', 6) == 1, PCF.get('hot60'))
check('hotend fan off at 45 C', fan('cool45', 6) == 0, PCF.get('cool45'))
check('controller fan: on 2 s after M84, off after 60 s', fan('idle2', 7) == 1 and fan('idle62', 7) == 0,
      (PCF.get('idle2'), PCF.get('idle62')))
check('no error / reset', 'Error:' not in out and out.count('] start') == 1, out.count('] start'))
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
