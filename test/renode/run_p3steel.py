#!/usr/bin/env python3
"""Renode test of the Prusa i3 Steel configuration (config/board.blackpill_p3steel.h,
config/printer.p3steel.h), built without test overrides: make CHIP=F401 TEST=0.

Renode has no STM32F4 ADC, so the ADC DMA buffer (adc_buffer) is written
directly: this tests analog_read() + Steinhart-Hart conversion + heater
control end to end with real thermistor settings.

Usage: run_p3steel.py [renode] [teacup.elf] [platform.repl] [F_CPU]
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')
F_CPU = int(sys.argv[4]) if len(sys.argv) > 4 else 84000000

OVERSAMPLE, SLOTS, ADC_MAX = 16, 2, 4095   # board: 16 samples, hotend + bed

# ---- Reference Steinhart-Hart, same three points as the board config ----
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
def c_to_adc(t):                          # invert numerically
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
def mark(name): cmd('echo "@@MARK %s"' % name)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def pin(port, n, level): cmd('sysbus.gpioPort%s OnGPIO %d %s' % (port, n, 'true' if level else 'false'))
def adc(hotend, bed):                     # raw values into all samples
    for i in range(OVERSAMPLE):
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 0), hotend))
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 1), bed))
def read32(tag, addr):
    cmd('echo "@@%s"' % tag); cmd('sysbus ReadDoubleWord 0x%08X' % addr)

for _m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', _m))
cmd('mach create "p3"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
# Filament runout input PA0: pull-up on the board = filament present.
cmd('sysbus.gpioPortA OnGPIO 0 true')
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
cmd('logLevel -1 sysbus.gpioPortA'); cmd('logLevel -1 sysbus.gpioPortB')
for _t in ('timer1', 'timer2', 'timer3'):
    cmd('logLevel -1 sysbus.%s' % _t)
for port, n in (('B', 10), ('B', 3), ('B', 15)):
    pin(port, n, True)                    # endstops released (active low)
ROOM = c_to_adc(25.0)
mark('boot')
run('0.02'); adc(ROOM, ROOM)              # after .bss init, before protection
run('0.4')
cmd('sysbus LogPeripheralAccess sysbus.gpioPortA true')
cmd('sysbus LogPeripheralAccess sysbus.gpioPortB true')
for _t in ('timer1', 'timer2', 'timer3'):     # STEP_TIMER_PULSES: X, Y, E
    cmd('sysbus LogPeripheralAccess sysbus.%s true' % _t)

# Temperature conversion: several points, compare with the reference.
TEMPS = [25.0, 60.0, 150.0, 200.0, 250.0]
BED_T = lambda t: min(t, 100.0)          # the bed would trigger MAXTEMP (150)
for t in TEMPS:
    adc(c_to_adc(t), c_to_adc(BED_T(t))); run('0.3')
    mark('t_%d' % int(t)); send('M105\n'); run('0.05')
adc(ROOM, ROOM); run('0.3')

# Supply on first (PS_ON, 500 ms power-up wait), like OctoPrint's PSU
# control does. Otherwise the first move waits for it.
send('M80\n'); run('0.7')

# Motion with the real steps/mm.
mark('m_x'); send('G1 X10 F3000\n'); run('0.6')
mark('m_z'); send('G1 Z1 F240\n'); run('0.6')
mark('m_e'); send('G1 E2 F1500\n'); run('0.5')
mark('m_pos'); send('M114\n'); run('0.05')
mark('m_limit'); send('G1 X300 F9000\n'); run('2.0')
mark('m_limit_pos'); send('M114\n'); run('0.05')
pin('B', 10, False)
# Z goes up from 1 to 5 mm first (Z_HOMING_HEIGHT), about 1.2 s.
mark('m_home'); send('G28 X\n'); run('2.0')
pin('B', 10, True); run('1.0')
mark('m_home_pos'); send('M114\n'); run('0.05')
# Y/Z endstops on PB3/PB15 (final pin plan).
mark('m_es_open'); send('M119\n'); run('0.05')
pin('B', 3, False); pin('B', 15, False)
mark('m_es_pressed'); send('M119\n'); run('0.05')
pin('B', 3, True); pin('B', 15, True); run('0.01')

# Heaters: PWM frequencies, PID hotend, PID bed with slow software PWM.
read32('TIM10_PSC', 0x40014428); read32('TIM11_PSC', 0x40014828)
mark('h_start'); send('M104 S200\nM140 S60\n'); run('0.6')
read32('CCR_COLD', 0x40014434)
read32('ODR_BED_COLD', 0x40020414)
adc(c_to_adc(200.0), c_to_adc(70.0)); run('1.0')
read32('CCR_HOT', 0x40014434)
read32('ODR_BED_70', 0x40020414)
# Bed PID, slow software PWM 2 Hz on PB0: start again at 59 C (1 C below
# the target, no D kick from a temperature jump), Kp 70 -> ~28 % duty.
send('M140 S0\n'); run('0.1')
adc(c_to_adc(200.0), c_to_adc(59.0)); run('0.3')
send('M140 S60\n'); run('0.3')
mark('h_bed_pid'); run('2.0')
mark('h_bed_pid_end'); send('M104 S0\nM140 S0\n'); run('0.1')
read32('ODR_BED_OFF', 0x40020414)
mark('h_end'); send('M105\n'); run('0.1')
mark('end')
cmd('quit')

script = '/tmp/p3steel_test.resc'
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script], stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout.read()); proc.wait()
open('/tmp/p3steel_test.log', 'w').write(out)

sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def uart(sec):
    r = [re.sub(r'^.*\] ', '', l) for l in sections.get(sec, []) if 'usart2: [host' in l]
    return ['ok' if re.match(r'^ok( N-?\d+)? P\d+ B\d+$', l) else l for l in r]
TIMER_OF = {('A', 8): 'timer1', ('A', 15): 'timer2', ('B', 4): 'timer3'}
def pulses(sec, port, bit):
    # Timer pins: count the one pulse starts (CR1 = OPM | CEN).
    if (port, bit) in TIMER_OF:
        return sum(1 for l in sections.get(sec, [])
                   if re.search(r'%s: .*WriteUInt32 to 0x0 .*value 0x9\b' % TIMER_OF[(port, bit)], l))
    n = 0
    for l in sections.get(sec, []):
        m = re.search(r'gpioPort%s: .*WriteUInt32 to 0x18 \(BitSet\), value 0x([0-9A-F]+)' % port, l)
        if m and int(m.group(1), 16) & (1 << bit): n += 1
    return n
def pos(sec):
    for l in uart(sec):
        m = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+) E:(-?[\d.]+) Count', l)
        if m: return tuple(float(v) for v in m.groups())
v = {}
for tag in re.findall(r'@+(\w+)\s*\n', out):
    m = re.search(r'@+%s\s*\n(?:.*\n)*?\s*(0x[0-9A-Fa-f]+)\s*\n' % tag, out)
    if m: v[tag] = int(m.group(1), 16)
# GPIO reads: take the logged peripheral access, the monitor output may
# interleave with log lines.
for tag in [t for t in v if t.startswith('ODR_')]:
    m = re.search(r'@+%s.*?OutputData\), returned (0x[0-9A-Fa-f]+)' % tag, out, re.S)
    if m: v[tag] = int(m.group(1), 16)

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-44s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1

u = uart('boot')
check('boot', u and u[0] == 'start' and 'ok' in u, u)
for t in TEMPS:
    exp = adc_to_c(c_to_adc(t))
    expb = adc_to_c(c_to_adc(BED_T(t)))
    got = [re.findall(r'[TB]:([\d.]+)/', l) for l in uart('t_%d' % int(t)) if l.startswith('ok T:')]
    ok = bool(got) and len(got[0]) == 2 and abs(float(got[0][0]) - exp) <= 0.3 and abs(float(got[0][1]) - expb) <= 0.3
    check('S-H %5.1f C (adc %4d, ref %.2f)' % (t, c_to_adc(t), exp), ok, uart('t_%d' % int(t)))
sp = [l for l in u if l.startswith('echo:Step pulses:')]
check('step pulses: X TIM1 Y TIM2 Z gpio E TIM3', sp == ['echo:Step pulses: X TIM1 Y TIM2 Z gpio E TIM3'], sp)
check('X 10 mm = 1600 timer pulses (TIM1, PA8)', pulses('m_x', 'A', 8) == 1600, pulses('m_x', 'A', 8))
check('Z 1 mm = 8000 GPIO steps (PB12, Z2 in parallel)', pulses('m_z', 'B', 12) == 8000, pulses('m_z', 'B', 12))
check('E 2 mm = 3344 timer pulses (TIM3, PB4)', pulses('m_e', 'B', 4) == 3344, pulses('m_e', 'B', 4))
check('no GPIO step pulses on the old pins', pulses('m_x', 'B', 12) == 0 and pulses('m_e', 'B', 3) == 0,
      (pulses('m_x', 'B', 12), pulses('m_e', 'B', 3)))
check('M114 X10 Z1 E2', pos('m_pos') == (10.0, 0.0, 1.0, 2.0), pos('m_pos'))
check('soft limit: G1 X300 -> X220', pos('m_limit_pos') is not None and pos('m_limit_pos')[0] == 220.0
      and pulses('m_limit', 'A', 8) == 210 * 160, (pos('m_limit_pos'), pulses('m_limit', 'A', 8)))
es0 = ' '.join(uart('m_es_open')); es1 = ' '.join(uart('m_es_pressed'))
check('M119: Y/Z endstops on PB3/PB15', 'y_min:open' in es0 and 'z_min:open' in es0
      and 'y_min:triggered' in es1 and 'z_min:triggered' in es1 and 'x_min:open' in es1, (es0, es1))
check('G28 X (pull-up, active low)', 'ok' in uart('m_home') + uart('m_home_pos') and pos('m_home_pos')
      and pos('m_home_pos')[0] == 0.0, (uart('m_home'), pos('m_home_pos')))
f4 = F_CPU / ((v.get('TIM10_PSC', 0) + 1) * 1020.0)
f3 = F_CPU / ((v.get('TIM11_PSC', 0) + 1) * 1020.0)
check('hotend PWM 100 Hz (TIM10), fan 500 Hz (TIM11)', abs(f4 - 100) < 1 and abs(f3 - 500) < 5, ('%.1f Hz' % f4, '%.1f Hz' % f3))
check('PID: full power when cold', v.get('CCR_COLD') == 1020, v.get('CCR_COLD'))
check('PID: less power at target', v.get('CCR_HOT', 1020) < 1020, v.get('CCR_HOT'))
bed = lambda k: (v.get(k, 0) >> 0) & 1      # PB0
check('bed PID: 25 C (35 below) full on', bed('ODR_BED_COLD') == 1, hex(v.get('ODR_BED_COLD', 0)))
check('bed PID: 70 C (10 above) off', bed('ODR_BED_70') == 0, hex(v.get('ODR_BED_70', 0)))
# PB0 is written every 10 ms tick: BSRR bit 0 = on, bit 16 = off.
ticks = []
for l in sections.get('h_bed_pid', []):
    m = re.search(r'gpioPortB: .*WriteUInt32 to 0x18 .*value 0x([0-9A-F]+)', l)
    if m:
        val = int(m.group(1), 16)
        if val & 1: ticks.append(1)
        elif val & 0x10000: ticks.append(0)
rises = sum(1 for a, b in zip(ticks, ticks[1:]) if b and not a)
duty = sum(ticks) / len(ticks) if ticks else 0
check('bed PID 1 C below: ~28 % duty', 180 <= len(ticks) <= 220 and 0.2 <= duty <= 0.36, (len(ticks), round(duty, 3)))
check('  slow PWM 2 Hz: 4 pulses in 2 s', 3 <= rises <= 5, rises)
check('  M140 S0: off', bed('ODR_BED_OFF') == 0, hex(v.get('ODR_BED_OFF', 0)))
check('no error / reset', 'Error:' not in out and out.count('] start') == 1, out.count('] start'))
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
