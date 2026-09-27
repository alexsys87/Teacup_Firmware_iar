#!/usr/bin/env python3
"""Renode test of the spindle / laser output (SPINDLE_LASER, M3 / M4 / M5)
on the P3 Steel configuration, PWM on PB9 (TIM11) instead of the fan:

  make CHIP=F401 TEST=0 EXTRA="-DSPINDLE_LASER -DLASER_MODE" BUILD_SUFFIX=_laser
  make CHIP=F401 TEST=0 EXTRA="-DSPINDLE_LASER -DSPINDLE_LASER_ENA_PIN=PC_14 -DSPINDLE_DIR_PIN=PC_15" BUILD_SUFFIX=_spindle
  run_laser.py [renode] laser|spindle [teacup.elf]

The PWM duty is read from TIM11 CCR1 (0..1000 = full power), enable and
direction from GPIOC ODR.

Laser (inline power):
  - M3 S128 alone doesn't switch the laser on, the following G1 runs with
    50 % (502), after the move off
  - G0 without power, G1 S255 with full power, M5: G1 without
  - M4 (dynamic): lower power while accelerating, full at speed
  - M112: off
Spindle:
  - M3 S128: 50 % right away, enable on, direction low; M4 S255: full,
    direction high; M5 waits for the move, then off
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
KIND = sys.argv[2] if len(sys.argv) > 2 else 'laser'
ELF = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, '../gcc/build_F401_0_%s/teacup.elf' % KIND)
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
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def adc(hotend, bed):
    for i in range(OVERSAMPLE):
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 0), hotend))
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 1), bed))
def sample(tag):
    cmd('echo "@@CCR %s"' % tag); cmd('sysbus ReadDoubleWord 0x40014834')    # TIM11 CCR1
    cmd('echo "@@ODRC %s"' % tag); cmd('sysbus ReadDoubleWord 0x40020814')   # GPIOC ODR

cmd('include @%s' % os.path.join(HERE, 'models', 'TeacupSTM32DMA.cs'))
cmd('include @%s' % os.path.join(HERE, 'models', 'TeacupSTM32_UART.cs'))
cmd('include @%s' % os.path.join(HERE, 'models', 'TeacupSTM32_OTGFS.cs'))
cmd('mach create "p3"')
cmd('machine LoadPlatformDescription @%s' % REPL)
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
cmd('sysbus.gpioPortA OnGPIO 0 true')
for port, n in (('B', 10), ('B', 3), ('B', 15)):
    cmd('sysbus.gpioPort%s OnGPIO %d true' % (port, n))
run('0.02'); adc(ROOM, ROOM); run('0.5')
cmd('echo "@@ARR"'); cmd('sysbus ReadDoubleWord 0x4001482C')
send('M80\n'); run('0.7')
send('G92 X0 Y0 Z0\n'); run('0.1')
sample('idle')

if KIND == 'laser':
    send('M3 S128\n'); run('0.1'); sample('m3')                   # no move: off
    send('G1 X20 F600\n'); run('1.0'); sample('g1')              # 2 s move
    run('1.5'); sample('g1_end')
    send('G0 X0\n'); run('0.05'); sample('g0')
    run('1.0')
    send('G1 X20 S255\n'); run('1.0'); sample('g1_s255')
    run('1.5')
    send('M5\nG1 X0\n'); run('1.0'); sample('m5_g1')
    run('1.5')
    # M4: dynamic power. 20 mm at F3000, acceleration 1000 mm/s^2: 50 mm/s
    # after 50 ms. Samples every 10 ms from the start.
    send('M4 S255\nG1 X20 F3000\n')
    for i in range(12):
        run('0.01'); sample('m4_%02d' % i)
    run('0.15'); sample('m4_mid')
    run('1.0')
    send('M3 S255\nG1 X0 F600\n'); run('0.5'); sample('kill_before')
    send('M112\n'); run('0.1'); sample('kill')
else:
    send('M3 S128\n'); run('0.2'); sample('m3')
    send('M4 S255\n'); run('0.2'); sample('m4')
    send('G1 X10 F600\nM5\n'); run('0.5'); sample('m5_moving')    # 1 s move
    run('1.0'); sample('m5_done')
cmd('quit')

script = '/tmp/teacup_laser_%s_%d.resc' % (KIND, os.getpid())
open(script, 'w').write('\n'.join(lines) + '\n')
# stdin stays open: at EOF on stdin Renode's console queues input events
# without end and runs out of memory.
proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script], stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout.read()); proc.wait()
open('/tmp/teacup_laser_%s.log' % KIND, 'w').write(out)
os.remove(script)

V = {}
for m in re.finditer(r'@@(CCR|ODRC|ARR) ?(\S*)\s*\n(?:.*\n)*?\s*(0x[0-9A-Fa-f]+)\s*\n', out):
    V[(m.group(1), m.group(2))] = int(m.group(3), 16)
ccr = lambda t: V.get(('CCR', t))
odrc = lambda t: V.get(('ODRC', t))
STREAM = [re.sub(r'^.*\] ', '', l) for l in out.splitlines() if 'usart2: [host' in l]

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-50s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1

check('PWM 0..1000 (ARR 999), off at start', V.get(('ARR', '')) == 999 and ccr('idle') == 0,
      (V.get(('ARR', '')), ccr('idle')))
if KIND == 'laser':
    check('M3 S128 without a move: off', ccr('m3') == 0, ccr('m3'))
    check('G1 with 50 % (502)', ccr('g1') == 502, ccr('g1'))
    check('  off after the move', ccr('g1_end') == 0, ccr('g1_end'))
    check('G0: off', ccr('g0') == 0, ccr('g0'))
    check('G1 S255: full', ccr('g1_s255') == 1000, ccr('g1_s255'))
    check('M5: G1 without power', ccr('m5_g1') == 0, ccr('m5_g1'))
    m4 = [ccr('m4_%02d' % i) for i in range(12)]
    ramp = [v for v in m4 if v]
    check('M4: power rises with the speed', len(ramp) >= 3 and ramp[0] < 700 and
          all(b >= a for a, b in zip(ramp, ramp[1:])), m4)
    check('  full at speed', ccr('m4_mid') is not None and ccr('m4_mid') >= 990, ccr('m4_mid'))
    check('M112: off', ccr('kill_before') == 1000 and ccr('kill') == 0, (ccr('kill_before'), ccr('kill')))
else:
    bit = lambda t, b: (odrc(t) >> b) & 1 if odrc(t) is not None else None
    check('M3 S128: 50 %, enable on, direction low', ccr('m3') == 502 and bit('m3', 14) == 1 and bit('m3', 15) == 0,
          (ccr('m3'), bit('m3', 14), bit('m3', 15)))
    check('M4 S255: full, direction high', ccr('m4') == 1000 and bit('m4', 15) == 1, (ccr('m4'), bit('m4', 15)))
    check('M5 waits for the move', ccr('m5_moving') == 1000, ccr('m5_moving'))
    check('  then off, enable off', ccr('m5_done') == 0 and bit('m5_done', 14) == 0, (ccr('m5_done'), bit('m5_done', 14)))
check('no error', not any(l.startswith('Error') for l in STREAM if 'M112' not in l and 'halted' not in l),
      [l for l in STREAM if l.startswith('Error')])
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
