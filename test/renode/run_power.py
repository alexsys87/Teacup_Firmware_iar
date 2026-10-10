#!/usr/bin/env python3
"""Renode test: power supply switch (PS_ON) and the "heater stuck on"
protection, on the working P3 Steel configuration (make CHIP=F401 TEST=0,
PS_ON_PIN PB2 with PS_INVERT_ON). Temperatures are simulated by writing
ADC values into the DMA buffer, like run_p3steel.py does.

Usage: run_power.py [renode] [teacup.elf] [platform.repl] [all|supply|hotend|bed]

  supply  PB2 low after boot, M80 / M81, heating switches the supply on
  hotend  heater off after heating: overshoot and slow rise (nozzle parked
          on a hot bed) must not trigger; a fast rise (shorted MOSFET) halts
          the printer and switches the supply off; M999 keeps it off
  bed     bed off, temperature rises: halt with Heater_ID: bed
"""
import math, os, re, subprocess, sys

from renode_common import artifact_path, monitor_command, run_renode

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')
PART = sys.argv[4] if len(sys.argv) > 4 else 'all'
def want(p): return PART == 'all' or p in PART.split(',')

OVERSAMPLE, SLOTS, ADC_MAX = 16, 2, 4095   # board: 16 samples, hotend + bed
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
def c_to_adc(t):
    lo, hi = 1, ADC_MAX - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if adc_to_c(mid) > t: lo = mid
        else: hi = mid
    return hi

_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
ADCBUF = int(re.search(r'^([0-9a-f]+) \S adc_buffer$', _nm, re.M).group(1), 16)

lines = []
def cmd(c): lines.append(monitor_command(c))
def mark(name): cmd('echo "@@MARK %s"' % name)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def temps(hotend, bed):                   # degree Celsius into all samples
    h, b = c_to_adc(hotend), c_to_adc(bed)
    for i in range(OVERSAMPLE):
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 0), h))
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 1), b))
def ramp(sec, h0, h1, b0, b1, seconds, step=2.0):
    """Linear temperature ramp, marks sec_0, sec_1, ..."""
    n = max(1, int(seconds / step))
    for k in range(n + 1):
        f = k / n
        temps(h0 + (h1 - h0) * f, b0 + (b1 - b0) * f)
        if k < n:
            run('%.3f' % (seconds / n))
def gpiob(tag):                           # PB2 mode and level, via the log
    cmd('echo "@@GPIO %s"' % tag)
    cmd('sysbus ReadDoubleWord 0x40020400')
    cmd('sysbus ReadDoubleWord 0x40020414')
    cmd('echo "@@GPIO_END %s"' % tag)

for m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', m))
cmd('mach create "pw"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
# Filament runout input PA0: pull-up on the board = filament present.
cmd('sysbus.gpioPortA OnGPIO 0 true')
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
for n in (10, 3, 15):                    # endstops X, Y, Z
    cmd('sysbus.gpioPortB OnGPIO %d true' % n)   # endstops released
mark('boot')
run('0.02'); temps(25, 25)                # after .bss init, before protection
run('0.4')
gpiob('boot')

if want('supply'):
    mark('s_m80'); send('M80\n'); run('0.7'); gpiob('m80')
    mark('s_m81'); send('M81\n'); run('0.1'); gpiob('m81')
    # Heating switches the supply on by itself.
    mark('s_heat'); send('M104 S100\n'); run('0.8'); gpiob('heat')
    cmd('echo "@@CCR"'); cmd('sysbus ReadDoubleWord 0x4000083C')
    mark('s_heat_off'); send('M104 S0\n'); run('0.3')
    # Idle for 35 s: steppers off, but without PS_AUTO_OFF the supply stays.
    mark('s_idle'); run('35'); gpiob('idle')
    mark('s_m81b'); send('M81\n'); run('0.1'); gpiob('m81b')

if want('hotend'):
    # Heat to 200 C, switch off. The heater block overshoots by 8 C in the
    # first 8 s, then cools down: no alarm.
    mark('h_heat'); send('M104 S200\n'); run('0.8')
    temps(200, 25); run('2.0')
    mark('h_off'); send('M104 S0\n'); run('0.3')
    ramp('h_over', 200, 208, 25, 25, 8.0)
    ramp('h_cool', 208, 60, 25, 25, 40.0)
    mark('h_cooled'); send('M105\n'); run('0.1')
    # Nozzle parked on a hot bed: +0.33 C/s for 90 s (6.7 C per 20 s).
    ramp('h_slow', 60, 90, 25, 25, 90.0, step=3.0)
    mark('h_slow_done'); send('M105\n'); run('0.1'); gpiob('slow')
    # Shorted MOSFET: +3 C/s, the heater is still "off".
    mark('h_fast')
    ramp('h_fast', 90, 150, 25, 25, 20.0, step=1.0)
    run('10.0')
    gpiob('fast')
    cmd('echo "@@CCMR1"'); cmd('sysbus ReadDoubleWord 0x40014418')   # TIM10
    temps(25, 25)
    # The restart clears RAM, the ADC buffer too: fill it again right after.
    mark('h_m999'); send('M999\n'); run('0.05'); temps(25, 25); run('0.75'); gpiob('m999')
    mark('h_after'); send('M105\n'); run('0.1')

if want('bed'):
    # Bed off from boot (or after M999), then it heats by itself: 0.5 C/s.
    mark('b_idle'); run('62.0')
    mark('b_rise')
    ramp('b_rise', 25, 25, 25, 90, 130.0, step=2.0)
    gpiob('bed')
    mark('b_end'); send('M105\n'); run('0.1')

mark('end')
cmd('quit')

script = artifact_path('power_test_%d.resc') % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
out = re.sub(r'\x1b\[[0-9;]*m', '', run_renode(RENODE, script))
open(artifact_path('power_test.log'), 'w').write(out)

sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def uart(sec):
    return [re.sub(r'^.*\] ', '', l) for l in sections.get(sec, []) if 'usart2: [host' in l]
def uart_range(first, last):
    names = list(sections)
    i, j = names.index(first), names.index(last)
    return [l for n in names[i:j + 1] for l in uart(n)]
def gpio(tag):
    """(MODER bits of PB2, ODR bit 2) or None."""
    m = re.search(r'@+GPIO %s\s*\n(.*?)@+GPIO_END %s' % (tag, tag), out, re.S)
    if not m: return None
    vals = re.findall(r'^\s*(0x[0-9A-Fa-f]+)\s*$', m.group(1), re.M)
    if len(vals) < 2: return None
    moder, odr = int(vals[0], 16), int(vals[1], 16)
    return ((moder >> 4) & 3, (odr >> 2) & 1)
def value(tag):
    m = re.search(r'@+%s\s*\n(?:.*\n)*?\s*(0x[0-9A-Fa-f]+)\s*\n' % tag, out)
    return int(m.group(1), 16) if m else None
OUT_LOW, OUT_HIGH = (1, 0), (1, 1)        # MODER 01 = output

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-50s %s' % ('PASS' if cond else 'FAIL', name, info if not cond else ''))
    if not cond: fails += 1

KILL = 'Error:Heater off but temperature rising, system stopped!'
u = uart('boot')
check('boot', u[:1] == ['start'] and 'ok' in u, u)
check('boot: PB2 output low (supply off)', gpio('boot') == OUT_LOW, gpio('boot'))

if want('supply'):
    print('--- supply ---')
    def ok(sec): return any(l.startswith('ok') for l in uart(sec))
    check('M80: PB2 high, ok', gpio('m80') == OUT_HIGH and ok('s_m80'), (gpio('m80'), uart('s_m80')))
    check('M81: PB2 low, ok', gpio('m81') == OUT_LOW and ok('s_m81'), (gpio('m81'), uart('s_m81')))
    check('M104 switches the supply on', gpio('heat') == OUT_HIGH and (value('CCR') or 0) > 0, (gpio('heat'), value('CCR')))
    check('idle 35 s: supply stays on (no PS_AUTO_OFF)', gpio('idle') == OUT_HIGH, gpio('idle'))
    check('M81 again: PB2 low', gpio('m81b') == OUT_LOW, gpio('m81b'))

if want('hotend'):
    print('--- hotend stuck on ---')
    quiet = uart_range('h_heat', 'h_slow_done')
    check('overshoot after switching off: no alarm', not any('Error' in l for l in quiet),
          [l for l in quiet if 'Error' in l])
    t = [re.findall(r'T:([\d.]+)/', l) for l in uart('h_slow_done') if l.startswith('ok T:')]
    check('slow rise 60 -> 90 C (nozzle on hot bed): no alarm', t and abs(float(t[0][0]) - 90) < 1.5
          and gpio('slow') == OUT_HIGH, (t, gpio('slow')))
    fast = uart('h_fast')
    check('fast rise: halted, Heater_ID: 0', KILL + ' Heater_ID: 0' in fast
          and 'Error:Printer halted. kill() called!' in fast, fast)
    check('fast rise: supply switched off (PB2 low)', gpio('fast') == OUT_LOW, gpio('fast'))
    cc = value('CCMR1')
    check('fast rise: hotend PWM forced inactive', cc is not None and (cc >> 4) & 7 == 4, cc)
    check('M999: restart, supply stays off', 'start' in uart('h_m999') and gpio('m999') == OUT_LOW,
          (uart('h_m999'), gpio('m999')))
    check('after restart: M105 T:25', any(l.startswith('ok T:25.') for l in uart('h_after')), uart('h_after'))

if want('bed'):
    print('--- bed stuck on ---')
    b = uart('b_rise')
    check('bed rises while off: halted, Heater_ID: bed', KILL + ' Heater_ID: bed' in b, b)
    check('bed: supply off (PB2 low)', gpio('bed') == OUT_LOW, gpio('bed'))

print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
