#!/usr/bin/env python3
"""CPU load profile in Renode: samples the program counter periodically
while the P3 Steel build (make CHIP=F401 TEST=0) runs typical workloads,
and reports which functions use the CPU.

Renode executes instructions, it doesn't model Flash wait states, the ART
cache, bus contention or pipeline stalls. The numbers are shares of
executed instructions, a guide to where optimising pays off, not real
cycle counts. Measure real cycles on the board with M9001.

Usage: run_profile.py [renode] [teacup.elf] [platform.repl] [sample_us]

Workloads:
  poly    circle r 20 mm as 360 G1 segments, 60 mm/s, M83 (slicer output)
  arc     the same circle as one G2
  fast    G1 X150 at 150 mm/s (24 kHz step rate)
  retract G1 E-20 at 25 mm/s (41.8 kHz, the highest step rate)
"""
import bisect, math, os, re, subprocess, sys
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')
SAMPLE_US = int(sys.argv[4]) if len(sys.argv) > 4 else 97   # odd: no aliasing with timers

nm = subprocess.run(['arm-none-eabi-nm', '-n', '-S', ELF], capture_output=True, text=True).stdout
syms = []
for l in nm.splitlines():
    p = l.split()
    if len(p) == 4 and p[2] in 'tTwW':
        syms.append((int(p[0], 16) & ~1, int(p[1], 16), p[3]))
syms.sort()
addrs = [s[0] for s in syms]
def func(pc):
    i = bisect.bisect_right(addrs, pc) - 1
    if i >= 0 and pc < syms[i][0] + max(syms[i][1], 2):
        return syms[i][2]
    return '?%08x' % pc
ADCBUF = int(re.search(r'^([0-9a-f]+) \S+ \S adc_buffer$', nm, re.M).group(1), 16)

lines = []
def cmd(c): lines.append(c)
def send(text, dev='usart2'):
    for ch in text:
        cmd('sysbus.%s WriteChar 0x%02X' % (dev, ord(ch)))
def run(t): cmd('emulation RunFor "%s"' % t)
def sample(name, seconds):
    cmd('echo "@@PROF %s"' % name)
    for _ in range(int(seconds * 1e6 / SAMPLE_US)):
        cmd('emulation RunFor "0.%06d"' % SAMPLE_US)
        cmd('sysbus.cpu PC')
    cmd('echo "@@PROF_END"')

for m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', m))
cmd('mach create "prof"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
# Filament runout input PA0: pull-up on the board = filament present.
cmd('sysbus.gpioPortA OnGPIO 0 true')
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('showAnalyzer sysbus.usb Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2'); cmd('logLevel -1 sysbus.usb')
for n in (10, 3, 15):                    # endstops X, Y, Z
    cmd('sysbus.gpioPortB OnGPIO %d true' % n)
run('0.02')
for i in range(32):
    cmd('sysbus WriteWord 0x%08X 3911' % (ADCBUF + 2 * i))       # 25 C
run('0.4')
send('M80\nG92 X100 Y100 Z5 E0\nM83\n'); run('0.8')
# The segment list goes over USB: flow control (NAK) instead of UART overrun,
# like a host waiting for "ok".

sample('idle', 0.3)

# All workloads over USB, one after the other: flow control keeps the order
# and nothing gets lost. Each ends with M9001 (step interrupt statistics).
U = 'usb'
pts = []
for k in range(1, 361):
    a = math.radians(k)
    pts.append('G1 X%.3f Y%.3f E0.0117 F3600' % (80 + 20 * math.cos(a), 100 + 20 * math.sin(a)))
send('G1 X100 Y100 F3600\nM400\nM9001 R\n', U); run('0.3')
send('\n'.join(pts) + '\nM400\nM9001\n', U)
sample('poly', 2.6)
run('0.2')

send('M9001 R\nG2 X100 Y100 I-20 J0 E4.2 F3600\nM400\nM9001\n', U)
sample('arc', 2.6)
run('0.2')

send('G1 X0 F9000\nM400\nM9001 R\n', U); run('1.3')
send('G1 X150 F9000\nM400\nM9001\n', U)
sample('fast', 1.3)
run('0.2')

send('M9001 R\nG1 E-20 F1500\nM400\nM9001\n', U)
sample('retract', 1.1)
run('0.3')
cmd('quit')

if os.environ.get('PROFILE_LOG'):             # evaluate an earlier run only
    out = open(os.environ['PROFILE_LOG']).read()
else:
    script = '/tmp/profile_%d.resc' % os.getpid()
    open(script, 'w').write('\n'.join(lines) + '\n')
    proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = re.sub(r'\x1b\[[0-9;]*m', '', proc.stdout.read()); proc.wait()
    open('/tmp/profile.log', 'w').write(out)

# Group functions into parts of the firmware.
GROUPS = [
    ('step ISR', r'^(TIM5_IRQHandler|queue_step|dda_step|dda_start|timer_set|timer_step_pulse_end|unstep|TIM[0-9]+_IRQHandler)$'),
    ('dda_clock (PendSV)', r'^(PendSV_Handler|dda_clock|update_current_position)$'),
    ('planner', r'^(dda_create|dda_find_crossing_speed|dda_join_moves|dda_new_startpoint|enqueue_home|approx_distance.*|int_sqrt|int_f_sqrt|int_inv_sqrt|muldiv.*|acc_ramp_len|dda_.*|__aeabi_.*div.*|__udivmoddi4|__divdi3|__udivdi3)$'),
    ('arc math', r'^(arc_move|apply_soft_limits|sinf|cosf|atan2f|__ieee754.*|__kernel.*|floorf|ceilf|sqrtf)$'),
    ('G-code parse/execute', r'^(gcode_parse_char|process_gcode_command|decfloat_to_int|finish_line|serwrite.*|sersendf.*|write_.*|temp_wait|restore_axis_word|seen_axes)$'),
    ('USB / UART I/O', r'^(serial_writechar|serial_writestr|uart_writechar|uart_popchar|usb_cdc_writechar|usb_cdc_popchar|usb_irq|OTG_FS_IRQHandler|rx_.*|tx_.*|ep0_.*|ep_write_packet|USART.*|DMA.*|usb_poll|wait_mode)$'),
    ('temp / heaters', r'^(temp_.*|heater_.*|thermal_.*|run_pid_loop|pid_.*|ADC_IRQHandler|analog_.*|soft_pwm_tick|check_heater_off)$'),
    ('idle polling', r'^(main|clock_poll|clock_10ms|clock_250ms|queue_full|queue_wait|gcode_queue_read_serial|read_port|gcode_queue_execute|serial_port_rxchars|uart_rxchars|usb_cdc_rxchars|rx_count|emergency_poll|SysTick_Handler|wd_reset|steppers_idle_tick|power_idle|delay_.*|serial_rx_poll)$'),
]
def group(fn):
    for g, rx in GROUPS:
        if re.match(rx, fn):
            return g
    return 'other'

res = {}
cur = None
for l in out.splitlines():
    m = re.search(r'@@PROF (\w+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); res[cur] = Counter(); continue
    if '@@PROF_END' in l and 'echo' not in l:
        cur = None; continue
    if cur:
        m = re.match(r'^\s*(0x[0-9A-Fa-f]+)\s*$', l)
        if m:
            res[cur][func(int(m.group(1), 16))] += 1

stats = re.findall(r'echo:Step IRQ.*', out)
for name, c in res.items():
    n = sum(c.values())
    g = defaultdict(int)
    for f, k in c.items():
        g[group(f)] += k
    print('=== %s: %d samples' % (name, n))
    for gname, k in sorted(g.items(), key=lambda x: -x[1]):
        print('  %-20s %5.1f %%' % (gname, 100.0 * k / n))
    print('  top functions: ' + ', '.join('%s %.1f%%' % (f, 100.0 * k / n) for f, k in c.most_common(8)))
print('M9001 reports (poly, arc, fast X150, retract), Renode: cycles = instructions:')
for s in stats:
    print('  ' + s)
