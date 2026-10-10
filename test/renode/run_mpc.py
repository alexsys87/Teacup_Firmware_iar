#!/usr/bin/env python3
"""Renode test of the model predictive hotend control (HOTEND_MPC, M306) on
the test build with the simulated hotend of the dummy sensor (src/temp.c,
temp_dummy_plant: 4 C/s at full power, time constant 40 s towards 25 C,
1 s dead time; part fan temp_dummy_fan_loss / 1000 C/s extra):

  make CHIP=F401 TEST=1 EXTRA="-DHOTEND_MPC" BUILD_SUFFIX=_mpc
  run_mpc.py [renode] [teacup.elf]

With P = 40 W the plant is C = 40 / 4 = 10 J/K, A = C / 40 = 0.25 W/K,
the fan (0.6 C/s at full speed at 150 C) F = A + 0.6 * C / 125 = 0.298.
(1.2 C/s would need 43 W at 150 C: more than the heater has.)

Checks:
  - M306 reports the defaults
  - M306 T S150: cooling, heating, holding; C, A, F within 20 %
  - the model holds 150 C (+-1)
  - part fan on at full speed: the temperature drops less than 2 C
  - M500 / M502 / M501 store and load M306
"""
import os, re, subprocess, sys

from renode_common import artifact_path, monitor_command, run_renode

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_1_mpc/teacup.elf')
REPL = os.path.join(HERE, 'stm32f401_dma.repl')

_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
def sym(name):
    return int(re.search(r'^([0-9a-f]+) \S %s$' % name, _nm, re.M).group(1), 16)
PLANT = sym('temp_dummy_plant')
FORCE = sym('temp_dummy_force')
FANLOSS = sym('temp_dummy_fan_loss')

lines = []
def cmd(c): lines.append(monitor_command(c))
def mark(name): cmd('echo "@@MARK %s"' % name)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))

for _m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', _m))
cmd('mach create "mpc"')
cmd('machine LoadPlatformDescription @%s' % REPL)
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
cmd('sysbus.gpioPortB OnGPIO 12 true')
run('0.3')
# The dummy sensor starts at 0 C: 25 C first, the plant starts from there.
cmd('sysbus WriteDoubleWord 0x%08X 100' % FORCE); run('0.5')
cmd('sysbus WriteDoubleWord 0x%08X 0xFFFFFFFF' % FORCE)
cmd('sysbus WriteDoubleWord 0x%08X 1' % PLANT)
mark('def'); send('M306\n'); run('0.1')
# The fan loss of the plant is constant, not proportional to the
# temperature: during the cooling phase (fan on at 25 C) it would cool
# below ambient. So only once the heating has started.
mark('tune'); send('M306 P40 T S150\n'); run('15.0')
cmd('sysbus WriteDoubleWord 0x%08X 600' % FANLOSS)
for k in range(23):
    run('10.0')
mark('tuned'); send('M306\n'); run('0.1')
# The hotend cooled down after the autotune; full power needs ~55 s from
# 47 C to 150 C with this plant, then settle.
mark('hold'); send('M104 S150\n')
for k in range(9):
    run('10.0')
for k in range(10):
    mark('hold_%d' % k); send('M105\n'); run('1.0')
mark('fan'); send('M106 S255\n')
for k in range(30):
    mark('fan_%d' % k); send('M105\n'); run('0.5')
send('M107\nM104 S0\n'); run('0.1')
cmd('sysbus WriteDoubleWord 0x%08X 0' % PLANT)
mark('st'); send('M500\nM502\nM306\nM501\nM306\n'); run('0.3')
mark('end')
cmd('quit')

script = artifact_path('mpc_test_%d.resc') % os.getpid()
open(script, 'w').write('\n'.join(lines) + '\n')
# stdin stays open: at EOF on stdin Renode's console queues input events
# without end and runs out of memory.
out = re.sub(r'\x1b\[[0-9;]*m', '', run_renode(RENODE, script))
open(artifact_path('mpc_test.log'), 'w').write(out)
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
def temp(sec):
    for l in uart(sec):
        m = re.search(r'T:([\d.]+)', l)
        if m: return float(m.group(1))
def m306(sec):
    for l in uart(sec):
        m = re.match(r'M306 P([\d.]+) C([\d.]+) R([\d.]+) A([\d.]+) F([\d.]+) H([\d.]+)', l)
        if m: return [float(v) for v in m.groups()]

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-52s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1

check('M306 defaults', uart('def')[:1] == ['M306 P40.00 C16.70 R0.2200 A0.0680 F0.0970 H0.0056'], uart('def'))
tune = [l for l in uart('tune') if 'MPC' in l or l.startswith('M306')]
p = m306('tune')
check('M306 T S150 finishes', any('MPC autotune finished' in l for l in tune), tune)
near = lambda v, e, tol: v is not None and abs(v - e) <= tol * e
check('  C = 10 J/K (+-20 %)', p is not None and near(p[1], 10.0, 0.2), p)
check('  A = 0.25 W/K (+-20 %)', p is not None and near(p[3], 0.25, 0.2), p)
check('  F = 0.298 W/K (+-20 %)', p is not None and near(p[4], 0.298, 0.2), p)
check('  R plausible (0.2..5 1/s)', p is not None and 0.2 <= p[2] <= 5, p)
hold = [temp('hold_%d' % k) for k in range(10)]
check('holds 150 C (+-1)', all(t is not None and abs(t - 150) <= 1.0 for t in hold), hold)
fan = [temp('fan_%d' % k) for k in range(30)]
check('part fan at full speed: drop < 2 C', all(t is not None and t >= 148.0 for t in fan) and
      all(t is not None and abs(t - 150) <= 1.0 for t in fan[-6:]), fan)
st = [l for l in uart('st') if l.startswith('M306')]
check('M500 / M502 / M501', len(st) == 2 and st[0] == 'M306 P40.00 C16.70 R0.2200 A0.0680 F0.0970 H0.0056'
      and p is not None and st[1] == [l for l in uart('tuned') if l.startswith('M306')][0], st)
stream = [re.sub(r'^.*\] ', '', l) for l in out.splitlines() if 'usart2: [host' in l]
check('no error', not any(l.startswith('Error') for l in stream), [l for l in stream if 'rror' in l])
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
