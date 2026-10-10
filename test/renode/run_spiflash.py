#!/usr/bin/env python3
"""Renode test of the SPI flash support with the P3 Steel configuration
(board.blackpill_p3steel.h: SPI_FLASH), a W25Q64 model on SPI1/PA4
(models/TeacupW25Q.cs): detection, settings in SPI flash (also during a
move), G-code upload M28/M29, M20, printing with M23/M24, M30, M9002.

Usage: run_spiflash.py [renode] [teacup.elf] [platform.repl]
"""
import math, os, re, subprocess, sys

from renode_common import artifact_path, monitor_command, run_renode

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma_flash.repl')

# Thermistor readings for 25 C (see run_p3steel.py), else MAXTEMP triggers.
RP, PTS = 4700.0, [(25.0, 100000.0), (150.0, 1641.9), (250.0, 226.15)]
def room_adc():
    (t1, r1), (t2, r2), (t3, r3) = PTS
    l1, l2, l3 = map(math.log, (r1, r2, r3))
    y1, y2, y3 = (1 / (t + 273.15) for t in (t1, t2, t3))
    g2, g3 = (y2 - y1) / (l2 - l1), (y3 - y1) / (l3 - l1)
    c = (g3 - g2) / (l3 - l2) / (l1 + l2 + l3); b = g2 - c * (l1*l1 + l1*l2 + l2*l2); a = y1 - (b + l1*l1*c) * l1
    f = lambda adc: 1 / (a + b * math.log(RP * adc / (4095 - adc)) + c * math.log(RP * adc / (4095 - adc)) ** 3) - 273.15
    lo, hi = 1, 4094
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if f(mid) > 25.0: lo = mid
        else: hi = mid
    return hi
_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
ADCBUF = int(re.search(r'^([0-9a-f]+) \S adc_buffer$', _nm, re.M).group(1), 16)

lines = []
def cmd(c): lines.append(monitor_command(c))
def mark(n): cmd('echo "@@MARK %s"' % n)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(t):
    for ch in t: cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def cs(t):
    c = 0
    for ch in t: c ^= ord(ch)
    return '%s*%d' % (t, c)
def read32(tag, addr): cmd('echo "@@%s"' % tag); cmd('sysbus ReadDoubleWord 0x%08X' % addr)

for m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupW25Q.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', m))
cmd('mach create "fl"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
# Filament runout input PA0: pull-up on the board = filament present.
cmd('sysbus.gpioPortA OnGPIO 0 true')
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
for port, n in (('B', 10), ('B', 3), ('B', 15)):
    cmd('sysbus.gpioPort%s OnGPIO %d true' % (port, n))
ROOM = room_adc()
def adc():
    for i in range(32):
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * i, ROOM))
def boot(name):
    mark(name); run('0.02'); adc(); run('0.4')
def reset(name):
    send('M112\n'); run('0.1'); mark(name); send('M999\n'); run('0.02'); adc(); run('0.4')

boot('boot')
# Supply on first (PS_ON waits 500 ms for it), else the move starts late.
send('M80\n'); run('0.7')
# Settings in SPI flash, stored while X moves (no queue wait, no stall).
send('G92 X0 Y0 Z0 E0\n'); run('0.05')
mark('move'); send('M92 X100\nG1 X50 F3000\n'); run('0.3')
mark('m500'); send('M500\nM114\n'); run('0.2')
run('2.5')
mark('after_move'); send('M114\n'); run('0.05')
read32('INTERNAL', 0x08020000)
reset('reset1')
mark('m503'); send('M503\n'); run('0.1')

# Upload like Pronterface: numbered lines with checksums, > 4 kB (2 sectors).
# 100 short moves, text lines in between to get past one 4 kB sector.
FILE = ['G92 X0 Y0 Z0 E0']
for i in range(100):
    FILE.append('G1 X%d Y%.1f F6000' % (i % 2, (i % 3) * 0.5))
    FILE.append('M117 line %03d padding the file' % i)
FILE.append('M117 From flash')
mark('upload'); send(cs('N-1 M110') + '\n'); run('0.02')
n = 0
send(cs('N%d M28 TEST.GCO' % n) + '\n'); run('0.1'); n += 1
for l in FILE:
    send(cs('N%d %s' % (n, l)) + '\n'); run('0.01'); n += 1
send(cs('N%d M29' % n) + '\n'); run('0.1')
mark('m20'); send('M20\n'); run('0.1')
mark('print'); send('M23 TEST.GCO\nM24\n'); run('2.0')
mark('m27'); send('M27\n'); run('14.0')
mark('m27_end'); send('M27\nM114\n'); run('0.1')
mark('m30'); send('M30 TEST.GCO\nM20\n'); run('0.2')
mark('m9002'); send('M28 B.GCO\nG1 X1\nM29\nM9002\nM20\n'); run('0.5')
reset('reset2')
mark('m503b'); send('M503\n'); run('0.1')
mark('end')
cmd('quit')

script = artifact_path('spiflash_test.resc')
open(script, 'w').write('\n'.join(lines) + '\n')
out = re.sub(r'\x1b\[[0-9;]*m', '', run_renode(RENODE, script))
open(artifact_path('spiflash_test.log'), 'w').write(out)
sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def uart(sec):
    r = [re.sub(r'^.*\] ', '', l) for l in sections.get(sec, []) if 'usart2: [host' in l]
    return ['ok' if re.match(r'^ok( N-?\d+)? P\d+ B\d+$', l) else l for l in r]
def pos(sec):
    for l in uart(sec):
        m = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+) E:(-?[\d.]+) Count', l)
        if m: return tuple(float(v) for v in m.groups())
fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-44s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1

u = uart('boot')
check('chip detected (8 MB)', 'echo:SPI flash: manufacturer 0xEF type 0x40, 8192 kB' in u, u)
check('settings in SPI flash, none stored yet', 'echo:Settings in SPI flash' in u and 'echo:No stored settings, using defaults' in u, '')
um = uart('m500')
pm = pos('m500')
check('M500 during a move, move continues', any(l.startswith('echo:Settings Stored') for l in um)
      and pm is not None and 5 < pm[0] < 45, (um[:2], pm))
check('move completes (X50)', pos('after_move') is not None and pos('after_move')[0] == 50.0, pos('after_move'))
mi = re.search(r'@+INTERNAL\s*\n\s*(0x[0-9A-Fa-f]+)', out)
check('internal Flash sector untouched', mi is not None and int(mi.group(1), 16) != 0x53505443, mi.group(1) if mi else None)
check('after reset: loaded from SPI flash', any('Stored settings retrieved' in l for l in uart('reset1'))
      and any(l.startswith('echo:  M92 X100.00') for l in uart('m503')), [l for l in uart('m503') if 'M92' in l])
uu = uart('upload')
size = sum(len(l) + 1 for l in FILE)
check('M28 / M29', 'Writing to file: TEST.GCO' in uu and 'Done saving file.' in uu and not any(l.startswith('Error') for l in uu),
      [l for l in uu if not l.startswith('ok')][:5])
check('M20 lists the file', 'TEST.GCO %d' % size in uart('m20'), uart('m20'))
up = uart('print')
check('M23 opens', 'File opened: TEST.GCO Size: %d' % size in up and 'File selected' in up, up[:4])
m27 = [l for l in uart('m27') if 'SD printing byte' in l]
check('M27 during print', bool(m27) and 0 < int(m27[0].split()[3].split('/')[0]) < size, m27)
ue = uart('m27_end')
check('print done: SD file done, Not SD printing', 'Not SD printing' in ue and any('SD file done' in l for l in uart('m27')), ue)
last = 99
check('position from the file (X%d Y%.1f)' % (last % 2, (last % 3) * 0.5),
      pos('m27_end') is not None and pos('m27_end')[:2] == (float(last % 2), (last % 3) * 0.5), pos('m27_end'))
u30 = uart('m30')
check('M30 deletes, list empty', 'File deleted:TEST.GCO' in u30 and 'Begin file list' in u30
      and u30.index('End file list') == u30.index('Begin file list') + 1, u30)
u92 = uart('m9002')
check('M9002 deletes all', 'echo:Flash files deleted' in u92 and u92.index('End file list') == u92.index('Begin file list') + 1, u92)
check('settings survive M9002 + reset', any(l.startswith('echo:  M92 X100.00') for l in uart('m503b')), '')
check('no errors', 'Error:' not in out.replace('Error:Printer halted. kill() called!', '').replace('Error:Emergency stop (M112)', ''), '')
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
