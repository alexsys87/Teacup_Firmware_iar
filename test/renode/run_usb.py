#!/usr/bin/env python3
"""Renode test of the USB CDC host port and of the UART + USB combination.

Needs the test build (make CHIP=F401 TEST=1) and the OTG_FS model
models/TeacupSTM32_OTGFS.cs, which also plays the USB host.

Usage: run_usb.py [renode] [teacup.elf] [platform.repl]

Checks:
  - enumeration: descriptors, strings (serial number from the chip UID),
    STALL of unsupported requests, line coding, no protocol violations
  - routing: answers go to the port the command came from, independent
    line numbers per port, Resend only to the port with the bad line
  - emergency parser across ports (M109 from UART, M108 from USB), M112 on
    USB halts and reports on both ports, M999 over USB in the halted state
    (polled with interrupts off), re-enumeration after the restart
  - flow control: a burst of 80 lines over USB, NAKs but no lost line
  - host doesn't read / port closed / cable unplugged: the printer keeps
    running, the UART keeps working, no watchdog reset
  - endpoint halt (SET_FEATURE / CLEAR_FEATURE), unknown class request
"""
import os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_1/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')

lines = []
def cmd(c): lines.append(c)
def mark(name): cmd('echo "@@MARK %s"' % name)
def run(t): cmd('emulation RunFor "%s"' % t)
def send_uart(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def send_usb(text):
    for ch in text:
        cmd('sysbus.usb WriteChar 0x%02X' % ord(ch))
def cs(text):
    c = 0
    for ch in text:
        c ^= ord(ch)
    return '%s*%d' % (text, c)

for m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', m))
cmd('mach create "teacup"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
# Unique ID, the serial number string is made from it.
cmd('sysbus WriteDoubleWord 0x1FFF7A10 0x33221100')
cmd('sysbus WriteDoubleWord 0x1FFF7A14 0x77665544')
cmd('sysbus WriteDoubleWord 0x1FFF7A18 0xBBAA9988')
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('showAnalyzer sysbus.usb Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3')
cmd('logLevel -1 sysbus.usart2')
cmd('logLevel -1 sysbus.usb')
cmd('sysbus.gpioPortB OnGPIO 12 true')         # X min endstop released
mark('boot'); run('0.3')

# ---- routing ----
mark('r_usb'); send_usb('M115\n'); run('0.05')
mark('r_uart'); send_uart('M105\n'); run('0.05')

# Both hosts with their own line numbers, interleaved.
mark('r_both')
send_uart(cs('N0 M110 N0') + '\n'); send_usb(cs('N0 M110 N0') + '\n'); run('0.02')
for n in range(1, 6):
    send_uart(cs('N%d G1 X%d F6000' % (n, n)) + '\n')
    send_usb(cs('N%d M105' % n) + '\n')
    run('0.01')
run('0.5')
mark('r_resend'); send_usb('N6 M105*1\n'); run('0.05')
send_usb(cs('N6 M105') + '\n'); run('0.05')

# ---- emergency parser across ports ----
mark('e_m109'); send_uart('M109 S200\n'); run('1.0')
mark('e_m108'); send_usb('M108\n'); run('1.2')
send_uart('M104 S0\n'); run('0.1')
mark('e_m112'); send_usb('M112\n'); run('0.2')
mark('e_m999'); send_usb('M999\n'); run('0.6')     # polled USB, interrupts off
mark('e_after'); send_usb('M105\n'); run('0.05')
cmd('sysbus.usb Stats')

# ---- flow control: 80 lines at once ----
mark('f_burst')
send_usb(''.join('G1 X%d Y%d F6000\n' % (i % 2, (i // 2) % 2) for i in range(80)))
run('8.0')
mark('f_pos'); send_usb('M114\n'); run('0.05')
cmd('sysbus.usb Stats')

# ---- host doesn't read ----
cmd('sysbus.usb HostReads false')
mark('n_noread'); send_usb('M503\nM503\nM503\nM503\nM105\n'); run('0.8')
mark('n_uart'); send_uart('M105\n'); run('0.1')
cmd('sysbus.usb HostReads true')
mark('n_reads'); run('0.1'); send_usb('M115\n'); run('0.2')

# ---- port closed (DTR 0): output is kept, arrives after opening ----
cmd('sysbus.usb Close'); run('0.01')
mark('c_closed'); send_usb('M115\n'); run('0.2')
mark('c_uart'); send_uart('M105\n'); run('0.05')
mark('c_open'); cmd('sysbus.usb Open'); run('0.1')

# ---- cable unplugged ----
cmd('sysbus.usb Unplug')
mark('u_unplugged'); send_uart('M503\nG1 X5 F6000\nM400\nM114\n'); run('1.0')
cmd('sysbus.usb Plug')
mark('u_plugged'); run('0.3'); send_usb('M105\n'); run('0.05')
cmd('sysbus.usb Stats')

# ---- control requests ----
mark('x_ctl')
cmd('sysbus.usb Control "BAD_CLASS" "21 7F 00 00 00 00 00 00" 0')
cmd('sysbus.usb Control "GET_LINE_CODING_2" "A1 21 00 00 00 00 07 00" 7')
cmd('sysbus.usb Control "SET_FEATURE HALT 81" "02 03 00 00 81 00 00 00" 0')
cmd('sysbus.usb Control "GET_STATUS EP81 halted" "82 00 00 00 81 00 02 00" 2')
cmd('sysbus.usb Control "CLEAR_FEATURE HALT 81" "02 01 00 00 81 00 00 00" 0')
cmd('sysbus.usb Control "GET_STATUS EP81 running" "82 00 00 00 81 00 02 00" 2')
cmd('sysbus.usb Control "GET_STATUS EP85 bad" "82 00 00 00 85 00 02 00" 2')
run('0.05')
mark('x_after'); send_usb('M105\n'); run('0.05')

# ---- halted by UART, M999 over UART, USB comes back ----
mark('k_m112'); send_uart('M112\n'); run('0.1')
mark('k_m999'); send_uart('M999\n'); run('0.6')
mark('k_after'); send_usb('M105\n'); run('0.05')
cmd('sysbus.usb Stats')
mark('end')
cmd('quit')

script = os.path.join('/tmp', 'teacup_usb_%d.resc' % os.getpid())
open(script, 'w').write('\n'.join(lines) + '\n')
proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script],
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, text=True)
out = proc.stdout.read()
proc.wait()
out = re.sub(r'\x1b\[[0-9;]*m', '', out)
open('/tmp/teacup_usb.log', 'w').write(out)

sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)

def port(sec, dev):
    return [re.sub(r'^.*\] ', '', l) for l in sections.get(sec, []) if '%s: [host' % dev in l]
def usb(sec): return port(sec, 'usb')
def uart(sec): return port(sec, 'usart2')
def host(sec=None):
    src = sections.get(sec, []) if sec else out.splitlines()
    res = {}
    for l in src:
        m = re.search(r'usb: HOST (.+?) -> (\S+) ?(.*)$', l)
        if m:
            res.setdefault(m.group(1), []).append((m.group(2), bytes.fromhex(m.group(3).replace(' ', '')) if m.group(2) == 'OK' else b''))
    return res
def stats():
    return [dict((k, int(v)) for k, v in re.findall(r'(\w+)=(\d+)', l)) for l in out.splitlines() if 'usb: STATS' in l]
def oks(lst): return [l for l in lst if l.startswith('ok')]

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-44s %s' % ('PASS' if cond else 'FAIL', name, info if not cond else ''))
    if not cond: fails += 1

print('--- enumeration ---')
h = host('boot')
dev = h.get('GET_DESCRIPTOR DEVICE', [('?', b'')])[0][1]
check('device descriptor', len(dev) == 18 and dev[2:4] == b'\x00\x02' and dev[4:7] == b'\x02\x02\x00'
      and dev[7] == 64 and dev[8:12] == bytes([0x83, 0x04, 0x40, 0x57]) and dev[17] == 1, dev.hex(' '))
cfg = h.get('GET_DESCRIPTOR CONFIG', [('?', b'')])[0][1]
def descs(b):
    i, r = 0, []
    while i + 1 < len(b) and b[i] >= 2:
        r.append(b[i:i + b[i]]); i += b[i]
    return r
d = descs(cfg)
ifs = [x for x in d if x[1] == 4]
eps = [x for x in d if x[1] == 5]
check('config descriptor: 67 bytes, parses', len(cfg) == 67 and cfg[2] == 67 and sum(len(x) for x in d) == 67, cfg.hex(' '))
check('interfaces: CDC ACM + data', len(ifs) == 2 and ifs[0][5:8] == b'\x02\x02\x00' and ifs[1][5] == 0x0A, [x.hex() for x in ifs])
check('endpoints: 0x82 int, 0x01/0x81 bulk 64', sorted((x[2], x[3], x[4]) for x in eps) == [(0x01, 2, 64), (0x81, 2, 64), (0x82, 3, 16)],
      [x.hex() for x in eps])
def sdesc(name):
    v = h.get(name, [('?', b'')])[0]
    return v[1][2:].decode('utf-16-le') if v[0] == 'OK' else None
check('string: product', sdesc('GET_DESCRIPTOR STRING 2') == 'Teacup 3D printer', sdesc('GET_DESCRIPTOR STRING 2'))
check('string: manufacturer', sdesc('GET_DESCRIPTOR STRING 1') == 'Teacup', sdesc('GET_DESCRIPTOR STRING 1'))
check('string: serial number from UID', sdesc('GET_DESCRIPTOR STRING 3') == '00112233445566778899AABB', sdesc('GET_DESCRIPTOR STRING 3'))
check('device qualifier: STALL (full speed only)', h.get('GET_DESCRIPTOR DEVICE_QUALIFIER', [('?',)])[0][0] == 'DATA', h.get('GET_DESCRIPTOR DEVICE_QUALIFIER'))
check('request after STALL works', h.get('GET_STATUS DEVICE', [('?', b'')])[0] == ('OK', b'\x00\x00'))
check('SET_CONFIGURATION / GET_CONFIGURATION', h.get('SET_CONFIGURATION 1', [('?',)])[0][0] == 'OK'
      and h.get('GET_CONFIGURATION', [('?', b'')])[0] == ('OK', b'\x01'))
check('line coding round trip', h.get('GET_LINE_CODING', [('?', b'')])[0] == ('OK', bytes([0x00, 0xC2, 0x01, 0, 0, 0, 8])))
u = uart('boot')
check('boot messages on the UART', u[:1] == ['start'] and 'ok' in u, u)
prot = [l for l in out.splitlines() if 'usb: PROTOCOL' in l]
check('no USB protocol violations', prot == [], prot[:5])

print('--- routing ---')
check('M115 over USB: answer on USB', any('FIRMWARE_NAME:Teacup' in l for l in usb('r_usb')) and oks(usb('r_usb')), usb('r_usb'))
check('M115 over USB: nothing on UART', uart('r_usb') == [], uart('r_usb'))
check('M105 over UART: answer on UART only', uart('r_uart') == ['ok T:25.0/0.0 @:0'] and usb('r_uart') == [], (uart('r_uart'), usb('r_uart')))
ub, uu = oks(uart('r_both')), oks(usb('r_both'))
nb = [int(re.match(r'ok N(\d+)', l).group(1)) for l in ub if re.match(r'ok N\d+', l)]
nu = [int(re.match(r'ok N(\d+)', l).group(1)) if re.match(r'ok N\d+', l) else -1 for l in uu]
check('own line numbers: UART N0..N5 ok', nb == [0, 1, 2, 3, 4, 5], ub)
check('own line numbers: USB N0..N5 ok', len(uu) == 6 and 'Error' not in ' '.join(usb('r_both')), usb('r_both'))
check('no Resend while interleaved', not any('Resend' in l for l in uart('r_both') + usb('r_both')))
r = usb('r_resend')
check('bad checksum on USB: Resend on USB', 'Error:checksum mismatch, Last Line: 5' in r and 'Resend: 6' in r, r)
check('bad checksum on USB: nothing on UART', uart('r_resend') == [], uart('r_resend'))

print('--- emergency parser / halt ---')
check('M109 from UART waits', 'ok' not in uart('e_m109'), uart('e_m109'))
check('M108 from USB cancels it (UART answers)', 'echo:Wait for temperature cancelled' in uart('e_m108') and oks(uart('e_m108')),
      (uart('e_m108'), usb('e_m108')))
for dev_name, f in (('USB', usb), ('UART', uart)):
    check('M112 from USB: halt reported on %s' % dev_name, 'Error:Printer halted. kill() called!' in f('e_m112'), f('e_m112'))
check('M999 over USB while halted: restart', 'echo:Restarting' in usb('e_m999') and 'start' in uart('e_m999'),
      (usb('e_m999'), uart('e_m999')))
check('USB after restart: M105 answered', any(l.startswith('ok T:') for l in usb('e_after')), usb('e_after'))
st = stats()
check('re-enumerated after restart', len(st) >= 1 and st[0].get('enumerations') == 2, st[:1])

print('--- flow control ---')
fb = usb('f_burst')
check('80 lines over USB: 80 x ok', len(oks(fb)) == 80, len(oks(fb)))
check('host was held off with NAK', len(st) >= 2 and st[1]['bulk_out_naks'] > 0 and st[1]['pending_tx'] == 0, st[1:2])
p = [l for l in usb('f_pos') if l.startswith('X:')]
check('end position after burst X1 Y1', p and p[0].startswith('X:1.000 Y:1.000'), p)

print('--- host not reading / closed / unplugged ---')
check('UART answers while USB host does not read', 'ok T:25.0/0.0 @:0' in uart('n_uart'), uart('n_uart'))
check('host reads again: USB answers', any('FIRMWARE_NAME' in l for l in usb('n_reads')), usb('n_reads')[-3:])
check('port closed: nothing arrives', usb('c_closed') == [], usb('c_closed'))
check('port closed: UART works', 'ok T:25.0/0.0 @:0' in uart('c_uart'), uart('c_uart'))
check('port opened: buffered answer arrives', any('FIRMWARE_NAME' in l for l in usb('c_open')), usb('c_open'))
uu = uart('u_unplugged')
check('unplugged: UART commands run', any(l.startswith('X:5.000') for l in uu) and len(oks(uu)) >= 4, uu[-4:])
check('plugged again: USB answers', any(l.startswith('ok T:') for l in usb('u_plugged')), usb('u_plugged'))
check('re-enumerated after plug', len(st) >= 3 and st[2]['enumerations'] == 3, st[2:3])

print('--- control requests ---')
h = host('x_ctl')
check('unknown class request: STALL', h.get('BAD_CLASS', [('?',)])[0][0] != 'OK', h.get('BAD_CLASS'))
check('next request works', h.get('GET_LINE_CODING_2', [('?', b'')])[0][0] == 'OK', h.get('GET_LINE_CODING_2'))
check('SET_FEATURE halt -> GET_STATUS 1', h.get('GET_STATUS EP81 halted', [('?', b'')])[0] == ('OK', b'\x01\x00'), h.get('GET_STATUS EP81 halted'))
check('CLEAR_FEATURE halt -> GET_STATUS 0', h.get('GET_STATUS EP81 running', [('?', b'')])[0] == ('OK', b'\x00\x00'), h.get('GET_STATUS EP81 running'))
check('GET_STATUS of a missing endpoint: STALL', h.get('GET_STATUS EP85 bad', [('?',)])[0][0] != 'OK', h.get('GET_STATUS EP85 bad'))
check('data after halt cleared', any(l.startswith('ok T:') for l in usb('x_after')), usb('x_after'))

print('--- halt from UART, M999 over UART ---')
check('M112 from UART: reported on USB too', 'Error:Printer halted. kill() called!' in usb('k_m112'), usb('k_m112'))
check('M999 over UART: restart', 'start' in uart('k_m999'), uart('k_m999'))
check('USB back after restart', any(l.startswith('ok T:') for l in usb('k_after')), usb('k_after'))
first = out[:out.find('@@MARK e_m999')]
rest = out[out.find('@@MARK e_after'):out.find('@@MARK k_m999')]
check('no unexpected reset (watchdog/fault)', first.count('usart2: [host') and first.count('] start') == 1 and rest.count('] start') == 0,
      (first.count('] start'), rest.count('] start')))
check('no USB protocol violations (whole run)', not any('usb: PROTOCOL' in l for l in out.splitlines()))

print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
