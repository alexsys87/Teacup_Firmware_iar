#!/usr/bin/env python3
"""Drive the firmware in Renode with Pronterface's own host code (printcore
from Printrun), over a TCP socket instead of a serial port.

Prints the PrusaSlicer sample from run_tests.py (G28 replaced by G92, there
is nobody to press the endstops), with Pronterface-style manual commands and
pause/resume in between.

Needs Printrun's printrun/ package on PYTHONPATH (printcore.py, device.py,
gcoder.py, utils.py, plugins/) and pyserial.

Usage: run_printcore.py <renode> <teacup.elf> <platform.repl>
  TEACUP_PORT=usb connects printcore to the USB CDC port (the USB host in
  models/TeacupSTM32_OTGFS.cs forwards the socket) instead of the UART.
"""
import logging, os, re, socket, subprocess, sys, time
logging.disable(logging.WARNING)             # printcore deprecation noise

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE, ELF, REPL = sys.argv[1], os.path.abspath(sys.argv[2]), os.path.abspath(sys.argv[3])
PORT = 3456
HOSTDEV = 'usb' if os.environ.get('TEACUP_PORT', 'uart') == 'usb' else 'usart2'

from printrun.printcore import printcore
from printrun import gcoder

# The PrusaSlicer sample, shared with run_tests.py.
src = open(os.path.join(HERE, 'run_tests.py')).read()
GCODE = re.search(r'SLICER_GCODE = """(.*?)"""', src, re.S).group(1).split('\n')
GCODE = ['G92 X0 Y0 Z0' if l == 'G28 W' else l for l in GCODE]

resc = '/tmp/printcore_test.resc'
open(resc, 'w').write('\n'.join([
    'include @%s' % os.path.join(HERE, 'models', 'TeacupSTM32DMA.cs'),
    'include @%s' % os.path.join(HERE, 'models', 'TeacupSTM32_UART.cs'),
    'include @%s' % os.path.join(HERE, 'models', 'TeacupSTM32_OTGFS.cs'),
    'mach create "pc"',
    'machine LoadPlatformDescription @%s' % REPL,
    'sysbus LoadELF @%s' % ELF,
    'logLevel 3',
    'emulation CreateServerSocketTerminal %d "term" false' % PORT,
    'connector Connect sysbus.%s term' % HOSTDEV,
    'start']) + '\n')
renode = subprocess.Popen([RENODE, '--console', '--disable-gui', resc], stdin=subprocess.PIPE,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

for _ in range(120):                       # wait for the socket
    try:
        socket.create_connection(('127.0.0.1', PORT), 1).close(); break
    except OSError:
        time.sleep(0.5)

received = []
p = printcore()
p.recvcb = lambda l: received.append(l.strip())
fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-44s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1
def wait(cond, timeout):
    t0 = time.time()
    while not cond() and time.time() - t0 < timeout:
        time.sleep(0.1)
    return cond()
def last_pos():
    for l in reversed(received):
        m = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+)', l)
        if m: return tuple(float(v) for v in m.groups())

try:
    p.connect('127.0.0.1:%d' % PORT, 115200)
    check('printcore online', wait(lambda: p.online, 60))

    # Manual control like Pronterface's buttons.
    for c in ['M105', 'G91', 'G1 X10 F3000', 'G1 Y5 F3000', 'G90', 'M114']:
        p.send_now(c); time.sleep(1.5)
    check('jog X+10 Y+5 (relative) -> X10 Y5', last_pos() is not None and last_pos()[:2] == (10.0, 5.0), last_pos())
    check('temperature report', any(re.match(r'ok T:[\d.]+/', l) for l in received), '')

    # Print the PrusaSlicer file, pause and resume in the middle.
    p.startprint(gcoder.LightGCode(GCODE))
    wait(lambda: p.queueindex > 30, 120)
    p.send_now('M105')
    p.pause(); time.sleep(3)
    paused_at = p.queueindex
    time.sleep(3)
    check('pause holds the print', p.queueindex == paused_at and not p.printing, (paused_at, p.queueindex))
    p.resume()
    check('print finishes', wait(lambda: not p.printing, 240), p.queueindex)
    time.sleep(2)
    p.send_now('M114'); time.sleep(2)
    pos = last_pos()
    check('final position X106.854 Y96.803 Z11', pos is not None and pos == (106.854, 96.803, 11.0), pos)
    errs = [l for l in received if l.startswith('Error')]
    check('no errors', not errs, errs)
    check('temperatures reported during M109', any(l.startswith('T:') for l in received), '')
finally:
    try: p.disconnect()
    except Exception: pass
    renode.kill()

open('/tmp/printcore_received.log', 'w').write('\n'.join(received))
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
