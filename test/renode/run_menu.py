#!/usr/bin/env python3
"""Renode test of the display menu (DISPLAY_MENU) with buttons on the
PCF8574, on the P3 Steel configuration with SPI flash and SD card, for
both display types:

  make CHIP=F401 TEST=0 EXTRA="-DDISPLAY_TYPE_SSD1306 -DDISPLAY_MENU -DSD_CARD_SELECT_PIN=PC_14" BUILD_SUFFIX=_oled
  make CHIP=F401 TEST=0 EXTRA="-DDISPLAY_TYPE_HD44780 -DDISPLAY_MENU -DSD_CARD_SELECT_PIN=PC_14" BUILD_SUFFIX=_lcd
  run_menu.py [renode] oled|lcd [teacup.elf]

Models on I2C1: PCF8574 at 0x20 with the buttons (models/TeacupPCF8574.cs,
Inputs: P0 up, P1 down, P2 OK, P3 back, pressed = low), the display:
SSD1306 128x64 at 0x3C (models/TeacupSSD1306.cs, the screen is decoded
with the firmware's font) or HD44780 20x4 behind a PCF8574 backpack at
0x27 (models/TeacupHD44780.cs). SPI1: W25Q64 and SD card
(models/TeacupSPIMux.cs, TeacupSDCard.cs).

Checks:
  - status screen: temperatures, "Ready"
  - main menu, moving the selection, back button
  - edit the hotend temperature, held button repeats; the value is set
    (M105), cooldown
  - settings: steps/mm X +0.3, store (M500), still there after a reset
  - SD card: file list (subdirectory, no README.TXT), into the
    subdirectory and back, print a file with confirmation (M27)
  - while printing: pause / resume, tune: speed with fast repeat
    (clamped at 500 %), stop print with confirmation (heaters off, not
    printing, message)
  - no error

Needs mkfs.fat (dosfstools) and mtools. Renode needs about 9 GB for
this test (its memory grows with the emulated time), don't run it in
parallel with other tests.
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, 'models')
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
KIND = sys.argv[2] if len(sys.argv) > 2 else 'oled'
ELF = sys.argv[3] if len(sys.argv) > 3 else os.path.join(
    HERE, '../gcc/build_F401_0_%s/teacup.elf' % KIND)
REPL = os.path.join(HERE, 'stm32f401_dma.repl')
TMP = '/tmp/teacup_menu_%s_%d' % (KIND, os.getpid())
os.makedirs(TMP, exist_ok=True)
LINES, COLS = (8, 21) if KIND == 'oled' else (4, 20)

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

# The firmware's font, to read the OLED.
_f = open(os.path.join(HERE, '../../display/font_6x8.c')).read()
GLYPHS = {tuple(int(x, 16) for x in g.split(',')): chr(0x20 + i)
          for i, g in enumerate(re.findall(r'\{\{(.*?)\}\}', _f))}

# --- Card image ---------------------------------------------------------------
# Short moves (0.1 s): host commands wait for queue space, not long.
PRINT = ['; menu test print', 'G92 X10 Y10 Z0 E0', 'M83']
for k in range(300):
    PRINT += ['G1 X%d Y10 F1200' % (12 if k % 2 == 0 else 10)]
    if k % 10 == 9:
        PRINT += ['G1 Z%.1f' % (0.1 * (k + 1) / 10)]
def sh(*a):
    subprocess.run(a, check=True, capture_output=True)
img = os.path.join(TMP, 'sd.img')
if os.path.exists(img): os.remove(img)
sh('mkfs.fat', '-C', '-F', '32', '-s', '1', img, str(40 * 1024))
def put(name, data):
    p = os.path.join(TMP, 'f.tmp'); open(p, 'wb').write(data)
    sh('mcopy', '-o', '-i', img, p, '::' + name)
put('TEST.GCO', ('\n'.join(PRINT) + '\n').encode())
put('README.TXT', b'not G-code\n')
sh('mmd', '-i', img, '::PARTS')
put('PARTS/PART.GCO', b'G1 X1\n')

repl2 = os.path.join(TMP, 'extra.repl')
open(repl2, 'w').write('''spimux: SPI.TeacupSPIMux @ spi1

spiflash: SPI.TeacupW25Q @ spimux 0
    sizeMB: 8

sdcard: SPI.TeacupSDCard @ spimux 1
    imageFile: "%s"

gpioPortA:
    4 -> spiflash@0

gpioPortC:
    14 -> sdcard@0

pcf: I2C.TeacupPCF8574 @ i2c1 0x20
''' % img + ('oled: I2C.TeacupSSD1306 @ i2c1 0x3C\n' if KIND == 'oled' else
             'lcd: I2C.TeacupHD44780 @ i2c1 0x27\n    cols: 20\n    lines: 4\n'))

# --- Script -------------------------------------------------------------------
lines = []
def cmd(c): lines.append(c)
TAGS = []
def mark(name):
    """Section of the UART output (M73 tag, see run_power_loss.py)."""
    TAGS.append(name)
    send('M73 P%d R0\nM73\n' % len(TAGS))
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def adc(hotend, bed):
    for i in range(OVERSAMPLE):
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 0), hotend))
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 1), bed))
UP, DOWN, OK, BACK = 0, 1, 2, 3
def press(bit, n=1, hold=0.08):
    for _ in range(n):
        cmd('sysbus.i2c1.pcf Inputs 0x%02X' % (0xFF & ~(1 << bit)))
        run('%.3f' % hold)
        cmd('sysbus.i2c1.pcf Inputs 0xFF')
        run('0.08')
    run('0.25')
SCREENS = []
def screen(name):
    SCREENS.append(name)
    cmd('echo "@@SCREEN %s"' % name)
    cmd('sysbus.i2c1.oled Dump' if KIND == 'oled' else 'sysbus.i2c1.lcd Text')
    cmd('echo "@@END"')

for m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs', 'TeacupW25Q.cs',
          'TeacupSDCard.cs', 'TeacupSPIMux.cs', 'TeacupPCF8574.cs',
          'TeacupSSD1306.cs' if KIND == 'oled' else 'TeacupHD44780.cs'):
    cmd('include @%s' % os.path.join(MODELS, m))
cmd('mach create "p3"')
cmd('machine LoadPlatformDescription @%s' % REPL)
cmd('machine LoadPlatformDescription @%s' % repl2)
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2')
def inputs():
    cmd('sysbus.gpioPortA OnGPIO 0 true')          # filament present
    for port, n in (('B', 10), ('B', 3), ('B', 15)):
        cmd('sysbus.gpioPort%s OnGPIO %d true' % (port, n))
inputs()
mark('boot'); run('0.02'); adc(ROOM, ROOM); run('0.5')
screen('greeting')
run('3.0')
screen('status')

# Main menu, selection, back button.
press(OK); screen('main')
press(DOWN, 3); screen('main_temp')
press(OK); screen('temperature')
press(BACK); screen('back')
# Hotend: hold up (repeat), OK.
press(OK); press(DOWN); press(OK); screen('edit0')
press(UP, 1, hold=1.25); screen('edit_held')
press(OK); screen('temp_after')
mark('m105'); send('M105\n'); run('0.2')
press(DOWN, 5); press(OK); screen('cooldown')         # Cooldown
mark('m105b'); send('M105\n'); run('0.2')
press(UP, 6); press(OK); screen('main_again')        # Back to main

# Settings: steps/mm X +0.3, store, reset.
press(DOWN); press(OK); screen('settings')           # Temperature -> Settings
press(DOWN); press(OK); screen('steps_edit')
press(UP, 3); screen('steps_up')
press(OK)
press(DOWN, 22); screen('settings_end')              # ... -> Store settings
press(OK); run('0.5')
mark('m503'); send('M503\n'); run('0.3')
send('M112\n'); run('0.1'); mark('reset'); send('M999\n'); run('0.02')
adc(ROOM, ROOM); inputs(); run('3.0')
mark('m503b'); send('M503\n'); run('0.3')

# SD card: list, subdirectory, print.
press(OK); press(DOWN); press(OK); screen('files')   # Main -> Print from SD
press(DOWN, 3); press(OK); screen('subdir')          # PARTS/
press(UP); press(OK); screen('updir')                # < Up
press(DOWN, 2); press(OK); screen('confirm')         # TEST.GCO
press(DOWN); press(OK); run('1.0')                   # Yes: status screen
mark('m27'); send('M27\n'); run('0.2')
screen('printing')

# While printing: pause, resume, tune, stop.
press(OK); screen('main_print')
press(DOWN); press(OK); run('0.5')                   # Pause print
mark('m27p'); send('M27\n'); run('0.2')
screen('main_paused')
press(OK); run('0.5')                                # Resume print
mark('m27r'); send('M27\n'); run('0.2')
press(DOWN, 2); press(OK); screen('tune')            # Tune
press(DOWN); press(OK)                               # Speed %
press(UP, 1, hold=2.6); screen('speed_fast')
press(OK); press(UP, 9)
press(OK); screen('main_print2')                     # < Back
press(UP); press(OK); screen('stop_confirm')         # Stop print
press(DOWN); press(OK); run('2.0')                   # Yes: status screen
mark('m27s'); send('M27\nM105\n'); run('0.3')
run('0.5'); screen('stopped')
mark('end'); run('0.2')
cmd('quit')

script = os.path.join(TMP, 'test.resc')
open(script, 'w').write('\n'.join(lines) + '\n')
log = '/tmp/teacup_menu_%s.log' % KIND
with open(log, 'w') as logf:
    rc = subprocess.run([RENODE, '--console', '--disable-gui', script],
                        stdin=subprocess.PIPE, stdout=logf, stderr=subprocess.STDOUT).returncode
if rc != 0:
    # Renode grows by about 100 MB per emulated second here: this test
    # takes about 9 GB, run it alone.
    print('Renode ended with %d (killed: out of memory?)' % rc)
out = re.sub(r'\x1b\[[0-9;]*m', '', open(log).read())
for f in os.listdir(TMP): os.remove(os.path.join(TMP, f))
os.rmdir(TMP)

# --- Evaluation ---------------------------------------------------------------
def decode_oled(hexlines):
    text = []
    for p in range(LINES):
        row = bytes.fromhex(hexlines[p])
        s = ''
        for c in range(COLS):
            s += GLYPHS.get(tuple(row[1 + 6 * c:6 + 6 * c]), '?')
        text.append(s)
    return text
SCR = {}
for mm in re.finditer(r'@@SCREEN (\S+)\s*\n(.*?)@@END', out, re.S):
    body = [l.strip() for l in mm.group(2).splitlines() if l.strip() and not l.startswith('(')]
    if KIND == 'oled':
        hx = [l for l in body if re.fullmatch(r'[0-9A-F]{264}', l)]
        SCR[mm.group(1)] = decode_oled(hx) if len(hx) >= LINES else []
    else:
        t = [l for l in body if '|' in l]
        SCR[mm.group(1)] = t[0].split('|')[:LINES] if t else []

STREAM = [re.sub(r'^.*\] ', '', l) for l in out.splitlines() if 'usart2: [host' in l]
TAGRE = re.compile(r'^echo:Progress: (\d+)%, remaining 0 min$')
sections, cur = {'pre': []}, 'pre'
for l in STREAM:
    mm = TAGRE.match(l)
    if mm and 1 <= int(mm.group(1)) <= len(TAGS):
        cur = TAGS[int(mm.group(1)) - 1]; sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def uart(sec):
    return sections.get(sec, [])

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-52s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1
def has(scr, text):
    return any(text in l for l in SCR.get(scr, []))
def sel(scr):
    for l in SCR.get(scr, []):
        if l.startswith('>'): return l[1:].strip()
    return None
def show(scr):
    return SCR.get(scr)

for k in ('greeting', 'status', 'main'):
    print('  %-10s %s' % (k, show(k)))
check('greeting', has('greeting', 'Teacup'), show('greeting'))
check('status screen: temperatures, Ready', (has('status', 'Hotend 25/0') or has('status', 'E25/0'))
      and has('status', 'Ready'), show('status'))
check('main menu', SCR.get('main', [''])[0].startswith('Main') and sel('main') == '< Status'
      and has('main', 'Print from SD') and has('main', 'Motion'), show('main'))
check('  down x3: Temperature', sel('main_temp') and sel('main_temp').startswith('Temperature'), show('main_temp'))
check('  OK: temperature menu', SCR.get('temperature', [''])[0].startswith('Temperature')
      and sel('temperature') == '< Back', show('temperature'))
check('  back button: main, Temperature selected', SCR.get('back', [''])[0].startswith('Main')
      and sel('back', ) and sel('back').startswith('Temperature'), show('back'))
check('edit hotend: 0', has('edit0', 'Hotend') and any(l.strip() == '0' for l in SCR.get('edit0', [])), show('edit0'))
held = None
for l in SCR.get('edit_held', []):
    if re.fullmatch(r'\s*\d+\s*', l): held = int(l)
check('  held 1.25 s: repeated (8..12)', held is not None and 8 <= held <= 12, show('edit_held'))
t105 = None
for l in uart('m105'):
    mm = re.search(r'T:[\d.]+/(\d+)\.0', l)
    if mm: t105 = int(mm.group(1))
check('  set: M105 target = shown value', t105 is not None and t105 == held, (t105, uart('m105')))
check('  temperature menu shows it', sel('temp_after') and sel('temp_after').startswith('Hotend')
      and sel('temp_after').endswith(str(held)), show('temp_after'))
check('cooldown: target 0', any(re.search(r'T:[\d.]+/0\.0 ', l) for l in uart('m105b')), uart('m105b'))
check('settings menu', SCR.get('settings', [''])[0].startswith('Settings'), show('settings'))
check('  steps/mm X 160.000 -> 160.300', has('steps_edit', '160.000') and has('steps_up', '160.300'),
      (show('steps_edit'), show('steps_up')))
check('  store settings selected', sel('settings_end') == 'Store settings', show('settings_end'))
check('  M503: M92 X160.30', any('M92 X160.30 ' in l for l in uart('m503')), uart('m503'))
check('  after reset: from the flash', any('M92 X160.30 ' in l for l in uart('m503b')), uart('m503b'))
print('  files      %s' % show('files'))
# The LCD shows 3 items: PARTS is below, checked by going into it.
check('file list: TEST.GCO (PARTS/), no README', has('files', 'TEST.GCO') and
      (has('files', 'PARTS') or LINES < 5) and not has('files', 'README'), show('files'))
check('  subdirectory', SCR.get('subdir', [''])[0].startswith('PARTS') and has('subdir', 'PART.GCO'), show('subdir'))
check('  up again', SCR.get('updir', [''])[0].startswith('Print from SD'), show('updir'))
check('  confirmation', SCR.get('confirm', [''])[0].startswith('Print file?') and sel('confirm') == '< No', show('confirm'))
check('printing (M27)', any(l.startswith('SD printing byte') for l in uart('m27')), uart('m27'))
check('  status screen: SD, file', has('printing', 'SD ') and has('printing', 'TEST.GCO'), show('printing'))
check('  menu: pause, stop (tune)', has('main_print', 'Pause print') and has('main_print', 'Stop print')
      and (has('main_print', 'Tune') or LINES < 5), show('main_print'))
check('  paused (M27), menu: resume', 'Not SD printing' in uart('m27p') and has('main_paused', 'Resume print'),
      (uart('m27p'), show('main_paused')))
check('  resumed (M27)', any(l.startswith('SD printing byte') for l in uart('m27r')), uart('m27r'))
check('  tune menu', SCR.get('tune', [''])[0].startswith('Tune'), show('tune'))
check('  speed: fast repeat, clamped at 500', any(l.strip() == '500' for l in SCR.get('speed_fast', [])), show('speed_fast'))
check('  stop: confirmation', SCR.get('stop_confirm', [''])[0].startswith('Stop print?'), show('stop_confirm'))
us = uart('m27s')
check('  stopped: not printing, heaters off', 'Not SD printing' in us and any(re.search(r'T:[\d.]+/0\.0 ', l) for l in us), us)
check('  status: Print stopped', has('stopped', 'Print stopped'), show('stopped'))
check('no error', rc == 0 and not any(l.startswith('Error') for l in STREAM if 'M112' not in l and 'halted' not in l),
      [l for l in STREAM if l.startswith('Error')])
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
