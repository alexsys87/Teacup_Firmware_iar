#!/usr/bin/env python3
"""Renode test of an SD card in SPI mode on the bus of the SPI flash
(SD_CARD_SELECT_PIN PC_14), together with a MAX31865 (HOTEND_MAX31865):

  make CHIP=F401 TEST=0 EXTRA="-DSD_CARD_SELECT_PIN=PC_14 -DHOTEND_MAX31865" BUILD_SUFFIX=_sd

Three devices on SPI1 (models/TeacupSPIMux.cs): W25Q64 (CS PA4), SD card
(models/TeacupSDCard.cs, CS PC14) and MAX31865 (CS PC15). The bus model
counts bytes sent with more than one chip selected. The card image is
FAT32 with a partition table (mkfs.fat, mtools), made like a used card:
TEST.GCO is fragmented and ends in clusters of a deleted file, whose
lines ("M112 ...") are still in the rest of the last sector. Its last
line has no line end. The mechanics model (models/p3_model.py) drives the
endstops.

Checks:
  - the card is mounted at startup ("echo:SD card ok"), the flash found
  - M20: files with sizes, subdirectory, long name as 8.3, no hidden file
  - M23 / M24: Marlin messages, the file is printed from the card; its
    commands use the bus while the file is read: G4 (MAX31865 readings),
    M500 (settings into the flash), power loss records (flash)
  - no bus conflict (two chips selected) at any time
  - power loss in the middle (M112, M999): the record in the flash names
    the file on the card, the card is mounted at startup, M1000 resumes
  - the last line without line end is executed, nothing after the end of
    the file (the junk would halt the printer)
  - no card: "SD init fail", M20 fails; card inserted: M21 mounts
  - a slow card (ACMD41 busy for ~1 s, longer than the watchdog): mounts
    without a watchdog reset

Needs mkfs.fat (dosfstools) and mcopy/mmd/mdel/mattrib/mshowfat (mtools).

Usage: run_sdcard.py [renode] [teacup.elf] [platform.repl]
"""
import math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, 'models')
sys.path.insert(0, MODELS)
import p3_model

RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_0_sd/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')
TMP = '/tmp/teacup_sd_%d' % os.getpid()
os.makedirs(TMP, exist_ok=True)

OVERSAMPLE, SLOTS, ADC_MAX = 16, 1, 4095     # only the bed is analog
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
BED = c_to_adc(60.0)
R200 = 100.0 * (1 + 3.9083e-3 * 200 - 5.775e-7 * 200 * 200)   # PT100 at 200 C

_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
ADCBUF = int(re.search(r'^([0-9a-f]+) \S adc_buffer$', _nm, re.M).group(1), 16)

# --- The card image ---------------------------------------------------------
LAYERS, LH = 6, 0.2
FILE = ['; SD card print test', 'M92 X40 Y40 Z400 E100', 'M83', 'M104 S200',
        'M140 S60', 'G28 X Y', 'G92 Z0 E0', 'M105', 'G4 P600', 'M500']
for k in range(LAYERS):
    FILE.append('G1 Z%.1f F600' % (LH * (k + 1)))
    FILE += ['G1 X30 Y10 E1 F1200', 'G1 X30 Y30 E1', 'G1 X10 Y30 E1', 'G1 X10 Y10 E1']
    # Padding: the file spans many sectors and clusters.
    FILE += ['; layer %d padding %s' % (k, 'x' * 90)] * 12
FILE.append('M114')                        # no line end after this one
E_FILE = 4.0 * LAYERS
text = ('\r\n'.join(FILE)).encode()        # CR LF, like from Windows

def sh(*a):
    subprocess.run(a, check=True, capture_output=True)
part, img = os.path.join(TMP, 'part.img'), os.path.join(TMP, 'sd.img')
PART_KB = 63 * 1024
if os.path.exists(part): os.remove(part)
sh('mkfs.fat', '-C', '-F', '32', '-s', '1', part, str(PART_KB))
def put(name, data):
    p = os.path.join(TMP, 'f.tmp'); open(p, 'wb').write(data)
    sh('mcopy', '-o', '-i', part, p, '::' + name)
put('HOLE.TXT', b'h' * 2048)               # a hole of 4 clusters later
put('SPACER1.TXT', b's' * 600)
put('JUNK.GCO', b'M112 junk of a deleted file\n' * 3000)
put('SPACER2.TXT', b's' * 600)
sh('mdel', '-i', part, '::HOLE.TXT')
sh('mdel', '-i', part, '::JUNK.GCO')
# FAT32 keeps a "next free cluster" hint (FSInfo, sector 1), mtools would
# append after the last file. Reset it: TEST.GCO fills the holes.
with open(part, 'r+b') as f:
    f.seek(512 + 492); f.write((2).to_bytes(4, 'little'))
put('TEST.GCO', text)                      # into the hole, then the junk
put('README.TXT', b'hello\n')
sh('mmd', '-i', part, '::PARTS')
put('PARTS/PART.GCO', b'G1 X1\n')
put('Long file name.gcode', b'G1 X2\n')
put('HIDDEN.GCO', b'G1 X3\n')
sh('mattrib', '-i', part, '+h', '::HIDDEN.GCO')
FAT = subprocess.run(['mshowfat', '-i', part, '::TEST.GCO'], capture_output=True, text=True).stdout.strip()
# Partition table: one FAT32 LBA partition at 1 MB.
mbr = bytearray(512)
start, count = 2048, PART_KB * 2
mbr[446:462] = bytes([0, 0xFE, 0xFF, 0xFF, 0x0C, 0xFE, 0xFF, 0xFF]) + \
    start.to_bytes(4, 'little') + count.to_bytes(4, 'little')
mbr[510:512] = b'\x55\xAA'
with open(img, 'wb') as f:
    f.write(bytes(mbr) + bytes(start * 512 - 512))
    f.write(open(part, 'rb').read())
os.remove(part)

# --- The Renode script ------------------------------------------------------
repl2 = os.path.join(TMP, 'bus.repl')
open(repl2, 'w').write('''spimux: SPI.TeacupSPIMux @ spi1

spiflash: SPI.TeacupW25Q @ spimux 0
    sizeMB: 8

sdcard: SPI.TeacupSDCard @ spimux 1
    imageFile: "%s"

max31865: SPI.TeacupMAX31865 @ spimux 2

gpioPortA:
    4 -> spiflash@0

gpioPortC:
    14 -> sdcard@0
    15 -> max31865@0
''' % img)

lines = []
def cmd(c): lines.append(c)
TAGS = []
def mark(name):
    """Start a section: "M73 P<n>" + "M73" puts a tag into the UART stream
    (see run_power_loss.py)."""
    TAGS.append(name)
    cmd('echo "@@MARK %s"' % name)
    send('M73 P%d R0\nM73\n' % len(TAGS))
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.usart2 WriteChar 0x%02X' % ord(ch))
def pin(port, n, level): cmd('sysbus.gpioPort%s OnGPIO %d %s' % (port, n, 'true' if level else 'false'))
def adc(bed):
    for i in range(OVERSAMPLE):
        cmd('sysbus WriteWord 0x%08X %d' % (ADCBUF + 2 * (i * SLOTS + 0), bed))
def model(call): cmd('python "import p3_model; p3_model.%s"' % call)
def report(tag): model("report('%s')" % tag)
def hook(addr, fn):
    cmd('sysbus AddWatchpointHook 0x%08X 4 Write "import p3_model; p3_model.%s(self.Machine, value)"' % (addr, fn))
SD, MUX = 'sysbus.spi1.spimux.sdcard', 'sysbus.spi1.spimux'
def prop(tag, path):
    cmd('echo "@@%s"' % tag); cmd(path)

for _m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs', 'TeacupW25Q.cs',
           'TeacupSDCard.cs', 'TeacupMAX31865.cs', 'TeacupSPIMux.cs'):
    cmd('include @%s' % os.path.join(MODELS, _m))
cmd('mach create "p3"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('machine LoadPlatformDescription @%s' % repl2)
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3'); cmd('logLevel -1 sysbus.usart2'); cmd('logLevel 2 %s' % MUX)
cmd('python "import sys; sys.path.append(\'%s\'); import p3_model"' % MODELS)
cmd('sysbus.spi1.spimux.max31865 Resistance %.4f' % R200)
def inputs():
    # Switches open (high), probe low, filament present (pull-up).
    pin('B', 10, True); pin('B', 3, True); pin('B', 15, False); pin('A', 0, True)
    model("pins.update({('B', 10): True, ('B', 3): True, ('B', 15): False})")
inputs()
hook(0x40010000, 'step_x')        # TIM1 CR1
hook(0x40000000, 'step_y')        # TIM2 CR1
hook(0x40000400, 'step_e')        # TIM3 CR1
hook(0x40020418, 'gpiob_bsrr')    # Z step (PB12)
def boot(name):
    mark(name); run('0.02'); adc(BED); run('0.6')
def reset(name):
    send('M112\n'); run('0.1'); mark(name); send('M999\n'); run('0.02'); adc(BED)
    inputs()                      # the reset clears the GPIO inputs
    run('0.8')

boot('boot')
send('M80\n'); run('0.7')
mark('m20'); send('M20\n'); run('0.3')
mark('open'); send('M23 TEST.GCO\n'); run('0.1')
model("pos.update({'x': 50.0, 'y': 60.0, 'z': 8.0, 'z2': 8.0, 'e': 0.0})")
mark('print'); send('M24\n'); run('11.0'); report('cut')
reset('restart')
mark('resume'); send('M1000\n'); run('60.0'); report('end')
mark('m27'); send('M27\n'); run('0.1')
prop('CONFLICTS', MUX + ' Conflicts'); prop('BYTES', MUX + ' Bytes')
prop('READS', SD + ' Reads')
# No card, then inserted again.
cmd(SD + ' Inserted false')
mark('nocard'); send('M21\n'); run('0.3')
mark('nocard_m20'); send('M20\n'); run('0.1')
cmd(SD + ' Inserted true')
mark('inserted'); send('M21\nM20\n'); run('0.3')
# A slow card: ACMD41 busy for ~1 s.
cmd(SD + ' PowerCycle'); cmd(SD + ' BusyCount 8000')
mark('slow'); send('M21\n'); run('2.5')
mark('slow_m23'); send('M23 README.TXT\n'); run('0.1')
prop('ACMD', SD + ' AcmdCount')
prop('CONFLICTS2', MUX + ' Conflicts')
cmd('quit')

script = os.path.join(TMP, 'test.resc')
open(script, 'w').write('\n'.join(lines) + '\n')
with open('/tmp/teacup_sdcard.log', 'w') as logf:
    proc = subprocess.Popen([RENODE, '--console', '--disable-gui', script],
                            stdin=subprocess.PIPE, stdout=logf,
                            stderr=subprocess.STDOUT, text=True)
    proc.wait()
out = re.sub(r'\x1b\[[0-9;]*m', '', open('/tmp/teacup_sdcard.log').read())
for f in os.listdir(TMP): os.remove(os.path.join(TMP, f))
os.rmdir(TMP)

STREAM = [re.sub(r'^.*\] ', '', l) for l in out.splitlines() if 'usart2: [host' in l]
TAGRE = re.compile(r'^echo:Progress: (\d+)%, remaining 0 min$')
sections, cur = {'pre': []}, 'pre'
for l in STREAM:
    mm = TAGRE.match(l)
    if mm and 1 <= int(mm.group(1)) <= len(TAGS):
        cur = TAGS[int(mm.group(1)) - 1]; sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)
def uart(sec):
    return ['ok' if re.match(r'^ok( N-?\d+)? P\d+ B\d+$', l) else l for l in sections.get(sec, [])]
v = {}
for mm in re.finditer(r'@@(\w+)\s*\n(?:.*\n)*?\s*(0x[0-9A-Fa-f]+|\d+)\s*\n', out):
    v.setdefault(mm.group(1), int(mm.group(2), 0))
MODEL = {}
for l in out.splitlines():
    mm = re.search(r'@@MODEL (\S+) (.*)', l)
    if mm:
        d = {}
        for kv in mm.group(2).split():
            k, val = kv.split('=', 1)
            try: d[k] = float(val)
            except ValueError: d[k] = val
        MODEL[mm.group(1)] = d

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-50s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1

print('TEST.GCO clusters: %s' % FAT)
check('image: TEST.GCO fragmented', FAT.count('<') >= 2, FAT)
u = uart('pre') + uart('boot')
check('boot: flash found, card mounted', any(l.startswith('echo:SPI flash: manufacturer') for l in u)
      and 'echo:SD card ok' in u, u)
u = uart('m20')
exp = ['TEST.GCO %d' % len(text), 'README.TXT 6', 'PARTS/PART.GCO 6', 'LONGFI~1.GCO 6']
check('M20: files with sizes, subdirectory, 8.3 name', 'Begin file list' in u and 'End file list' in u
      and all(e in u for e in exp), u)
check('  no hidden, deleted or directory entries', not any(('HIDDEN' in l or 'JUNK' in l or 'HOLE' in l
      or l.startswith('PARTS ') or l == 'PARTS/') for l in u), u)
u = uart('open')
check('M23: opened, selected', 'File opened: TEST.GCO Size: %d' % len(text) in u
      and 'File selected' in u, u)
u = uart('print')
check('printing: G4 / M105 in the file', any('T:200.0' in l for l in u), u)
# Boot messages come before the section tag (sent after the reset).
u = uart('print') + uart('restart')
check('restart: settings from the flash (M500 in the file)', 'echo:Settings in SPI flash' in u, u)
check('  card mounted, interrupted print reported', 'echo:SD card ok' in u
      and any(l.startswith('echo:Power loss recovery: TEST.GCO') for l in u), u)
m0, m1 = MODEL.get('cut', {}), MODEL.get('end', {})
check('  printing was in the middle', 2.0 < m0.get('e', 0) < E_FILE - 2.0, m0.get('e'))
u = uart('resume')
check('M1000: resumed from the card, file finished', 'echo:Print resumed' in u and 'SD file done.' in u, u)
Z0 = 13.0                                   # physical Z of G92 Z0 (8 + 5 mm lift)
check('  end position X10 Y10 Z%.1f' % (LH * LAYERS), m1 and abs(m1['x'] - 10) <= 0.2
      and abs(m1['y'] - 10) <= 0.2 and abs(m1['z'] - Z0 - LH * LAYERS) <= 0.003,
      (m1.get('x'), m1.get('y'), m1.get('z')))
# M114 is the only one in the file (read ahead, while moves are queued).
check('  last line without line end executed (M114)', any(l.startswith('X:') for l in u), u)
check('  nothing after the end of the file', not any('halted' in l or 'kill' in l for l in u), u)
check('M27: not printing', 'Not SD printing' in uart('m27'), uart('m27'))
check('no bus conflict (%d bytes)' % v.get('BYTES', 0), v.get('CONFLICTS') == 0 and v.get('BYTES', 0) > 10000,
      (v.get('CONFLICTS'), v.get('BYTES')))
check('card read (%d blocks)' % v.get('READS', 0), v.get('READS', 0) > 50, v.get('READS'))
u = uart('nocard')
check('no card: SD init fail', any(l.startswith('echo:SD init fail') for l in u), u)
check('  M20 fails', any('Failed to open dir' in l for l in uart('nocard_m20')), uart('nocard_m20'))
u = uart('inserted')
check('inserted: M21 mounts, M20 lists', 'echo:SD card ok' in u and 'TEST.GCO %d' % len(text) in u, u)
u = uart('slow') + uart('slow_m23')
check('slow card (%d ACMD41): mounted, no watchdog reset' % v.get('ACMD', 0), 'echo:SD card ok' in u
      and not any('Watchdog' in l or l == 'start' for l in u) and v.get('ACMD', 0) >= 8000, u)
check('  file opened after it', 'File opened: README.TXT Size: 6' in u, u)
check('no bus conflict at the end', v.get('CONFLICTS2') == 0, v.get('CONFLICTS2'))
check('no error', not any(l.startswith('Error') for l in STREAM if 'kill' not in l and 'M112' not in l
      and 'halted' not in l), [l for l in STREAM if l.startswith('Error')])
print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
