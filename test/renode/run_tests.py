#!/usr/bin/env python3
"""Renode emulation test of the Teacup STM32F4 firmware.

Builds nothing: expects test/gcc/build_F401_1/teacup.elf (make CHIP=F401
TEST=1). Drives G-code over USART2, observes UART output and GPIO writes
(step pulses) and checks the results.

Usage: run_tests.py [renode] [teacup.elf] [platform.repl] [all|basic|safety|proto|cmds|settings|slicer|tune|dfu]
  F401 build: stm32f401.repl (84 MHz), F411 build: stm32f411.repl (96 MHz).

Host port: TEACUP_PORT=usb runs the same tests over the USB CDC port (model
models/TeacupSTM32_OTGFS.cs), default is the UART. Boot and restart
messages ("start") are always checked on the UART: USB isn't enumerated
yet when they're sent.
"""
import math, os, re, subprocess, sys

from renode_common import artifact_path, monitor_command, run_renode

HERE = os.path.dirname(os.path.abspath(__file__))
RENODE = sys.argv[1] if len(sys.argv) > 1 else 'renode'
ELF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '../gcc/build_F401_1/teacup.elf')
REPL = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, 'stm32f401_dma.repl')
PART = sys.argv[4] if len(sys.argv) > 4 else 'all'      # all or a list: basic,proto,...
PORT = os.environ.get('TEACUP_PORT', 'uart')            # uart or usb
HOSTDEV = 'usb' if PORT == 'usb' else 'usart2'
PARTS = set(PART.split(','))
def want(part, in_all=True):
    return part in PARTS or (in_all and 'all' in PARTS)

# Addresses of test hooks (see src/temp.c) and of the status message.
def sym(name):
    m = re.search(r'^([0-9a-f]+) \S %s(\.lto_priv\.\d+)?$' % name, _nm, re.M)
    return int(m.group(1), 16) if m else 0
_nm = subprocess.run(['arm-none-eabi-nm', ELF], capture_output=True, text=True).stdout
FORCE = sym('temp_dummy_force')
STATUS_MSG = sym('status_msg')
PLANT = sym('temp_dummy_plant')
FANLOSS = sym('temp_dummy_fan_loss')
# Linear advance and input shaping step E, X, Y on their own
# (motion/linear_advance.c, motion/input_shaping.c).
STEP_AUX = sym('la_service') != 0 or sym('shaper_service') != 0

lines = []
def cmd(c): lines.append(monitor_command(c))
def mark(name): cmd('echo "@@MARK %s"' % name)
def run(t): cmd('emulation RunFor "%s"' % t)
def send(text):
    for ch in text:
        cmd('sysbus.%s WriteChar 0x%02X' % (HOSTDEV, ord(ch)))
def pin(port, n, level): cmd('sysbus.gpioPort%s OnGPIO %d %s' % (port, n, 'true' if level else 'false'))

for _m in ('TeacupSTM32DMA.cs', 'TeacupSTM32_UART.cs', 'TeacupSTM32_OTGFS.cs'):
    cmd('include @%s' % os.path.join(HERE, 'models', _m))
cmd('mach create "teacup"')
cmd('machine LoadPlatformDescription @%s' % os.path.abspath(REPL))
cmd('sysbus LoadELF @%s' % os.path.abspath(ELF))
cmd('showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('showAnalyzer sysbus.usb Antmicro.Renode.Analyzers.LoggingUartAnalyzer')
cmd('logLevel 3')
cmd('logLevel -1 sysbus.usart2')
cmd('logLevel -1 sysbus.usb')
cmd('logLevel -1 sysbus.gpioPortA')
cmd('logLevel -1 sysbus.gpioPortB')
pin('B', 12, True)                       # X min endstop released (inverted input)
mark('boot'); run('0.3')
cmd('sysbus LogPeripheralAccess sysbus.gpioPortA true')
cmd('sysbus LogPeripheralAccess sysbus.gpioPortB true')
for _t in (('timer1', 'timer2', 'timer3') if sym('step_timer') else ()):  # STEP_TIMER_PULSES
    cmd('sysbus LogPeripheralAccess sysbus.%s true' % _t)
    cmd('logLevel -1 sysbus.%s' % _t)

if want('basic'):
    mark('m115'); send('M115\n'); run('0.05')
    mark('m105'); send('M105\n'); run('0.05')
    mark('m119_open'); send('M119\n'); run('0.1')
    pin('B', 12, False)
    mark('m119_trig'); send('M119\n'); run('0.1')
    pin('B', 12, True)

    # 10 mm at 10 mm/s: 400 steps, ~1.0 s plus ramps. Position probe halfway.
    mark('move_x'); send('G1 X10 F600\n'); run('0.55')
    mark('m114_mid'); send('M114\n'); run('1.2')
    mark('m114_end'); send('M114\n'); run('0.05')

    # Combined move back: X -400 steps, Y +200 steps, E ~193 steps.
    mark('move_xye'); send('G1 X0 Y5 E2 F1200\n'); run('1.5')
    mark('m114_xye'); send('M114\n'); run('0.05')

    # Queue burst: 8 moves sent at once, lookahead joins them.
    mark('burst'); send(''.join('G1 X%d Y%d F3000\n' % (i, 5 + i) for i in range(1, 9))); run('2.0')
    mark('m114_burst'); send('M114\n'); run('0.05')

    # Heating with the dummy sensor (~5 C/s). M116 returns at once, the next
    # move waits until target + residency time are reached.
    mark('heat'); send('M104 S30\nM116\nG1 X1\n'); run('12.0')
    mark('m105_hot'); send('M105\n'); run('0.05')

    # Homing X: endstop triggered, fast move stops; released, back-off stops.
    pin('B', 12, False)
    mark('home'); send('G28 X\n'); run('0.3')
    pin('B', 12, True); run('0.5')
    mark('m114_home'); send('M114\n'); run('0.05')

    # Hardware PWM heater (TIM4 CH3, PB8): 70 C below the target the PID
    # gives full power. (M106 P0 would last until the next PID run only,
    # 100 ms at most; the PWM scaling is checked with the fan, TIM4 CH4.)
    mark('m106'); send('M104 S100\n'); run('0.3')
    cmd('echo "@@TIM4 CCR3"')
    cmd('sysbus ReadDoubleWord 0x4000083C')
    cmd('echo "@@TIM4 ARR"')
    cmd('sysbus ReadDoubleWord 0x4000082C')
    send('M104 S0\n'); run('0.05')
    mark('end')

def cs(text):                            # add a checksum, like hosts do
    c = 0
    for ch in text:
        c ^= ord(ch)
    return '%s*%d' % (text, c)

if want('proto'):
    # ---------------- Host protocol and command queue (2.1, 2.2) ----------------
    mark('p_m110'); send(cs('N0 M110 N0') + '\n'); run('0.05')
    mark('p_n1'); send(cs('N1 G1 X1 F600') + '\n'); run('0.3')
    mark('p_badcs'); send('N2 M105*99\n'); run('0.05')
    mark('p_badn'); send(cs('N5 M105') + '\n'); run('0.05')
    mark('p_nocs'); send('N2 M105\n'); run('0.05')
    mark('p_csnon'); send('M105*12\n'); run('0.05')
    mark('p_resend'); send(cs('N2 M105') + '\n'); run('0.05')
    mark('p_m110set'); send(cs('N3 M110 N100') + '\n'); run('0.05')
    mark('p_after110'); send(cs('N101 M114') + '\n'); run('0.05')
    mark('p_comment'); send('M105 ; comment, M112 in here is ignored\n'); run('0.05')
    mark('p_unknown'); send('M9876\n'); run('0.05')
    # Long blocking command: G1 after M116 waits for the heater (~4 s).
    # (Not G4: its DWT busy-wait is very slow to emulate in Renode.)
    mark('p_busy'); send('M104 S45\nM116\nG1 X2 F600\n'); run('7.0')
    mark('p_m113'); send('M113 S0\nM104 S60\nM116\nG1 X1 F600\n'); run('6.0')
    send('M113 S2\nM104 S0\n'); run('0.05')
    # Host streams ahead (12 lines, ~2 ms apart like at 115200 baud).
    mark('p_stream')
    for i in range(12):
        send(cs('N%d G1 X%d F6000' % (102 + i, 2 + i)) + '\n'); run('0.002')
    run('2.0')
    mark('p_stream_pos'); send('M114\n'); run('0.05')

def read32(tag, addr):
    cmd('echo "@@%s"' % tag); cmd('sysbus ReadDoubleWord 0x%08X' % addr)

def force(v):                            # dummy sensor reading, qC; -1 = off
    cmd('sysbus WriteDoubleWord 0x%08X 0x%08X' % (FORCE, v & 0xFFFFFFFF))

if want('cmds'):
    # ---------------- G/M commands (2.3) ----------------
    # Known state, independent of earlier parts: 25 C, origin.
    send('M104 S0\nG92 X0 Y0 Z0 E0\n'); force(100); run('0.3'); force(-1); run('0.05')
    mark('c_m114'); send('M114\n'); run('0.05')
    mark('c_m109_start'); send('M109 S40\n'); run('1.0')        # still heating
    mark('c_m109_done'); run('6.0')
    mark('c_m109_hot'); send('M105\n'); run('0.05')
    mark('c_m109_noheat'); send('M109 S30\n'); run('0.1')      # hotter already: no wait
    mark('c_m109_r_start'); send('M109 R30\n'); run('1.0')     # R waits for cooling
    mark('c_m109_r_done'); run('5.0')
    mark('c_m109_cancel_start'); send('M109 S200\n'); run('1.0')
    mark('c_m109_cancel'); send('M108\n'); run('1.2')
    send('M104 S0\n'); run('0.05')
    # Part fan: kick-start 100 ms at full power, S1..255 -> PWM 20..255.
    mark('c_m106'); send('M106 S200\n'); run('0.05'); read32('CCR4_KICK', 0x40000840)
    run('0.1'); read32('CCR4_ON', 0x40000840)
    send('M106 S100\n'); run('0.02'); read32('CCR4_LOWER', 0x40000840)
    mark('c_m107'); send('M107\n'); run('0.05'); read32('CCR4_OFF', 0x40000840)
    send('M106 S1\n'); run('0.15'); read32('CCR4_MIN', 0x40000840)
    send('M107\n'); run('0.05')
    mark('c_m17'); send('M17\n'); run('0.05'); read32('ODR_M17', 0x40020014)
    mark('c_m84'); send('M84\n'); run('0.05'); read32('ODR_M84', 0x40020014)
    mark('c_m84s'); send('M84 S2\nM17\n'); run('0.05'); read32('ODR_IDLE0', 0x40020014)
    run('3.5'); read32('ODR_IDLE1', 0x40020014)
    send('M84 S120\n'); run('0.05')
    mark('c_m400'); send('G1 X10 F600\nM400\nM114\n'); run('2.0')
    mark('c_m117'); send('M117 Hello X99 World ; comment\nG1 Y2 F600\n'); run('0.5')
    cmd('echo "@@MSG"'); cmd('sysbus ReadBytes 0x%08X 20' % STATUS_MSG)
    mark('c_m73'); send('M73 P42 R17\nM73\n'); run('0.05')
    mark('c_m206'); send('M206 X5\nM114\n'); run('0.1')
    mark('c_m206_g1'); send('G1 Y3 F600\nM400\nM114\n'); run('0.5')
    pin('B', 12, False)
    mark('c_home'); send('G28 X\n'); run('0.3')
    pin('B', 12, True); run('0.5')
    mark('c_home_pos'); send('M114\nM206\n'); run('0.05')
    mark('c_m428_move'); send('G1 X8 F600\nM400\n'); run('1.0')
    mark('c_m428'); send('M428\nM114\n'); run('0.1')
    mark('c_m206_0'); send('M206 X0 Y0 Z0\nM114\n'); run('0.1')
    mark('c_m300'); send('M300 S1000 P200\n'); run('0.1')
    mark('c_m300_done'); run('0.3')

def m503(sec):
    mark(sec); send('M503\n'); run('0.1')

if want('settings'):
    # ---------------- Runtime settings and Flash storage (2.4, 2.5) ----------------
    mark('s_boot_check'); run('0.01')
    send('G92 X0 Y0 Z0 E0\n'); run('0.05')
    m503('s_defaults')
    mark('s_m92'); send('M92 X80\nG1 X10 F600\n'); run('1.5')
    mark('s_m92_pos'); send('M114\n'); run('0.05')
    mark('s_m203'); send('M203 X5\nG1 X20 F6000\n'); run('1.0')
    mark('s_m203_mid'); send('M114\n'); run('1.5')
    send('M203 X100\n'); run('0.05')
    mark('s_set'); send('M204 S500\nM205 X15\nM201 Z80\nM301 P10 I0.5 D50 F12.5\n'); run('0.1')
    m503('s_changed')
    mark('s_m204'); send('M204 P700 R3000\nM204 T800\n'); run('0.1')
    m503('s_m204_changed')
    mark('s_m500'); send('M500\n'); run('0.2')
    mark('s_m502'); send('M502\n'); run('0.1')
    m503('s_after502')
    mark('s_m501'); send('M501\n'); run('0.1')
    m503('s_after501')
    # Reset (M112 + M999): the settings must come back from Flash.
    # Print job timer: 2 s, paused 1 s, 1 s more. Earlier parts heated with
    # M109 (timer started): stop it and start from zero statistics.
    send('M77\n'); run('0.1'); send('M78 S78\n'); run('0.1')
    mark('j_start'); send('M75\n'); run('2.2')
    send('M76\n'); run('1.0')
    mark('j_paused'); send('M31\n'); run('0.1')
    send('M75\n'); run('1.0')
    send('M77\n'); run('0.2')
    mark('j_stopped'); send('M31\nM78\n'); run('0.2')
    mark('s_reset'); send('M112\n'); run('0.1'); send('M999\n'); run('0.5')
    m503('s_after_reset')
    mark('j_after_reset'); send('M78\n'); run('0.1')
    # Filament counts while the timer runs.
    send('M75\nG1 E30 F3000\nM400\nM77\n'); run('1.5')
    mark('j_filament'); send('M78\nG92 E0\n'); run('0.1')
    # Wear levelling: 10 saves in a 1 kB area force an erase. The
    # statistics record survives it.
    for k in range(10):
        send('M92 Y%d\nM500\n' % (41 + k)); run('0.1')
    # A print that never ends (power loss): failed.
    send('M75\n'); run('0.3')
    mark('s_many_reset'); send('M112\n'); run('0.1'); send('M999\n'); run('0.5')
    m503('s_after_many')
    mark('j_after_many'); send('M78\n'); run('0.1')
    # Leave defaults behind for the following parts.
    send('M502\nM500\n'); run('0.2')


# PrusaSlicer output (Marlin 2 flavour, Original Prusa style start G-code),
# sent like Pronterface/printcore does: "N-1 M110", then numbered lines with
# checksums, plain M105 in between. Comments are stripped by the host.
SLICER_GCODE = """M73 P0 R5
M201 X1000 Y1000 Z200 E5000
M203 X100 Y100 Z3 E100
M204 P1000 R1000 T1000
M205 X8.00 Y8.00 Z0.40 E4.50
M205 S0 T0
M107
M862.3 P "MK3S"
M115 U3.12.2
G90
M83
M104 S35
M140 S30
M190 S30
M109 S35
G28 W
G1 Y-3.0 F1000.0
G92 E0.0
G1 X60.0 E9.0 F1000.0
G1 X100.0 E12.5 F1000.0
G92 E0.0
M221 S95
G21
M900 K0.05
M107
G1 E-.8 F2100
G1 Z.6 F10800
G1 X104.215 Y98.057
G1 Z.2
G1 E.8 F2100
M204 S1000
G1 F1200
G1 X105.438 Y97.227 E.04632
G1 X106.854 Y96.803 E.04632
M73 Q17 S12
M106 S255
G4 S1
M117 Layer 2
G1 E-.8 F2100
M104 S0
M140 S0
M107
G1 Z10 F720
M84 X Y E
M73 P100 R0
G1 Z11 F720
M400
M114""".split('\n')
SLOW = {'M109 S35': '5.0', 'G4 S1': '1.2', 'M400': '1.0'}

def move_time(line, st):
    """Approximate duration of a G1 line (host pacing like ping-pong: the
    next line is sent when the move is done, so the queues never overflow
    - Pronterface waits for "ok", which comes when there's room)."""
    w = dict(re.findall(r'([XYZEF])(-?[\d.]+)', line))
    if 'F' in w: st['F'] = float(w['F'])
    d = {}
    for a in 'XYZ':
        if a in w:
            d[a] = float(w[a]) - st[a]; st[a] = float(w[a])
    e = float(w.get('E', 0))
    dist = math.sqrt(sum(v * v for v in d.values())) or abs(e)
    v = st['F'] / 60.0
    if 'Z' in d and dist == abs(d['Z']): v = min(v, 3.0)     # M203 Z3
    return dist / v + 0.1 if dist else 0.05

if want('slicer'):
    # ---------------- PrusaSlicer G-code via Pronterface protocol ----------------
    send('M104 S0\nG92 X0 Y0 Z0 E0\n'); force(100); run('0.3'); force(-1); run('0.05')
    mark('ps_start'); send(cs('N-1 M110') + '\n'); run('0.05')
    st = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'F': 1000.0}
    for n, line in enumerate(SLICER_GCODE):
        if n == 20:
            # Transmission error: bad checksum, the host resends.
            mark('ps_resend'); send('N%d %s*1\n' % (n, line)); run('0.05')
            send(cs('N%d %s' % (n, line)) + '\n'); run('0.3')
            mark('ps_after_resend')
            continue
        if n == 30:
            send('M105\n'); run('0.05')      # Pronterface's temperature poll
        if line == 'M84 X Y E':
            mark('ps_m84')
        if line == 'G28 W':
            # Endstops: pressed when the carriage arrives, released when it
            # backs off. X homes first, then Y.
            pin('B', 12, False); pin('B', 6, False)
            send(cs('N%d %s' % (n, line)) + '\n'); run('0.3')
            pin('B', 12, True); run('0.4')
            pin('B', 6, True); run('1.0')
            st.update({'X': 0.0, 'Y': 0.0})
            continue
        if line.startswith('G1'):
            delay = '%.2f' % move_time(line, st)
        elif line.startswith('G92'):
            delay = '0.1'
        else:
            delay = SLOW.get(line, '0.05')
        send(cs('N%d %s' % (n, line)) + '\n'); run(delay)
    mark('ps_end'); run('0.05')
    cmd('echo "@@PSMSG"'); cmd('sysbus ReadBytes 0x%08X 20' % STATUS_MSG)

if want('perf'):
    # ---------------- Step interrupt optimisations (4.1, 4.2, 4.9) ----------------
    # Own marker: the "ok" of this G92 must not count for the part before.
    mark('f_setup'); send('G92 X0 Y0 Z0 E0\n'); run('0.05')
    mark('f_reset'); send('M9001 R\n'); run('0.05')
    mark('f_move'); send('G1 X20 Y10 E5 F3000\n'); run('1.5')
    mark('f_stats'); send('M9001\n'); run('0.05')
    read32('F_ODR_A', 0x40020014); read32('F_ODR_B', 0x40020414)
    # Endstop reaction: homing X, the switch closes while cruising.
    pin('B', 12, True)
    mark('f_home'); send('G28 X\n'); run('0.3')
    pin('B', 12, False)
    mark('f_hit'); run('0.3')
    pin('B', 12, True)
    mark('f_backoff'); run('0.5')
    mark('f_home_pos'); send('M114\n'); run('0.05')

if want('dfu', in_all=False):                # ends the emulated firmware
    # M997: marker in RTC BKP0R, reset, then the jump into the system
    # bootloader (0x1FFF0000; Renode has no bootloader ROM there).
    mark('dfu'); send('M997\n'); run('0.3')
    cmd('echo "@@BKP0R"'); cmd('sysbus ReadDoubleWord 0x40002850')
    cmd('echo "@@VTOR"'); cmd('sysbus ReadDoubleWord 0xE000ED08')

if want('tune', in_all=False):               # ~200 s host, not part of 'all'
    # ---------------- PID autotune (2.6), simulated hotend ----------------
    cmd('sysbus WriteDoubleWord 0x%08X 1' % PLANT)
    mark('t_tune'); send('M303 E0 S100 C3 U1\n')
    for k in range(8):
        run('10.0')
    mark('t_hold'); send('M104 S100\n'); run('40.0')
    for k in range(5):
        mark('t_hold_%d' % k); send('M105\n'); run('2.0')
    # Part fan: 1.2 C/s extra loss at full speed, 0.3 C/s at S64. Feed-
    # forward F = 1.2 / 4 * 255 = 76.5 counts. Once without (the dip), once
    # with it, each from a settled hotend.
    cmd('sysbus WriteDoubleWord 0x%08X 1200' % FANLOSS)
    for ff in (0, 76.5):
        send('M301 F%g\n' % ff); run('20.0')
        mark('t_fan_%g' % ff); send('M106 S255\n')
        for k in range(30):
            send('M105\n'); run('0.5')
        mark('t_fan_end_%g' % ff); send('M107\n'); run('0.1')
    cmd('sysbus WriteDoubleWord 0x%08X 0' % FANLOSS)
    send('M104 S0\n'); run('0.1')
    cmd('sysbus WriteDoubleWord 0x%08X 0' % PLANT)

if want('safety'):
    # ---------------- Safety (section 1) ----------------
    def restart(name):                       # M999 in the halted state
        force(-1); mark(name); send('M999\n'); run('0.5')

    # M112 arrives while the firmware sits in G4 with a move running.
    mark('m112_setup'); send('M106 S255\nG1 X30 F600\nG4 P5000\n'); run('0.5')
    mark('m112'); send('M112\n'); run('0.1')
    mark('m112_after'); run('3.0')           # no steps, watchdog must not reset
    cmd('echo "@@TIM4 CCMR2"')
    cmd('sysbus ReadDoubleWord 0x4000081C')
    restart('m112_restart')

    # M108 cancels waiting for temperature.
    mark('m108_setup'); send('M104 S200\nM116\nG1 X1 F600\n'); run('1.0')
    mark('m108'); send('M108\n'); run('0.8')
    send('M104 S0\n'); run('0.1')

    # M410 quickstop keeps the position.
    mark('m410_move'); send('G1 X21 F600\n'); run('0.5')
    mark('m410'); send('M410\n'); run('0.3')
    mark('m410_pos'); send('M114\n'); run('0.05')
    mark('m410_back'); send('G1 X1 F600\n'); run('2.5')
    mark('m410_end'); send('M114\n'); run('0.05')

    # Heating failed: the temperature doesn't rise.
    force(100)
    mark('tp_heatfail'); send('M104 S100\n'); run('12.0')
    restart('tp_heatfail_restart')

    # Thermal runaway: target reached, then the temperature drops.
    mark('tp_runaway_heat'); send('M104 S40\n'); run('4.0')
    force(120)
    mark('tp_runaway'); run('12.0')
    restart('tp_runaway_restart')

    # MAXTEMP (heater off).
    force(1200)
    mark('tp_maxtemp'); run('1.5')
    restart('tp_maxtemp_restart')

    # MINTEMP (thermistor open) while heating.
    force(0)
    mark('tp_mintemp'); send('M104 S50\n'); run('1.5')
    restart('tp_mintemp_restart')

    # Sensor delivers no readings.
    force(0xFFFF)
    mark('tp_timeout'); send('M104 S50\n'); run('6.0')
    restart('tp_timeout_restart')

    # Target limited to MAXTEMP - overshoot.
    mark('tp_limit'); send('M104 S300\nM105\n'); run('0.1')
    send('M104 S0\n'); run('0.1')
    mark('end2')
cmd('quit')

script = artifact_path('teacup_test_%d.resc' % os.getpid())
open(script, 'w').write('\n'.join(lines) + '\n')
# Renode's console quits on EOF of stdin, so keep stdin open.
out = run_renode(RENODE, script)
out = re.sub(r'\x1b\[[0-9;]*m', '', out)
open(artifact_path('teacup_test_%s.log') % PORT, 'w').write(out)

# Split log into sections by markers.
sections, cur = {}, 'pre'
for l in out.splitlines():
    m = re.search(r'@+MARK (\S+)', l)
    if m and 'echo' not in l:
        cur = m.group(1); sections[cur] = []; continue
    sections.setdefault(cur, []).append(l)

def port_raw(sec, dev):
    return [re.sub(r'^.*\] ', '', l) for l in sections.get(sec, []) if '%s: [host' % dev in l]

def uart_raw(sec):                       # the host port under test
    return port_raw(sec, HOSTDEV)

def serial0(sec):                        # always the UART: boot messages
    return port_raw(sec, 'usart2')

ADV_OK = re.compile(r'^ok( N-?\d+)? P(\d+) B(\d+)$')
def uart(sec):                           # "ok N12 P7 B8" -> "ok"
    return ['ok' if ADV_OK.match(l) else l for l in uart_raw(sec)]

# With STEP_TIMER_PULSES the test board's step pins are driven by timers:
# X PA10 -> TIM1, Y PB3 -> TIM2, E PA6 -> TIM3. A step is a write of
# OPM | CEN (0x9) to the timer's CR1.
STEP_TIMERS = bool(re.search(r'echo:Step pulses: X TIM1 Y TIM2 Z gpio E TIM3', out))
TIMER_OF = {('A', 10): 'timer1', ('B', 3): 'timer2', ('A', 6): 'timer3'}

def pulses(sec, port, bit):
    if STEP_TIMERS and (port, bit) in TIMER_OF:
        t = TIMER_OF[(port, bit)]
        return sum(1 for l in sections.get(sec, [])
                   if re.search(r'%s: .*WriteUInt32 to 0x0 .*value 0x9\b' % t, l))
    n = 0
    for l in sections.get(sec, []):
        m = re.search(r'gpioPort%s: .*WriteUInt32 to 0x18 \(BitSet\), value 0x([0-9A-F]+)' % port, l)
        if m and int(m.group(1), 16) & (1 << bit):
            n += 1
    return n

def pos(sec):
    for l in uart(sec):
        m = re.match(r'X:(-?[\d.]+) Y:(-?[\d.]+) Z:(-?[\d.]+) E:(-?[\d.]+) Count', l)
        if m: return tuple(float(v) for v in m.groups())
    return None

fails = 0
def check(name, cond, info=''):
    global fails
    print('%s  %-38s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond: fails += 1

u = serial0('boot')
check('boot: start/ok', u and u[0] == 'start' and 'ok' in u, u)
if PORT == 'usb':
    print('--- host port: USB CDC ---')
    prot = [l for l in out.splitlines() if 'usb: PROTOCOL' in l]
    check('usb: enumerated, no protocol errors', 'HOST GET_LINE_CODING -> OK' in out and not prot, prot[:5])
if want('basic'):
    check('m115 firmware info', any('FIRMWARE_NAME:Teacup' in l for l in uart('m115')))
    if PORT == 'usb':
        check('usb: answers not on the UART', serial0('m115') == [] and serial0('burst') == [],
              serial0('m115') + serial0('burst'))
    check('m105 initial temp (ok T:..)', 'ok T:25.0/0.0 @:0' in uart('m105'), uart('m105'))
    check('m119 endstop open', any('x_min:open' in l for l in uart('m119_open')), uart('m119_open'))
    check('m119 endstop triggered', any('x_min:triggered' in l for l in uart('m119_trig')), uart('m119_trig'))
    n = pulses('move_x', 'A', 10) + pulses('m114_mid', 'A', 10)
    check('G1 X10: 400 X step pulses', n == 400, n)
    p = pos('m114_mid')
    check('M114 halfway at 0.55 s (5.0..5.8 mm)', p is not None and 5.0 < p[0] < 5.8, p)
    check('M114 after move X=10', pos('m114_end') == (10.0, 0.0, 0.0, 0.0), pos('m114_end'))
    nx, ny, ne = pulses('move_xye', 'A', 10), pulses('move_xye', 'B', 3), pulses('move_xye', 'A', 6)
    check('G1 X0 Y5 E2 steps (400/200/193)', (nx, ny) == (400, 200) and abs(ne - 193) <= 1, (nx, ny, ne))
    check('M114 after XYE move', pos('m114_xye') == (0.0, 5.0, 0.0, 2.0), pos('m114_xye'))
    ub = uart('burst')
    check('burst: 8 x ok', ub.count('ok') == 8, ub)
    check('burst: end position X8 Y13', pos('m114_burst') is not None and pos('m114_burst')[:2] == (8.0, 13.0), pos('m114_burst'))
    uh = uart('heat')
    check('M116: G1 waits for temperature', uh.count('ok') == 3 and 'Temp achieved' in uh
          and uh.index('Temp achieved') < len(uh) - 1 - uh[::-1].index('ok'), uh)
    check('M105 at target', any(l.startswith('ok T:30.') and '/30.0' in l for l in uart('m105_hot')), uart('m105_hot'))
    check('G28 X completes', 'ok' in uart('home'), uart('home'))
    ph = pos('m114_home')
    check('M114 after homing X=0', ph is not None and ph[0] == 0.0, ph)
    first = out[:out.find('@@MARK end')] if '@@MARK end' in out else out
    check('no reset (watchdog/fault)', first.count('] start') == 1, first.count('] start'))
    vals = re.findall(r'@+TIM4 (\w+)\s*\n\s*(0x[0-9A-Fa-f]+)', out)
    d = {k: int(v, 16) for k, v in vals}
    check('heater PWM TIM4 CH3: full power far below target', d.get('ARR') == 1019 and d.get('CCR3') == 1020, d)
if want('proto'):
    print('--- host protocol / command queue ---')
    r = uart_raw('p_m110')
    check('N0 M110 N0 accepted', any(re.match(r'^ok N0 P\d+ B\d+$', l) for l in r), r)
    r = uart_raw('p_n1')
    check('N1 G1: ok N1 P.. B..', any(re.match(r'^ok N1 P[0-7] B[0-8]$', l) for l in r), r)
    check('checksum mismatch -> Resend: 2', uart('p_badcs') == ['Error:checksum mismatch, Last Line: 1', 'Resend: 2', 'ok'], uart('p_badcs'))
    check('wrong line number -> Resend: 2', uart('p_badn') == ['Error:Line Number is not Last Line Number+1, Last Line: 1', 'Resend: 2', 'ok'], uart('p_badn'))
    check('N without checksum -> Resend: 2', uart('p_nocs') == ['Error:No Checksum with line number, Last Line: 1', 'Resend: 2', 'ok'], uart('p_nocs'))
    check('checksum without N -> Resend: 2', uart('p_csnon') == ['Error:No Line Number with checksum, Last Line: 1', 'Resend: 2', 'ok'], uart('p_csnon'))
    M105_OK = re.compile(r'^ok T:[\d.]+/[\d.]+ @:\d+$')
    check('resent line accepted, M105 one line', len(uart('p_resend')) == 1 and M105_OK.match(uart('p_resend')[0]), uart('p_resend'))
    r = uart_raw('p_after110')
    check('M110 N100 -> N101 accepted', any(l.startswith('X:') for l in r) and any(re.match(r'^ok N101 ', l) for l in r), r)
    check('comment stripped, no halt', len(uart('p_comment')) == 1 and M105_OK.match(uart('p_comment')[0]), uart('p_comment'))
    check('unknown command', uart('p_unknown') == ['echo:Unknown command: "M9876"', 'ok'], uart('p_unknown'))
    ub = uart('p_busy')
    nb = ub.count('echo:busy: processing')
    check('long command: busy every 2 s', nb >= 1 and nb <= 3 and ub[-1] == 'ok' and ub.count('ok') == 3, ub)
    check('M113 S0: no busy', 'echo:busy: processing' not in uart('p_m113') and uart('p_m113').count('ok') >= 4, uart('p_m113'))
    oks = [ADV_OK.match(l) for l in uart_raw('p_stream')]
    oks = [m for m in oks if m]
    ns = [int(m.group(1)[2:]) for m in oks]
    check('stream 12 lines: 12 ok N102..113', ns == list(range(102, 114)), ns)
    check('stream: P/B within limits', all(0 <= int(m.group(2)) <= 7 and 0 <= int(m.group(3)) <= 8 for m in oks),
          [(m.group(2), m.group(3)) for m in oks])
    check('stream: end position X13', pos('p_stream_pos') is not None and pos('p_stream_pos')[0] == 13.0, pos('p_stream_pos'))

if want('cmds'):
    print('--- G/M commands ---')
    r = uart('c_m114')
    check('M114 Marlin format', any(re.match(r'^X:-?[\d.]+ Y:-?[\d.]+ Z:-?[\d.]+ E:-?[\d.]+ Count X:-?\d+ Y:-?\d+ Z:-?\d+$', l) for l in r), r)
    check('M109 S40 blocks while heating', 'ok' not in uart('c_m109_start'), uart('c_m109_start'))
    check('M109 S40 returns when reached', 'ok' in uart('c_m109_done'), uart('c_m109_done'))
    t = [l for l in uart('c_m109_hot') if l.startswith('ok T:')]
    check('  temperature at target', t and 38.0 <= float(t[0][5:].split('/')[0]) <= 42.0, t)
    check('M109 S30 when hotter: no wait', 'ok' in uart('c_m109_noheat'), uart('c_m109_noheat'))
    check('M109 R30 waits for cooling', 'ok' not in uart('c_m109_r_start') and 'ok' in uart('c_m109_r_done'),
          (uart('c_m109_r_start'), uart('c_m109_r_done')))
    u = uart('c_m109_cancel')
    check('M108 cancels M109', u.count('ok') >= 2 and 'echo:Wait for temperature cancelled' in u, u)
    v = {k: int(x, 16) for k, x in re.findall(r'@+(\w+)\s*\n\s*(0x[0-9A-Fa-f]+)', out)}
    # GPIO reads: the monitor output may interleave with log lines, take the
    # logged peripheral access right after the marker instead.
    for tag in ('ODR_M17', 'ODR_M84', 'ODR_IDLE0', 'ODR_IDLE1'):
        m = re.search(r'@+%s.*?OutputData\), returned (0x[0-9A-Fa-f]+)' % tag, out, re.S)
        if m:
            v[tag] = int(m.group(1), 16)
    fan_ccr = lambda s: (20 + (s * 235 + 127) // 255) * 256 * 1020 // (255 * 256)
    check('M106 S200 / M107 (fan, TIM4 CH4)', v.get('CCR4_ON', 0) == fan_ccr(200) and v.get('CCR4_OFF', 1) == 0, (v.get('CCR4_ON'), v.get('CCR4_OFF')))
    check('  kick-start: full power first', v.get('CCR4_KICK', 0) == 1020, v.get('CCR4_KICK'))
    check('  running fan: new speed at once', v.get('CCR4_LOWER', 0) == fan_ccr(100), v.get('CCR4_LOWER'))
    check('  M106 S1: minimum PWM 20', v.get('CCR4_MIN', 0) == fan_ccr(1), (v.get('CCR4_MIN'), fan_ccr(1)))
    check('M17 enables (PA9 low)', (v.get('ODR_M17', 0xFFFF) >> 9) & 1 == 0, hex(v.get('ODR_M17', 0)))
    check('M84 disables (PA9 high)', (v.get('ODR_M84', 0) >> 9) & 1 == 1, hex(v.get('ODR_M84', 0)))
    check('M84 S2: idle timeout disables', (v.get('ODR_IDLE0', 0xFFFF) >> 9) & 1 == 0 and (v.get('ODR_IDLE1', 0) >> 9) & 1 == 1,
          (hex(v.get('ODR_IDLE0', 0)), hex(v.get('ODR_IDLE1', 0))))
    p4 = pos('c_m400')
    check('M400 waits for the move', p4 is not None and p4[0] == 10.0, p4)
    mb = re.search(r'@+MSG\s*\[\s*([0-9A-Fa-fx,\s]+)\]', out)
    msg = ''
    if mb:
        for b in re.findall(r'0x([0-9A-Fa-f]{2})', mb.group(1)):
            if b == '00': break
            msg += chr(int(b, 16))
    check('M117 message stored', msg == 'Hello X99 World', repr(msg))
    check('M117 text does not move X', pulses('c_m117', 'A', 10) == 0 and pulses('c_m117', 'B', 3) == 80,
          (pulses('c_m117', 'A', 10), pulses('c_m117', 'B', 3)))
    check('M73 progress', 'echo:Progress: 42%, remaining 17 min' in uart('c_m73'), uart('c_m73'))
    p6 = pos('c_m206')
    check('M206 X5 shifts coordinates', p6 is not None and p6[0] == 15.0, p6)
    p6b = pos('c_m206_g1')
    check('  next move does not go to X5', pulses('c_m206_g1', 'A', 10) == 0 and p6b and p6b[0] == 15.0, (pulses('c_m206_g1', 'A', 10), p6b))
    ph = pos('c_home_pos')
    check('G28 X with offset: X=5', ph is not None and ph[0] == 5.0 and 'echo:M206 X5.000 Y0.000 Z0.000' in uart('c_home_pos'),
          uart('c_home_pos'))
    p8 = pos('c_m428')
    check('M428: current position becomes 0', p8 is not None and p8[0] == 0.0 and p8[1] == 0.0, (p8, uart('c_m428')))
    p0 = pos('c_m206_0')
    check('M206 X0 restores coordinates', p0 is not None and p0[0] == 3.0, p0)
    check('M300 without beeper: waits P ms', 'ok' not in uart('c_m300') and 'ok' in uart('c_m300_done'),
          (uart('c_m300'), uart('c_m300_done')))

def m503_line(sec, cmdname):
    for l in uart(sec):
        if l.startswith('echo:  ' + cmdname + ' '):
            return l[7:]
    return None

if want('settings'):
    print('--- settings / Flash ---')
    check('boot without stored settings', any('No stored settings' in l for l in serial0('boot') + serial0('s_boot_check')) or PART == 'all',
          serial0('boot'))
    check('M503 defaults', m503_line('s_defaults', 'M92') == 'M92 X40.00 Y40.00 Z320.00 E96.27'
          and m503_line('s_defaults', 'M203') == 'M203 X100.00 Y100.00 Z3.33 E100.00', (m503_line('s_defaults', 'M92'), m503_line('s_defaults', 'M203')))
    check('M92 X80: 10 mm = 800 steps', pulses('s_m92', 'A', 10) == 800 and pos('s_m92_pos')[0] == 10.0,
          (pulses('s_m92', 'A', 10), pos('s_m92_pos')))
    pm = pos('s_m203_mid')
    check('M203 X5: 5 mm/s', pm is not None and 14.0 <= pm[0] <= 16.0, pm)
    c = uart('s_changed')
    # M204 S sets printing and travel, R keeps the default (ACCELERATION).
    check('M204/M205/M201/M301 reported', m503_line('s_changed', 'M204') == 'M204 P500.00 R1000.00 T500.00'
          and m503_line('s_changed', 'M205').startswith('M205 X15.00')
          and m503_line('s_changed', 'M201').endswith('Z80.00 E1000.00')
          and m503_line('s_changed', 'M301') == 'M301 P10.00 I0.50 D50.00 F12.50', c)
    check('M204 P R T set separately', m503_line('s_m204_changed', 'M204') == 'M204 P700.00 R3000.00 T800.00',
          m503_line('s_m204_changed', 'M204'))
    check('M500 stores', any(l.startswith('echo:Settings Stored') for l in uart('s_m500')), uart('s_m500'))
    check('M502 defaults', 'echo:Hardcoded Default Settings Loaded' in uart('s_m502')
          and m503_line('s_after502', 'M92').startswith('M92 X40.00'), m503_line('s_after502', 'M92'))
    check('M501 loads', any('Stored settings retrieved' in l for l in uart('s_m501'))
          and m503_line('s_after501', 'M92').startswith('M92 X80.00')
          and m503_line('s_after501', 'M301') == 'M301 P10.00 I0.50 D50.00 F12.50'
          and m503_line('s_after501', 'M204') == 'M204 P700.00 R3000.00 T800.00',
          (m503_line('s_after501', 'M92'), m503_line('s_after501', 'M204')))
    # Boot messages: UART only (USB isn't enumerated yet at boot).
    check('after reset: loaded from Flash', any('Stored settings retrieved' in l for l in serial0('s_reset'))
          and m503_line('s_after_reset', 'M92').startswith('M92 X80.00')
          and m503_line('s_after_reset', 'M204') == 'M204 P700.00 R3000.00 T800.00',
          (uart('s_reset'), m503_line('s_after_reset', 'M92'), m503_line('s_after_reset', 'M204')))
    check('M31 while paused: 2 s', 'echo:Print time: 2s' in uart('j_paused'), uart('j_paused'))
    uj = uart('j_stopped')
    check('M77, M31: 3 s', 'echo:Print time: 3s' in uj, uj)
    check('M78: 1 print, finished, 3 s', 'echo:Stats: Prints: 1, Finished: 1, Failed: 0' in uj
          and 'echo:Stats: Total time: 3s, Longest job: 3s' in uj and 'echo:Stats: Filament used: 0.00m' in uj, uj)
    check('  statistics survive a reset', 'echo:Stats: Prints: 1, Finished: 1, Failed: 0' in uart('j_after_reset'), uart('j_after_reset'))
    check('  ... and the erase of the full sector; unfinished print failed',
          'echo:Stats: Prints: 3, Finished: 2, Failed: 1' in uart('j_after_many'), uart('j_after_many'))
    check('  filament used while the timer runs: 30 mm', 'echo:Stats: Filament used: 0.03m' in uart('j_filament'), uart('j_filament'))
    check('10 saves (erase), newest survives reset', m503_line('s_after_many', 'M92') == 'M92 X80.00 Y50.00 Z320.00 E96.27',
          m503_line('s_after_many', 'M92'))

if want('slicer'):
    print('--- PrusaSlicer / Pronterface ---')
    allu = []
    for sec in sections:
        if sec.startswith('ps_'):
            allu += uart(sec)
    errors = [l for l in allu if l.startswith('Error:')]
    check('only the provoked checksum error', errors == ['Error:checksum mismatch, Last Line: 19'], errors)
    check('resend requested and accepted', 'Resend: 20' in uart('ps_resend'), uart('ps_resend'))
    unknown = sorted(set(l for l in allu if 'Unknown command' in l))
    # M900 is known with LINEAR_ADVANCE.
    check('unknown: only M862.3 and M900', unknown in (['echo:Unknown command: "M8623"', 'echo:Unknown command: "M900"'],
                                                      ['echo:Unknown command: "M8623"']), unknown)
    oks = sum(1 for l in allu if l == 'ok' or l.startswith('ok T:'))
    check('one ok per line (+ M110, resend, M105)', oks == len(SLICER_GCODE) + 3, (oks, len(SLICER_GCODE) + 3))
    check('temperatures reported while M109 waits', any(l.startswith('T:') for l in allu), '')
    check('M73 Q/S silent', not any('Progress' in l for l in allu), '')
    check('M84 X Y E: no X/Y move afterwards', pulses('ps_m84', 'A', 10) == 0 and pulses('ps_m84', 'B', 3) == 0,
          (pulses('ps_m84', 'A', 10), pulses('ps_m84', 'B', 3)))
    pe = pos('ps_m84') or pos('ps_end')
    check('final position X106.854 Y96.803 Z11', pe is not None and pe[:3] == (106.854, 96.803, 11.0), pe)
    mb = re.search(r'@+PSMSG\s*\[\s*([0-9A-Fa-fx,\s]+)\]', out)
    msg = ''
    if mb:
        for b in re.findall(r'0x([0-9A-Fa-f]{2})', mb.group(1)):
            if b == '00': break
            msg += chr(int(b, 16))
    check('M117 Layer 2', msg == 'Layer 2', repr(msg))

if want('perf'):
    print('--- step interrupt / endstops ---')
    st = [l for l in uart('f_stats') if l.startswith('echo:Step IRQ')]
    m = re.search(r'n (\d+), cycles min (\d+) avg (\d+) max (\d+), latency max (\d+), late (\d+), pulses (\d+)', st[0]) if st else None
    # Pulse end interrupts only for GPIO steps (none with timer pulses on
    # all axes of the test board). With linear advance or input shaping,
    # pulses which don't come right after a step get a pulse end of their own.
    if STEP_AUX and not STEP_TIMERS:
        ok = bool(m) and int(m.group(1)) >= 800 and int(m.group(7)) >= int(m.group(1))
    else:
        ok = bool(m) and int(m.group(1)) >= 800 and \
             int(m.group(7)) == (0 if STEP_TIMERS else int(m.group(1)))
    check('M9001: every step gets a pulse end', ok, st)
    check('  late steps', bool(m) and int(m.group(6)) == 0, m.group(6) if m else None)
    check('  multi-stepping setting reported', any(l.startswith('echo:Multi-stepping:') for l in uart('f_stats')), uart('f_stats'))
    nx = pulses('f_move', 'A', 10)
    rx = 0
    for l in sections.get('f_move', []):
        mm = re.search(r'gpioPortA: .*WriteUInt32 to 0x18 \(BitSet\), value 0x([0-9A-F]+)', l)
        if mm and int(mm.group(1), 16) & (1 << 26): rx += 1
    if STEP_TIMERS:
        check('X: 800 timer pulses, no GPIO pulses', nx == 800 and rx == 0, (nx, rx))
    else:
        # With linear advance or shaping, pulse ends of other pulses lower X, too.
        check('X: 800 pulses, each one ended', nx == 800 and
              (rx >= nx if STEP_AUX else rx == nx), (nx, rx))
    va = {}
    for tag in ('F_ODR_A', 'F_ODR_B'):
        mm = re.search(r'@+%s.*?OutputData\), returned (0x[0-9A-Fa-f]+)' % tag, out, re.S)
        if mm: va[tag] = int(mm.group(1), 16)
    check('step pins low after moves', 'F_ODR_A' in va and va['F_ODR_A'] & ((1 << 10) | (1 << 6)) == 0
          and va.get('F_ODR_B', 1) & (1 << 3) == 0, {k: hex(v) for k, v in va.items()})
    # Count X steps after the switch closed, until the back-off move starts
    # (X_DIR on PB4 changes).
    after = 0
    for l in sections.get('f_hit', []):
        if STEP_TIMERS and re.search(r'timer1: .*WriteUInt32 to 0x0 .*value 0x9\b', l):
            after += 1
            continue
        mm = re.search(r'gpioPort([AB]): .*WriteUInt32 to 0x18 \(BitSet\), value 0x([0-9A-F]+)', l)
        if not mm: continue
        v = int(mm.group(2), 16)
        if mm.group(1) == 'B' and v & ((1 << 4) | (1 << 20)): break
        if mm.group(1) == 'A' and v & (1 << 10) and not STEP_TIMERS: after += 1
    # Homing speed sqrt(2 * 1000 * 1 mm) = 44.7 mm/s, decelerating to 0
    # takes 1 mm = 40 steps. Polling with ENDSTOP_STEPS 4 would add 8 ms,
    # about 15 steps.
    check('endstop interrupt: stop within decel distance', 30 <= after <= 44, after)
    ph = pos('f_home_pos')
    check('  homing completes, X=0', ph is not None and ph[0] == 0.0, ph)

if want('dfu', in_all=False):
    print('--- M997 ---')
    check('M997: message, then reset', 'echo:Rebooting into the DFU bootloader' in uart('dfu'), uart('dfu'))
    vals = dict(re.findall(r'@@(BKP0R|VTOR)\s*\n\s*(0x[0-9A-Fa-f]+)', out))
    # (Renode's SYSCFG doesn't keep MEMRMP, so it isn't checked.) VTOR
    # 0x1FFF0000 is set only after reading the marker at startup.
    check('  bootloader entered: marker read and cleared, VTOR 0x1FFF0000',
          int(vals.get('BKP0R', '1'), 16) == 0 and int(vals.get('VTOR', '0'), 16) == 0x1FFF0000, vals)

if want('tune', in_all=False):
    print('--- PID autotune ---')
    u = uart('t_tune')
    fin = [l for l in u if 'PID Autotune finished' in l]
    vals = {}
    for l in u:
        m = re.match(r'#define DEFAULT_(Kp|Ki|Kd) ([\d.]+)', l)
        if m: vals[m.group(1)] = float(m.group(2))
    check('M303 finishes', bool(fin), [l for l in u if 'Autotune' in l or 'bias' in l][:12])
    check('  Kp, Ki, Kd plausible', len(vals) == 3 and 1 < vals['Kp'] < 200 and 0 < vals['Ki'] < 50 and 0 < vals['Kd'] < 2000, vals)
    check('  applied (U1)', 'echo:PID values applied, M500 to store them' in u, '')
    temps = []
    for k in range(5):
        for l in uart('t_hold_%d' % k):
            m = re.match(r'ok T:([\d.]+)/', l)
            if m: temps.append(float(m.group(1)))
    check('tuned PID holds 100 C (+-2)', len(temps) == 5 and all(abs(t - 100) <= 2.0 for t in temps), temps)
    dips = {}
    for ff in (0, 76.5):
        t = [float(m.group(1)) for l in uart('t_fan_%g' % ff)
             for m in [re.match(r'ok T:([\d.]+)/', l)] if m]
        dips[ff] = (100 - min(t)) if t else 99
    check('fan on: hotend dips without feed-forward (> 1.5 C)', dips[0] > 1.5, dips)
    check('  M301 F: dip below 0.5 C', dips[76.5] < 0.5, dips)

if want('safety'):
    print('--- safety ---')
    def kills(sec, text):
        u = uart(sec)
        return any(text in l for l in u) and 'Error:Printer halted. kill() called!' in u
    def restarted(sec):
        return 'start' in serial0(sec)
    check('M112 during G4: halted', kills('m112', 'Error:Emergency stop (M112)'), uart('m112'))
    check('M112: no steps after halt', pulses('m112_after', 'A', 10) == 0, pulses('m112_after', 'A', 10))
    check('M112: no watchdog reset while halted', 'start' not in serial0('m112_after'), serial0('m112_after'))
    cc = int(re.search(r'@+TIM4 CCMR2\s*\n\s*(0x[0-9A-Fa-f]+)', out).group(1), 16)
    check('M112: heater PWM forced inactive', (cc >> 4) & 7 == 4, hex(cc))
    check('M999 restarts', restarted('m112_restart'), uart('m112_restart'))
    u = uart('m108_setup')
    check('M108 setup: G1 waits, temps reported', u.count('ok') == 2 and any(l.startswith('T:') for l in u), u)
    check('M108: wait cancelled, G1 runs', 'echo:Wait for temperature cancelled' in uart('m108')
          and uart('m108').count('ok') >= 2 and pulses('m108', 'A', 10) == 40,
          (uart('m108'), pulses('m108', 'A', 10)))
    fwd = pulses('m410_move', 'A', 10) + pulses('m410', 'A', 10)
    back = pulses('m410_back', 'A', 10)
    p410 = pos('m410_pos')
    check('M410: stopped mid move', 0 < fwd < 800 and p410 is not None and abs(p410[0] - (1 + fwd / 40.0)) < 0.03,
          (fwd, p410))
    check('M410: position kept (back = forth)', back == fwd and pos('m410_end')[0] == 1.0, (fwd, back, pos('m410_end')))
    check('Heating failed detected', kills('tp_heatfail', 'Error:Heating failed, system stopped! Heater_ID: 0'), uart('tp_heatfail'))
    check('  restart', restarted('tp_heatfail_restart'))
    check('Runaway: normal heating passes', not any('Error' in l for l in uart('tp_runaway_heat')), uart('tp_runaway_heat'))
    check('Thermal runaway detected', kills('tp_runaway', 'Error:Thermal Runaway, system stopped! Heater_ID: 0'), uart('tp_runaway'))
    check('  restart', restarted('tp_runaway_restart'))
    check('MAXTEMP detected (heater off)', kills('tp_maxtemp', 'Error:MAXTEMP triggered'), uart('tp_maxtemp'))
    check('  restart', restarted('tp_maxtemp_restart'))
    check('MINTEMP detected', kills('tp_mintemp', 'Error:MINTEMP triggered'), uart('tp_mintemp'))
    check('  restart', restarted('tp_mintemp_restart'))
    check('Sensor timeout detected', kills('tp_timeout', 'Error:Temperature sensor timeout'), uart('tp_timeout'))
    check('  restart', restarted('tp_timeout_restart'))
    u = uart('tp_limit')
    check('Target limited to 260 C', 'echo:Target temperature limited by MAXTEMP' in u and any('/260.0' in l for l in u), u)

print('\n%d failure(s)' % fails)
sys.exit(1 if fails else 0)
