"""Mechanics model of the P3 Steel for the Renode tests (run_features.py).

Loaded into Renode's Python by watchpoint hooks on the step outputs:
  X step  TIM1 CR1 = OPM|CEN (0x9), DIR PA9
  Y step  TIM2 CR1 = 0x9,           DIR PA10
  Z step  GPIOB BSRR bit 12 set,    DIR PB13 (1 = up)
  E step  TIM3 CR1 = 0x9,           DIR PB5 (inverted: 0 = forward)
It tracks the physical nozzle position and drives the inputs:
  X_MIN PB10, Y_MIN PB3 (active low switches at X = 0 / Y = 0),
  BLTouch signal PB15 (active high), servo commands from TIM9 CCR1.
The bed is a tilted plane, bed(x, y), in the same physical frame.
"""

SPM = {'x': 40.0, 'y': 40.0, 'z': 400.0, 'e': 100.0}
pos = {'x': 50.0, 'y': 60.0, 'z': 8.0, 'e': 0.0}
PROBE = {'dx': -30.0, 'dy': -10.0, 'h': 1.5}    # tip below the nozzle by h
servo = {'us': 0, 'deployed': False, 'triggered': False, 'cmds': [],
         'pin_set': 0, 'pin_reset': 0}
pins = {}
track = {'on': False, 'gap': 0.0, 'min': 1e9, 'max': -1e9, 'n': 0}
triggers = []
zsteps = {'up': 0, 'down': 0}
m = None

# Tilted plane plus a bump: 0.15 mm at the middle node of the 3x3 mesh
# G29 probes (X 15/102.5/190, Y 15/90/165), bilinear in each mesh cell.
# The mesh reproduces this bed exactly, but only if moves are split at
# the grid lines; without splitting Z would cut through the bump.
def bed(x, y):
    tx = max(0.0, 1.0 - abs(x - 102.5) / 87.5)
    ty = max(0.0, 1.0 - abs(y - 90.0) / 75.0)
    return 0.10 + 0.002 * x - 0.0015 * y + 0.15 * tx * ty

def setup(spm=None):
    if spm:
        SPM.update(spm)

def _pin(port, n, level):
    key = (port, n)
    if pins.get(key) == level:
        return
    pins[key] = level
    m['sysbus.gpioPort%s' % port].OnGPIO(n, level)

def _odr(port):
    base = {'A': 0x40020000, 'B': 0x40020400, 'C': 0x40020800}[port]
    return m.SystemBus.ReadDoubleWord(base + 0x14)

def _endstops():
    _pin('B', 10, not (pos['x'] <= 0.0))       # pressed = low
    _pin('B', 3, not (pos['y'] <= 0.0))

def _track():
    if track['on']:
        g = pos['z'] - bed(pos['x'], pos['y'])
        track['min'] = min(track['min'], g)
        track['max'] = max(track['max'], g)
        track['n'] += 1

def _release():
    if servo['triggered']:
        servo['triggered'] = False
        _pin('B', 15, False)

def init(machine):
    global m
    m = machine
    _endstops()
    _pin('B', 15, False)

def step_x(machine, value):
    global m
    m = machine
    if value != 0x9:
        return
    d = 1 if (_odr('A') >> 9) & 1 else -1
    pos['x'] += d / SPM['x']
    _endstops()
    _track()

def step_y(machine, value):
    global m
    m = machine
    if value != 0x9:
        return
    d = 1 if (_odr('A') >> 10) & 1 else -1
    pos['y'] += d / SPM['y']
    _endstops()
    _track()

def step_e(machine, value):
    global m
    m = machine
    if value != 0x9:
        return
    d = -1 if (_odr('B') >> 5) & 1 else 1
    pos['e'] += d / SPM['e']

def gpiob_bsrr(machine, value):
    global m
    m = machine
    if not (value & (1 << 12)):
        return
    up = (_odr('B') >> 13) & 1
    if up:
        pos['z'] += 1 / SPM['z']
        zsteps['up'] += 1
        _release()
    else:
        pos['z'] -= 1 / SPM['z']
        zsteps['down'] += 1
        if servo['deployed'] and not servo['triggered']:
            tip = pos['z'] - PROBE['h']
            if tip <= bed(pos['x'] + PROBE['dx'], pos['y'] + PROBE['dy']):
                servo['deployed'] = False          # BLTouch pulls the pin in
                servo['triggered'] = True
                triggers.append((pos['x'] + PROBE['dx'], pos['y'] + PROBE['dy'], pos['z']))
                _pin('B', 15, True)
    _track()

def servo_ccr(machine, value):
    global m
    m = machine
    servo['us'] = value
    servo['cmds'].append(value)
    _release()
    if 600 <= value <= 700:
        servo['deployed'] = True
    elif 1400 <= value <= 1550 or 2100 <= value <= 2300:
        servo['deployed'] = False

def gpioc_bsrr(machine, value):
    if value & (1 << 13):
        servo['pin_set'] += 1
    if value & (1 << (13 + 16)):
        servo['pin_reset'] += 1

def track_start(gap):
    track.update({'on': True, 'gap': gap, 'min': 1e9, 'max': -1e9, 'n': 0})

def track_stop():
    track['on'] = False

def report(tag):
    print('@@MODEL %s x=%.4f y=%.4f z=%.4f e=%.4f gap=%.4f pgap=%.4f tmin=%.4f tmax=%.4f tn=%d '
          'trig=%d servo=%s pset=%d preset=%d zup=%d zdown=%d' % (
          tag, pos['x'], pos['y'], pos['z'], pos['e'],
          pos['z'] - bed(pos['x'], pos['y']),
          pos['z'] - bed(pos['x'] + PROBE['dx'], pos['y'] + PROBE['dy']), track['min'], track['max'], track['n'],
          len(triggers), ','.join(str(c) for c in servo['cmds'][-12:]),
          servo['pin_set'], servo['pin_reset'], zsteps['up'], zsteps['down']))
