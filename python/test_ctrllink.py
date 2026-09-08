"""Exercises the host module against a byte-faithful simulation of the sketch.

The fake reproduces what CtrlLink.cpp actually puts on the wire, CRLF on
println() lines and bare LF on telemetry rows included, so the decoder is tested
against the real framing rather than an idealised version of it.
"""
import sys, time, struct
sys.path.insert(0, __import__("os").path.dirname(__file__) or ".")

import numpy as np

# name -> (wire type, fractional bits, stored raw value). `kq` and `alpha` are
# kept in fixed point the way ControlDemo keeps its gains, so the host's
# conversion is exercised rather than assumed.
PARAMS = {'dec': ('u16', 0, 1), 'kp': ('f32', 0, 0.5), 'ki': ('f32', 0, 0.0),
          'ref': ('i16', 0, 0), 'mode': ('u8', 0, 0),
          'kq': ('i32', 22, 0), 'alpha': ('i32', 16, 65536),
          # Health counters, as ControlDemo declares them. The host discovers
          # these by name and zeroes them before every capture.
          'missed': ('u16', 0, 0), 'maxlate': ('u16', 0, 0),
          'sovr': ('u16', 0, 0), 'serr': ('u16', 0, 0)}
CHANS = [('ref', 'i16', 0.0878906, 'deg'), ('y', 'i16', 0.0878906, 'deg'),
         ('e', 'i16', 0.0878906, 'deg'), ('u', 'i16', 1.0, 'pwm')]
WIDTH = {'i16': 4, 'u16': 4, 'u8': 2, 'i32': 8, 'f32': 8}


class FakeUno:
    """Device-side state machine; `out` is what the host would read."""

    def __init__(self, rate_hz=1000, start_tick=0, unhealthy=None, drops=0):
        self.out = bytearray()
        self.line = bytearray()
        self.params = dict(PARAMS)
        self.streaming = False
        self.tick = start_tick
        self.rows = 0          # rows actually written, as CtrlLink counts them
        self.produced = 0      # control periods elapsed, decimated away or not
        self.rate = rate_hz
        self.t0 = None
        self.y = 0
        # Counter values the device "discovers" once a run is under way, so a
        # capture that zeroes them first still finds them non-zero at the end.
        self.unhealthy = unhealthy or {}
        self.drops = drops

    def println(self, s=''):
        self.out += (s + '\r\n').encode()   # Arduino println appends CRLF

    def feed(self, data):
        for byte in data:
            if byte == 0x0A:
                self.command(bytes(self.line).decode().strip())
                self.line.clear()
            else:
                self.line.append(byte)

    def command(self, cmd):
        head, _, rest = cmd.partition(' ')
        if head == 'id':
            self.println('# id CtrlLink 1 ControlDemo chans=4 row=21 dt_us=1000')
            self.println('# ok')
        elif head == 'params':
            for name, (type_, frac, value) in self.params.items():
                shown = f'{value:.6f}' if type_ == 'f32' else str(value)
                self.println(f'# p {name} {type_} {frac} {shown}')
            self.println('# ok')
        elif head == 'chans':
            for i, (name, type_, scale, unit) in enumerate(CHANS):
                self.println(f'# c {i} {name} {type_} {scale:.7f} {unit}')
            self.println('# ok')
        elif head == 'get':
            self._show(rest)
            self.println('# ok')
        elif head == 'set':
            name, _, value = rest.partition(' ')
            type_, frac, _old = self.params[name]
            self.params[name] = (type_, frac,
                                 float(value) if type_ == 'f32' else int(value))
            if self.streaming:
                self._pump()   # the mark lands on the tick reached so far
                type_, frac, value = self.params[name]
                shown = f'{value:.6f}' if type_ == 'f32' else str(value)
                self.println(f'# mark {self.tick} {name} {shown}')
            self._show(name)
            self.println('# ok')
        elif head == 'start':
            self.rows = 0
            self.produced = 0
            self.println('# begin')
            self.println(f'# rate dt_us=1000 dec={self.params["dec"][2]}')
            self.println('# col tick u16 1 tick')
            for name, type_, scale, unit in CHANS:
                self.println(f'# col {name} {type_} {scale:.7f} {unit}')
            self.println('# data')
            self.streaming = True
            self.t0 = time.monotonic()
            for name, value in self.unhealthy.items():
                type_, frac, _ = self.params[name]
                self.params[name] = (type_, frac, value)
        elif head == 'stop':
            self._pump()
            self.streaming = False
            self.println(f'# end rows={self.rows} drops={self.drops}')
            self.println('# ok')
        else:
            self.println('# err unknown command')

    def _show(self, name):
        type_, _frac, value = self.params[name]
        shown = f'{value:.6f}' if type_ == 'f32' else str(value)
        self.println(f'# v {name} {shown}')

    def _pump(self):
        """Emits the rows that should have been produced by now."""
        if not self.streaming:
            return
        due = int((time.monotonic() - self.t0) * self.rate)
        dec = self.params['dec'][2]
        while self.produced < due:
            ref = self.params['ref'][2]
            self.y += (ref - self.y) // 8          # visibly first-order
            err = ref - self.y
            u = max(-255, min(255, err // 4))
            if self.tick % dec == 0:
                row = ''.join(f'{v & 0xFFFF:04X}'
                              for v in (self.tick, ref, self.y, err, u))
                self.out += (row + '\n').encode()  # rows use a bare LF
                self.rows += 1
            self.tick = (self.tick + 1) & 0xFFFF
            self.produced += 1


class FakeSerial:
    def __init__(self, uno):
        self.uno = uno
        self.is_open = True
        self.pos = 0

    @property
    def in_waiting(self):
        self.uno._pump()
        return len(self.uno.out) - self.pos

    def write(self, data):
        self.uno._pump()
        self.uno.feed(data)
        return len(data)

    def read(self, n=1):
        self.uno._pump()
        chunk = bytes(self.uno.out[self.pos:self.pos + n])
        self.pos += len(chunk)
        return chunk

    def readline(self):
        for _ in range(400):
            self.uno._pump()
            nl = self.uno.out.find(b'\n', self.pos)
            if nl >= 0:
                line = bytes(self.uno.out[self.pos:nl + 1])
                self.pos = nl + 1
                return line
            time.sleep(0.001)
        return b''

    def reset_input_buffer(self):
        self.uno._pump()
        self.pos = len(self.uno.out)

    def flush(self):
        pass

    def close(self):
        self.is_open = False


def connect(uno):
    import ctrllink
    dev = ctrllink.CtrlLink.__new__(ctrllink.CtrlLink)
    dev.ser = FakeSerial(uno)
    dev.info = dev.sync()
    dev._params = dev._read_params()
    dev.channels = dev._read_channels()
    # Lets a check look at what the device actually stored, rather than at what
    # the host reports after scaling it back.
    dev._uno_raw = lambda name: uno.params[name][2]
    return dev


failures = []


def _raises(call, kind):
    try:
        call()
    except kind:
        return True
    except Exception:
        return False
    return False


def check(label, condition, detail=''):
    print(f'{"PASS" if condition else "FAIL"}  {label}' + (f'  -- {detail}' if detail and not condition else ''))
    if not condition:
        failures.append(label)


# ---------------------------------------------------------------- discovery
dev = connect(FakeUno())
check('id parsed', dev.info.startswith('CtrlLink 1 ControlDemo'), dev.info)
check('params discovered', set(dev._params) == set(PARAMS), str(dev._params))
check('channels discovered', [c.name for c in dev.channels] == ['ref', 'y', 'e', 'u'])
check('float param typed', dev._params['kp'].type == 'f32')
check('param format discovered', dev._params['kq'].frac == 22
      and dev._params['kq'].scale == 2.0 ** -22, str(dev._params['kq']))

# ------------------------------------------------------------- param access
dev.kp = 2.5
check('float set/get round trip', abs(dev.kp - 2.5) < 1e-6, str(dev.kp))
dev.ref = 1024
check('int set/get round trip', dev.ref == 1024, str(dev.ref))
check('int stays int', isinstance(dev.ref, int))
check('params snapshot', dev.params['kp'] == 2.5, str(dev.params))
try:
    dev.nonexistent
    check('unknown attribute raises', False)
except AttributeError:
    check('unknown attribute raises', True)

# ------------------------------------------------------------------ capture
dev.ref = 0
df = dev.capture(0.30)
check('capture returned rows', len(df) > 100, f'{len(df)} rows')
check('columns as declared', list(df.columns) == ['t', 'ref', 'y', 'e', 'u'], str(list(df.columns)))
check('no tick gaps', df.attrs['gaps'] == 0, str(df.attrs['gaps']))
check('time increases uniformly',
      np.allclose(np.diff(df['t']), 1e-3, atol=1e-9), str(np.unique(np.diff(df['t']))[:3]))
check('device row count agrees', abs(df.attrs['rows'] - len(df)) <= 2,
      f"device {df.attrs['rows']} vs host {len(df)}")
check('drops reported', df.attrs['drops'] == 0)
check('scaled channel is float', df['y'].dtype == float)
check('unscaled channel stays integer', np.issubdtype(df['u'].dtype, np.integer), str(df['u'].dtype))
check('units carried', df.attrs['units']['y'] == 'deg')
check('columns are native byte order',
      all(df[c].values.dtype.byteorder in '=|' for c in df.columns if c != 't'),
      str({c: df[c].values.dtype.str for c in df.columns}))
check('boolean indexing works on every column',
      all(len(df[c][df['t'] > df['t'].median()]) > 0 for c in df.columns))

# --------------------------------------------------------------------- step
dev.ref = 0
df = dev.step('ref', 2048, pre=0.10, post=0.25)
marks = df.attrs['marks']
check('step produced a mark', len(marks) == 1 and marks[0][1] == 'ref', str(marks))
check('t is exactly zero at the step', (df['t'] == 0.0).sum() == 1,
      str(df['t'].abs().min()))
check('step sample is not counted as pre-step',
      df['u'][df['t'] < 0].nunique() <= 1, str(df['u'][df['t'] < 0].unique()))
check('step has pre-trigger data', (df['t'] < 0).sum() > 50, str((df['t'] < 0).sum()))
check('step has post-trigger data', (df['t'] > 0).sum() > 100, str((df['t'] > 0).sum()))
check('ref actually stepped', df['ref'].iloc[0] == 0 and df['ref'].iloc[-1] > 100,
      f"{df['ref'].iloc[0]} -> {df['ref'].iloc[-1]}")
check('response settles toward ref',
      abs(df['y'].iloc[-1] - df['ref'].iloc[-1]) < abs(df['y'].iloc[0] - df['ref'].iloc[-1]))
check('scaling applied', abs(df['ref'].max() - 2048 * 0.0878906) < 0.01, str(df['ref'].max()))

# ---------------------------------------------------------------- decimation
dev.set('dec', 4)
dev.ref = 0
df = dev.capture(0.30)
check('decimation reported', df.attrs['dec'] == 4)
check('decimated sample spacing',
      np.allclose(np.diff(df['t']), 4e-3, atol=1e-9), str(np.unique(np.diff(df['t']))[:3]))
check('decimated run has no gaps', df.attrs['gaps'] == 0, str(df.attrs['gaps']))
dev.set('dec', 1)

# ------------------------------------------------------- 16-bit tick rollover
uno = FakeUno(start_tick=65500)
dev2 = connect(uno)
df = dev2.capture(0.20)
tick = df.attrs['tick']
check('tick wrapped during the run', tick[0] < 65536 <= tick[-1], f'{tick[0]} .. {tick[-1]}')
check('unwrapped tick is monotonic', bool(np.all(np.diff(tick) == 1)))
check('time is monotonic across the wrap', bool(np.all(np.diff(df['t']) > 0)))

# --------------------------------------- step whose mark lands across the wrap
uno = FakeUno(start_tick=65450)
dev4 = connect(uno)
dev4.ref = 0
df = dev4.step('ref', 2048, pre=0.10, post=0.20)
tick = df.attrs['tick']
check('rollover step wrapped', tick[0] < 65536 <= tick[-1], f'{tick[0]} .. {tick[-1]}')
check('rollover step zeroed at the mark',
      abs(df['t'].abs().min()) < 1.5e-3, str(df['t'].abs().min()))
check('rollover step has both sides',
      (df['t'] < 0).sum() > 50 and (df['t'] > 0).sum() > 50,
      f"{(df['t'] < 0).sum()} before, {(df['t'] > 0).sum()} after")
check('rollover step ref actually moved', df['ref'].iloc[0] == 0 and df['ref'].iloc[-1] > 100)

# ------------------------------------------------------------ malformed input
uno = FakeUno()
dev3 = connect(uno)
uno.out += b'GARBAGE\nDEADBEE\n'          # wrong-width rows before the header
df = dev3.capture(0.15)
check('short rows discarded', len(df) > 50 and df.attrs['gaps'] == 0, f'{len(df)} rows')

# ------------------------------------------------------- fixed-point params
# The device keeps these as integers; the host is the only side that ever sees
# them in real units.
dev.kq = 0.5
check('fixed-point param stored as an integer',
      dev._uno_raw('kq') == 1 << 21, str(dev._uno_raw('kq')))
check('fixed-point param reads back in real units', abs(dev.kq - 0.5) < 1e-6, str(dev.kq))

dev.kq = -0.001
check('negative fixed-point round trip', abs(dev.kq + 0.001) < 1e-6, str(dev.kq))

# Below the device's resolution: rounding to the nearest representable value is
# the right answer, not a failed write.
dev.kq = 1e-9
check('sub-resolution set does not raise', abs(dev.kq) < 1e-6, str(dev.kq))

dev.alpha = 0.1667
check('alpha quantised to Q16', dev._uno_raw('alpha') == round(0.1667 * 65536),
      str(dev._uno_raw('alpha')))
check('alpha reads back close', abs(dev.alpha - 0.1667) < 2 ** -17, str(dev.alpha))

check('dt read from the device', abs(dev.dt - 0.001) < 1e-9, str(dev.dt))

# ------------------------------------------------------------------- health
# A capture zeroes the counters the device declares, so what comes back
# describes that capture and not everything since the board booted.
uno = FakeUno()
uno.params['missed']  = ('u16', 0, 77)     # left over from some earlier run
uno.params['maxlate'] = ('u16', 0, 900)
dev4 = connect(uno)

check('health counters discovered',
      dev4._health == ('missed', 'maxlate', 'sovr', 'serr'), str(dev4._health))

df = dev4.capture(0.15)
check('stale counters zeroed before the run', df.attrs['missed'] == 0
      and df.attrs['maxlate'] == 0, str(df.attrs['maxlate']))
check('clean capture reports nothing', df.attrs['health'] == [], str(df.attrs['health']))

# Now a device that misses periods, runs late and drops rows while streaming.
uno = FakeUno(unhealthy={'missed': 12, 'maxlate': 950, 'serr': 3}, drops=4)
dev5 = connect(uno)
df = dev5.capture(0.15, warn=False)

check('missed periods reported', df.attrs['missed'] == 12, str(df.attrs['missed']))
notes = ' | '.join(df.attrs['health'])
check('missed periods explained', 'missed' in notes and '12' in notes, notes)
check('late service explained', '950 us' in notes, notes)
check('dropped rows explained', 'dropped' in notes, notes)
check('sensor errors explained', 'transfer(s) failed' in notes, notes)
check('healthy counters stay quiet', 'overrun' not in notes, notes)
check('health() reads them directly', dev5.health()['missed'] == 12, str(dev5.health()))

# ---------------------------------------------------------------- portability
# Two things that work on this machine and would not on another, so they are
# checked here rather than discovered by a student on a different one.

class _FakePort:
    def __init__(self, device, vid=None):
        self.device, self.vid = device, vid


import ctrllink as _cl
import serial.tools.list_ports as _lp

_real_comports = _lp.comports

# Ports are picked by USB vendor id, not by what they are called: COM3 on
# Windows, /dev/cu.usbmodem on macOS, /dev/ttyACM0 on Linux.
_lp.comports = lambda: [_FakePort('COM1'), _FakePort('COM3', vid=0x2341)]
check('windows COM port found', _cl.find_port() == 'COM3', _cl.find_port())

_lp.comports = lambda: [_FakePort('/dev/cu.Bluetooth-Incoming-Port'),
                        _FakePort('/dev/cu.usbmodem1101', vid=0x2341)]
check('bluetooth port ignored', _cl.find_port() == '/dev/cu.usbmodem1101',
      _cl.find_port())

# Some platforms leave vid unset; the name fallback has to cover COM as well.
_lp.comports = lambda: [_FakePort('COM3')]
check('name fallback covers COM', _cl.find_port() == 'COM3', _cl.find_port())

_lp.comports = lambda: [_FakePort('COM1', vid=1), _FakePort('COM3', vid=2)]
check('ambiguous ports rejected',
      _raises(lambda: _cl.find_port(), _cl.CtrlLinkError))
check('hint disambiguates', _cl.find_port('COM3') == 'COM3')

_lp.comports = _real_comports

# The gap between command bytes is half a millisecond. time.sleep() on Windows
# rounds up to the 15.6 ms system tick before Python 3.11, which would make
# every command thirty times slower than intended, so short waits are spun out
# instead. The bound is loose enough not to be flaky on a busy machine and
# still an order of magnitude under the failure it guards against.
_t0 = time.perf_counter()
for _ in range(200):
    _cl._pause(_cl._BYTE_GAP)
_each = (time.perf_counter() - _t0) / 200
check('short waits are actually short', _each < 2e-3, f'{_each * 1e6:.0f} us each')
check('long waits still sleep', _cl._SPIN_UNDER <= 2e-3, str(_cl._SPIN_UNDER))

# -------------------------------------------------------------------- bench
# The rig's own unit conventions, which sit on top of the protocol rather than
# in it. Checked against stub channels: what matters is the arithmetic, and the
# link underneath it is already covered above.
import bench

rig = bench.Bench.__new__(bench.Bench)
rig.channels = [_cl.Column('y_uw', 'i32', 360.0 / 4096, 'deg'),
                _cl.Column('i',    'i16', 26.4,         'mA')]

check('degrees -> counts', abs(rig.deg(45) - 45 / (360.0 / 4096)) < 1e-9, str(rig.deg(45)))
check('milliamps -> LSBs', abs(rig.ma(264) - 10.0) < 1e-9, str(rig.ma(264)))
check('counts -> degrees', abs(rig.as_deg(512) - 45.0) < 1e-9, str(rig.as_deg(512)))
check('LSBs -> milliamps', abs(rig.as_ma(10) - 264.0) < 1e-9, str(rig.as_ma(10)))
check('unknown channel raises',
      _raises(lambda: rig.channel('nope'), _cl.CtrlLinkError))

# ----------------------------------------------------------- device-side error
try:
    dev3.cmd('bogus')
    check('device error raises', False)
except Exception as exc:
    check('device error raises', 'unknown command' in str(exc), str(exc))

print()
print(f'{len(failures)} failure(s)' + (': ' + ', '.join(failures) if failures else ''))
sys.exit(1 if failures else 0)
