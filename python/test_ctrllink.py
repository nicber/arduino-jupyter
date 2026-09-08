"""Ejercita el módulo del lado computadora contra una simulación del sketch fiel byte a byte.

El simulacro reproduce lo que CtrlLink.cpp realmente pone en el cable —CRLF en las
líneas de println() y LF pelado en las filas de telemetría incluidos—, así que el
decodificador se prueba contra el encuadre real y no contra una versión idealizada.
"""
import sys, time, struct
sys.path.insert(0, __import__("os").path.dirname(__file__) or ".")

import numpy as np

# nombre -> (tipo de cable, bits fraccionarios, valor crudo almacenado). `kq` y
# `alpha` se guardan en punto fijo tal como ControlDemo guarda sus ganancias, así
# que la conversión de la computadora se ejercita en lugar de darse por buena.
PARAMS = {'dec': ('u16', 0, 1), 'kp': ('f32', 0, 0.5), 'ki': ('f32', 0, 0.0),
          'ref': ('i16', 0, 0), 'mode': ('u8', 0, 0),
          'kq': ('i32', 22, 0), 'alpha': ('i32', 16, 65536),
          # Contadores de salud, tal como los declara ControlDemo. La computadora
          # los descubre por nombre y los pone en cero antes de cada captura.
          'missed': ('u16', 0, 0), 'maxlate': ('u16', 0, 0),
          'sovr': ('u16', 0, 0), 'serr': ('u16', 0, 0)}
CHANS = [('ref', 'i16', 0.0878906, 'deg'), ('y', 'i16', 0.0878906, 'deg'),
         ('e', 'i16', 0.0878906, 'deg'), ('u', 'i16', 1.0, 'pwm')]
WIDTH = {'i16': 4, 'u16': 4, 'u8': 2, 'i32': 8, 'f32': 8}


class FakeUno:
    """Máquina de estados del lado dispositivo; `out` es lo que leería la computadora."""

    def __init__(self, rate_hz=1000, start_tick=0, unhealthy=None, drops=0):
        self.out = bytearray()
        self.line = bytearray()
        self.params = dict(PARAMS)
        self.streaming = False
        self.tick = start_tick
        self.rows = 0          # filas realmente escritas, como las cuenta CtrlLink
        self.produced = 0      # periodos de control transcurridos, diezmados o no
        self.rate = rate_hz
        self.t0 = None
        self.y = 0
        # Valores de contador que el dispositivo "descubre" una vez que la corrida
        # esta en marcha, de modo que una captura que primero los pone en cero
        # igual los encuentre distintos de cero al final.
        self.unhealthy = unhealthy or {}
        self.drops = drops

    def println(self, s=''):
        self.out += (s + '\r\n').encode()   # println de Arduino agrega CRLF

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
                self._pump()   # la marca cae en el tick alcanzado hasta ahora
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
            self.println('# err comando desconocido')

    def _show(self, name):
        type_, _frac, value = self.params[name]
        shown = f'{value:.6f}' if type_ == 'f32' else str(value)
        self.println(f'# v {name} {shown}')

    def _pump(self):
        """Emite las filas que a esta altura tendrian que haberse producido."""
        if not self.streaming:
            return
        due = int((time.monotonic() - self.t0) * self.rate)
        dec = self.params['dec'][2]
        while self.produced < due:
            ref = self.params['ref'][2]
            self.y += (ref - self.y) // 8          # visiblemente de primer orden
            err = ref - self.y
            u = max(-255, min(255, err // 4))
            if self.tick % dec == 0:
                row = ''.join(f'{v & 0xFFFF:04X}'
                              for v in (self.tick, ref, self.y, err, u))
                self.out += (row + '\n').encode()  # las filas usan LF pelado
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
    # Permite que una verificacion mire lo que el dispositivo realmente guardo, en
    # lugar de lo que informa la computadora despues de reescalarlo.
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
    print(f'{"PASA " if condition else "FALLA"}  {label}' + (f'  -- {detail}' if detail and not condition else ''))
    if not condition:
        failures.append(label)


# ------------------------------------------------------------ descubrimiento
dev = connect(FakeUno())
check('se interpreta id', dev.info.startswith('CtrlLink 1 ControlDemo'), dev.info)
check('se descubren los parametros', set(dev._params) == set(PARAMS), str(dev._params))
check('se descubren los canales', [c.name for c in dev.channels] == ['ref', 'y', 'e', 'u'])
check('parametro float tipado', dev._params['kp'].type == 'f32')
check('se descubre el formato del parametro', dev._params['kq'].frac == 22
      and dev._params['kq'].scale == 2.0 ** -22, str(dev._params['kq']))

# ------------------------------------------------------- acceso a parametros
dev.kp = 2.5
check('ida y vuelta de un float', abs(dev.kp - 2.5) < 1e-6, str(dev.kp))
dev.ref = 1024
check('ida y vuelta de un entero', dev.ref == 1024, str(dev.ref))
check('el entero sigue siendo entero', isinstance(dev.ref, int))
check('instantanea de parametros', dev.params['kp'] == 2.5, str(dev.params))
try:
    dev.nonexistent
    check('un atributo desconocido levanta excepcion', False)
except AttributeError:
    check('un atributo desconocido levanta excepcion', True)

# ------------------------------------------------------------------ captura
dev.ref = 0
df = dev.capture(0.30)
check('la captura devolvio filas', len(df) > 100, f'{len(df)} filas')
check('columnas como se declararon', list(df.columns) == ['t', 'ref', 'y', 'e', 'u'], str(list(df.columns)))
check('sin huecos de tick', df.attrs['gaps'] == 0, str(df.attrs['gaps']))
check('el tiempo crece uniformemente',
      np.allclose(np.diff(df['t']), 1e-3, atol=1e-9), str(np.unique(np.diff(df['t']))[:3]))
check('coincide el conteo de filas del dispositivo', abs(df.attrs['rows'] - len(df)) <= 2,
      f"dispositivo {df.attrs['rows']} contra computadora {len(df)}")
check('se informan los descartes', df.attrs['drops'] == 0)
check('un canal escalado es float', df['y'].dtype == float)
check('un canal sin escalar sigue entero', np.issubdtype(df['u'].dtype, np.integer), str(df['u'].dtype))
check('se transportan las unidades', df.attrs['units']['y'] == 'deg')
check('las columnas estan en el orden de bytes nativo',
      all(df[c].values.dtype.byteorder in '=|' for c in df.columns if c != 't'),
      str({c: df[c].values.dtype.str for c in df.columns}))
check('el indexado booleano funciona en todas las columnas',
      all(len(df[c][df['t'] > df['t'].median()]) > 0 for c in df.columns))

# ------------------------------------------------------------------- escalon
dev.ref = 0
df = dev.step('ref', 2048, pre=0.10, post=0.25)
marks = df.attrs['marks']
check('el escalon produjo una marca', len(marks) == 1 and marks[0][1] == 'ref', str(marks))
check('t es exactamente cero en el escalon', (df['t'] == 0.0).sum() == 1,
      str(df['t'].abs().min()))
check('la muestra del escalon no cuenta como previa',
      df['u'][df['t'] < 0].nunique() <= 1, str(df['u'][df['t'] < 0].unique()))
check('el escalon tiene datos previos al disparo', (df['t'] < 0).sum() > 50, str((df['t'] < 0).sum()))
check('el escalon tiene datos posteriores al disparo', (df['t'] > 0).sum() > 100, str((df['t'] > 0).sum()))
check('ref efectivamente dio el escalon', df['ref'].iloc[0] == 0 and df['ref'].iloc[-1] > 100,
      f"{df['ref'].iloc[0]} -> {df['ref'].iloc[-1]}")
check('la respuesta se establece hacia ref',
      abs(df['y'].iloc[-1] - df['ref'].iloc[-1]) < abs(df['y'].iloc[0] - df['ref'].iloc[-1]))
check('se aplico el escalado', abs(df['ref'].max() - 2048 * 0.0878906) < 0.01, str(df['ref'].max()))

# ---------------------------------------------------------------- diezmacion
dev.set('dec', 4)
dev.ref = 0
df = dev.capture(0.30)
check('se informa la diezmacion', df.attrs['dec'] == 4)
check('separacion de muestras diezmadas',
      np.allclose(np.diff(df['t']), 4e-3, atol=1e-9), str(np.unique(np.diff(df['t']))[:3]))
check('la corrida diezmada no tiene huecos', df.attrs['gaps'] == 0, str(df.attrs['gaps']))
dev.set('dec', 1)

# ------------------------------------------- vuelta al cero del tick de 16 bits
uno = FakeUno(start_tick=65500)
dev2 = connect(uno)
df = dev2.capture(0.20)
tick = df.attrs['tick']
check('el tick dio la vuelta durante la corrida', tick[0] < 65536 <= tick[-1], f'{tick[0]} .. {tick[-1]}')
check('el tick desenrollado es monotono', bool(np.all(np.diff(tick) == 1)))
check('el tiempo es monotono a traves de la vuelta', bool(np.all(np.diff(df['t']) > 0)))

# ------------------------- escalon cuya marca cae del otro lado de la vuelta
uno = FakeUno(start_tick=65450)
dev4 = connect(uno)
dev4.ref = 0
df = dev4.step('ref', 2048, pre=0.10, post=0.20)
tick = df.attrs['tick']
check('el escalon con vuelta dio la vuelta', tick[0] < 65536 <= tick[-1], f'{tick[0]} .. {tick[-1]}')
check('el escalon con vuelta quedo en cero en la marca',
      abs(df['t'].abs().min()) < 1.5e-3, str(df['t'].abs().min()))
check('el escalon con vuelta tiene los dos lados',
      (df['t'] < 0).sum() > 50 and (df['t'] > 0).sum() > 50,
      f"{(df['t'] < 0).sum()} antes, {(df['t'] > 0).sum()} despues")
check('en el escalon con vuelta ref efectivamente se movio',
      df['ref'].iloc[0] == 0 and df['ref'].iloc[-1] > 100)

# --------------------------------------------------------- entrada mal formada
uno = FakeUno()
dev3 = connect(uno)
uno.out += b'GARBAGE\nDEADBEE\n'          # filas de ancho equivocado antes del encabezado
df = dev3.capture(0.15)
check('se descartan las filas cortas', len(df) > 50 and df.attrs['gaps'] == 0, f'{len(df)} filas')

# ------------------------------------------------- parametros en punto fijo
# El dispositivo los guarda como enteros; la computadora es el unico lado que los
# ve alguna vez en unidades reales.
dev.kq = 0.5
check('un parametro en punto fijo se guarda como entero',
      dev._uno_raw('kq') == 1 << 21, str(dev._uno_raw('kq')))
check('un parametro en punto fijo se relee en unidades reales', abs(dev.kq - 0.5) < 1e-6, str(dev.kq))

dev.kq = -0.001
check('ida y vuelta de un punto fijo negativo', abs(dev.kq + 0.001) < 1e-6, str(dev.kq))

# Por debajo de la resolucion del dispositivo: redondear al valor representable
# mas cercano es la respuesta correcta, no una escritura fallida.
dev.kq = 1e-9
check('fijar por debajo de la resolucion no levanta excepcion', abs(dev.kq) < 1e-6, str(dev.kq))

dev.alpha = 0.1667
check('alpha cuantizado a Q16', dev._uno_raw('alpha') == round(0.1667 * 65536),
      str(dev._uno_raw('alpha')))
check('alpha se relee cerca', abs(dev.alpha - 0.1667) < 2 ** -17, str(dev.alpha))

check('dt leido del dispositivo', abs(dev.dt - 0.001) < 1e-9, str(dev.dt))

# --------------------------------------------------------------------- salud
# Una captura pone en cero los contadores que el dispositivo declara, asi que lo
# que vuelve describe esa captura y no todo lo ocurrido desde que arranco la placa.
uno = FakeUno()
uno.params['missed']  = ('u16', 0, 77)     # resabio de alguna corrida anterior
uno.params['maxlate'] = ('u16', 0, 900)
dev4 = connect(uno)

check('se descubren los contadores de salud',
      dev4._health == ('missed', 'maxlate', 'sovr', 'serr'), str(dev4._health))

df = dev4.capture(0.15)
check('los contadores viejos se ponen en cero antes de la corrida', df.attrs['missed'] == 0
      and df.attrs['maxlate'] == 0, str(df.attrs['maxlate']))
check('una captura limpia no informa nada', df.attrs['health'] == [], str(df.attrs['health']))

# Ahora un dispositivo que pierde periodos, llega tarde y descarta filas mientras
# emite.
uno = FakeUno(unhealthy={'missed': 12, 'maxlate': 950, 'serr': 3}, drops=4)
dev5 = connect(uno)
df = dev5.capture(0.15, warn=False)

check('se informan los periodos perdidos', df.attrs['missed'] == 12, str(df.attrs['missed']))
notes = ' | '.join(df.attrs['health'])
check('se explican los periodos perdidos', 'perdieron' in notes and '12' in notes, notes)
check('se explica la atencion tardia', '950 us' in notes, notes)
check('se explican las filas descartadas', 'descartaron' in notes, notes)
check('se explican los errores del sensor', 'transferencia(s) del sensor' in notes, notes)
check('los contadores sanos se quedan callados', 'desborde' not in notes, notes)
check('health() los lee directamente', dev5.health()['missed'] == 12, str(dev5.health()))

# ------------------------------------------------------------- portabilidad
# Dos cosas que funcionan en esta maquina y no funcionarian en otra, asi que se
# verifican aca en lugar de que las descubra un alumno en una distinta.

class _FakePort:
    def __init__(self, device, vid=None):
        self.device, self.vid = device, vid


import ctrllink as _cl
import serial.tools.list_ports as _lp

_real_comports = _lp.comports

# Los puertos se eligen por identificador de fabricante USB, no por como se
# llaman: COM3 en Windows, /dev/cu.usbmodem en macOS, /dev/ttyACM0 en Linux.
_lp.comports = lambda: [_FakePort('COM1'), _FakePort('COM3', vid=0x2341)]
check('se encuentra el puerto COM de Windows', _cl.find_port() == 'COM3', _cl.find_port())

_lp.comports = lambda: [_FakePort('/dev/cu.Bluetooth-Incoming-Port'),
                        _FakePort('/dev/cu.usbmodem1101', vid=0x2341)]
check('se ignora el puerto Bluetooth', _cl.find_port() == '/dev/cu.usbmodem1101',
      _cl.find_port())

# Algunas plataformas dejan vid sin cargar; el respaldo por nombre tiene que
# cubrir COM tambien.
_lp.comports = lambda: [_FakePort('COM3')]
check('el respaldo por nombre cubre COM', _cl.find_port() == 'COM3', _cl.find_port())

_lp.comports = lambda: [_FakePort('COM1', vid=1), _FakePort('COM3', vid=2)]
check('se rechazan los puertos ambiguos',
      _raises(lambda: _cl.find_port(), _cl.CtrlLinkError))
check('el hint desambigua', _cl.find_port('COM3') == 'COM3')

_lp.comports = _real_comports

# La separacion entre bytes de comando es medio milisegundo. Antes de Python 3.11,
# time.sleep() en Windows redondea hacia arriba hasta el tic de 15,6 ms del
# sistema, lo que haria cada comando treinta veces mas lento de lo previsto, asi
# que las esperas cortas se hacen en vacio. La cota es lo bastante holgada como
# para no ser inestable en una maquina ocupada y sigue estando un orden de
# magnitud por debajo de la falla que previene.
_t0 = time.perf_counter()
for _ in range(200):
    _cl._pause(_cl._BYTE_GAP)
_each = (time.perf_counter() - _t0) / 200
check('las esperas cortas son realmente cortas', _each < 2e-3, f'{_each * 1e6:.0f} us cada una')
check('las esperas largas siguen durmiendo', _cl._SPIN_UNDER <= 2e-3, str(_cl._SPIN_UNDER))

# --------------------------------------------------------------------- banco
# Las convenciones de unidades del equipo, que se apoyan sobre el protocolo en
# lugar de estar dentro de el. Se verifican contra canales de prueba: lo que
# importa es la aritmetica, y el enlace de abajo ya quedo cubierto mas arriba.
import bench

rig = bench.Bench.__new__(bench.Bench)
rig.channels = [_cl.Column('y_uw', 'i32', 360.0 / 4096, 'deg'),
                _cl.Column('i',    'i16', 26.4,         'mA')]

check('grados -> cuentas', abs(rig.deg(45) - 45 / (360.0 / 4096)) < 1e-9, str(rig.deg(45)))
check('miliamperes -> LSBs', abs(rig.ma(264) - 10.0) < 1e-9, str(rig.ma(264)))
check('cuentas -> grados', abs(rig.as_deg(512) - 45.0) < 1e-9, str(rig.as_deg(512)))
check('LSBs -> miliamperes', abs(rig.as_ma(10) - 264.0) < 1e-9, str(rig.as_ma(10)))
check('un canal desconocido levanta excepcion',
      _raises(lambda: rig.channel('nope'), _cl.CtrlLinkError))

# ------------------------------------------------- error del lado dispositivo
try:
    dev3.cmd('bogus')
    check('un error del dispositivo levanta excepcion', False)
except Exception as exc:
    check('un error del dispositivo levanta excepcion',
          'comando desconocido' in str(exc), str(exc))

print()
print(f'{len(failures)} falla(s)' + (': ' + ', '.join(failures) if failures else ''))
sys.exit(1 if failures else 0)
