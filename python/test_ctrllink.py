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
          'sovr': ('u16', 0, 0), 'serr': ('u16', 0, 0),
          # Estado, no cuentas: la computadora los lee despues de una captura
          # pero no los pone en cero antes.
          'spres': ('u8', 0, 1), 'mstat': ('u8', 0, 0x20)}
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
        if head == '':
            return                 # linea vacia: un empujon para resincronizar
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
            if not name or not value:
                self.println('# err set necesita un nombre y un valor')
                return
            if name not in self.params:
                self.println('# err no existe ese parametro')
                return
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


class OldFakeUno(FakeUno):
    """Un dispositivo con el sketch anterior a que los parametros cruzaran el
    cable en punto fijo: las lineas `# p` no traen la columna de bits
    fraccionarios."""

    def command(self, cmd):
        head, _, _rest = cmd.partition(' ')
        if head != 'params':
            return super().command(cmd)
        for name, (type_, _frac, value) in self.params.items():
            shown = f'{value:.6f}' if type_ == 'f32' else str(value)
            self.println(f'# p {name} {type_} {shown}')
        self.println('# ok')


class FakeSerial:
    def __init__(self, uno):
        self.uno = uno
        self.is_open = True
        self.pos = 0
        # Cuantas lecturas (o escrituras de un byte) faltan para que el puerto
        # simule que la celda se corto por el medio. Es la unica forma fiel de
        # probar la recuperacion: una interrupcion cae dentro de una llamada al
        # puerto, no entre dos operaciones prolijas.
        self.fail_read_after = None
        self.fail_write_after = None
        self.fail_with = KeyboardInterrupt
        # Bytes salientes que el enlace se traga antes de empezar a entregar. El
        # camino de ida pierde bytes de verdad; ver _BYTE_GAP. `deaf_after_fail`
        # los arma recien al cortarse la operacion, que es donde interesan: el
        # comando que se pierde es el que manda la limpieza.
        self.deaf_writes = 0
        self.deaf_after_fail = 0

    def _maybe_fail(self, which):
        left = getattr(self, which)
        if left is None:
            return
        if left > 0:
            setattr(self, which, left - 1)
            return
        setattr(self, which, None)   # una sola vez: la limpieza tiene que poder correr
        self.deaf_writes += self.deaf_after_fail
        raise self.fail_with

    @property
    def in_waiting(self):
        self.uno._pump()
        return len(self.uno.out) - self.pos

    def write(self, data):
        self._maybe_fail('fail_write_after')
        self.uno._pump()
        if self.deaf_writes > 0:
            self.deaf_writes -= len(data)
            return len(data)
        self.uno.feed(data)
        return len(data)

    def read(self, n=1):
        self._maybe_fail('fail_read_after')
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

import ctrllink as _cl

# ------------------------------------------------- sensor ausente en el bus
# `spres` es estado, no una cuenta: dice si el AS5600 contesta *ahora*. Es lo
# que separa un iman mal montado -- el sensor contesta y se queja del iman -- de
# un sensor que no esta en el bus.
check('se descubren los parametros de estado', dev._state == ('spres', 'mstat'),
      str(dev._state))

uno = FakeUno(unhealthy={'spres': 0, 'serr': 2})
dev6 = connect(uno)
df = dev6.capture(0.15, warn=False)
notes = ' | '.join(df.attrs['health'])
check('el sensor ausente se informa', df.attrs['spres'] == 0, str(df.attrs['spres']))
check('el sensor ausente se explica primero',
      df.attrs['health'] and 'no contesta' in df.attrs['health'][0], notes)
check('el sondeo al sensor ausente no se cuenta como intermitencia',
      'transferencia(s) del sensor' not in notes, notes)

# Con el sensor presente, en cambio, las fallas sueltas si son intermitencias.
uno = FakeUno(unhealthy={'serr': 2})
dev7 = connect(uno)
df = dev7.capture(0.15, warn=False)
notes = ' | '.join(df.attrs['health'])
check('con el sensor presente las fallas sueltas se informan',
      'transferencia(s) del sensor' in notes, notes)

# El estado no se pone en cero antes de una captura: hacerlo seria inventar una
# lectura, y ademas dejaria `spres` diciendo "ausente" en cada corrida.
check('el estado no se pone en cero antes de la corrida',
      dev7._uno_raw('spres') == 1, str(dev7._uno_raw('spres')))

# ---------------------------------------------- fin de captura sin carrera
# El dispositivo contesta "# end ..." y despues "# ok". Darse por satisfecho con
# el "# end" deja el "# ok" en el puerto, y el comando siguiente lo lee como su
# propio terminador y vuelve vacio.
check('_ended espera la linea que cierra',
      not _cl._ended(b'# end rows=149 drops=0\r\n'))
check('_ended con la respuesta completa',
      _cl._ended(b'# end rows=149 drops=0\r\n# ok\r\n'))
check('_ended con una fila delante',
      _cl._ended(b'0412CDB9\n# end rows=1 drops=0\r\n# ok\r\n'))

# ----------------------------------------------- duracion real de la ventana
# La unica referencia de tiempo independiente que hay: los ticks los cuenta el
# dispositivo y avanzan una vez por periodo *atendido*, asi que filas sobre
# ticks da el periodo nominal pase lo que pase.
df = dev7.capture(0.20, warn=False)
check('la captura informa su duracion real',
      0.20 <= df.attrs['wall'] < 0.20 * 4, str(df.attrs.get('wall')))

# ------------------------------------------------------ firmware desactualizado
uno_old = OldFakeUno()
dev8 = _cl.CtrlLink.__new__(_cl.CtrlLink)
dev8.ser = FakeSerial(uno_old)
dev8.info = dev8.sync()
try:
    dev8._read_params()
    check('el firmware viejo se explica', False)
except Exception as exc:
    check('el firmware viejo se explica',
          isinstance(exc, _cl.CtrlLinkError) and 'sketch viejo' in str(exc), str(exc))

# ------------------------------------------------------------- portabilidad
# Dos cosas que funcionan en esta maquina y no funcionarian en otra, asi que se
# verifican aca en lugar de que las descubra un alumno en una distinta.

class _FakePort:
    def __init__(self, device, vid=None):
        self.device, self.vid = device, vid


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

# --------------------------------------------- celda cortada por el medio
# Lo que de verdad pasa en un notebook: el boton de parar en mitad de una
# captura, un traceback a mitad de un `set`. El dispositivo queda emitiendo o el
# comando queda a medio escribir, y la celda siguiente heredaria el desastre. La
# operacion siguiente tiene que encontrar el enlace limpio sin reiniciar nada.

uno = FakeUno()
dev9 = connect(uno)
dev9.ser.fail_read_after = 3          # se corta en plena captura
check('la captura interrumpida propaga la interrupcion',
      _raises(lambda: dev9.capture(0.40), KeyboardInterrupt))
check('la captura interrumpida callo al dispositivo', not uno.streaming)
check('la captura interrumpida dejo el enlace limpio', dev9._broken is False)

dev9.ref = 512
check('despues de la interrupcion se puede fijar un parametro', dev9.ref == 512, str(dev9.ref))
df = dev9.capture(0.20)
check('despues de la interrupcion se puede volver a capturar',
      len(df) > 50 and df.attrs['gaps'] == 0, f"{len(df)} filas, {df.attrs['gaps']} huecos")

# Una excepcion cualquiera, no una interrupcion: el enlace no distingue, porque
# lo que lo ensucia es haberse cortado y no el motivo.
uno = FakeUno()
dev10 = connect(uno)
dev10.ser.fail_with = ValueError
dev10.ser.fail_read_after = 3
check('una excepcion en plena captura sale a la celda',
      _raises(lambda: dev10.capture(0.40), ValueError))
check('una excepcion en plena captura tambien calla al dispositivo', not uno.streaming)
check('el enlace sobrevive a una excepcion cualquiera', dev10.get('kp') == 0.5, str(dev10.get('kp')))

# Cortado a mitad de una linea de comando: los bytes que llegaron estan en el
# buffer del dispositivo y se pegarian adelante del comando siguiente.
uno = FakeUno()
dev11 = connect(uno)
dev11.ser.fail_write_after = 3        # "set" enviado, el resto no
check('el comando interrumpido propaga la interrupcion',
      _raises(lambda: dev11.set('ref', 1024), KeyboardInterrupt))
check('la media linea no envenena el comando siguiente', dev11.kp == 0.5, str(dev11.kp))
check('el comando interrumpido no dejo la referencia a medias', dev11.ref == 0, str(dev11.ref))

# Un escalon interrumpido no deja la referencia en pie: `back` es donde el
# usuario dijo que queria terminar, y del otro lado del cable puede haber un
# motor empujando contra un tope.
uno = FakeUno()
dev12 = connect(uno)
dev12.ref = 0
dev12.ser.fail_read_after = 3
check('el escalon interrumpido propaga la interrupcion',
      _raises(lambda: dev12.step('ref', 2048, pre=0.05, post=0.35, back=0),
              KeyboardInterrupt))
check('el escalon interrumpido restituye la referencia', dev12.ref == 0, str(dev12.ref))

# La limpieza se puede pedir a mano, para lo que este modulo no vio pasar.
uno = FakeUno()
dev13 = connect(uno)
dev13.cmd('start')
check('el dispositivo quedo emitiendo', uno.streaming)
dev13.resync()
check('resync callo al dispositivo', not uno.streaming)
check('resync deja el enlace usable', dev13.get('kp') == 0.5, str(dev13.get('kp')))

# Un enlace marcado como sucio se limpia solo antes de mandar nada, aunque nadie
# haya llegado a limpiarlo en su momento (una segunda interrupcion encima de la
# primera).
uno = FakeUno()
dev14 = connect(uno)
dev14.cmd('start')
dev14._broken = True
check('un enlace sucio se limpia antes del comando siguiente',
      dev14.get('kp') == 0.5 and not uno.streaming, str(uno.streaming))

# Un `stop` se puede perder en el camino de ida, y entonces el puerto callado no
# prueba nada: con `dec` alto una fila tarda mas que cualquier ventana de
# silencio razonable. La limpieza espera la confirmacion del `stop`, no el
# silencio, y lo reintenta hasta tenerla.
uno = FakeUno()
dev15 = connect(uno)
dev15.set('dec', 400)                 # una fila cada 400 ms
dev15.ser.deaf_after_fail = 6         # el primer "stop" de la limpieza se pierde
dev15.ser.fail_read_after = 2
check('la captura lenta interrumpida propaga la interrupcion',
      _raises(lambda: dev15.capture(0.40), KeyboardInterrupt))
check('un stop perdido se reintenta hasta que el dispositivo confirma',
      not uno.streaming)
check('el enlace queda usable despues del stop perdido',
      dev15.get('kp') == 0.5, str(dev15.get('kp')))
dev15.set('dec', 1)

# ---------------------------------------- el puerto no queda tomado si falla
# Un puerto serie es exclusivo. Si el descubrimiento falla despues de abrirlo, el
# traceback de la celda sobrevive en sys.last_traceback y se queda con el puerto:
# el intento siguiente falla con "no se pudo abrir el puerto" y parece otra cosa.

class _MutePort:
    """Un puerto que abre pero del que no contesta nadie."""
    def __init__(self):
        self.is_open = True
        self.edges = []          # transiciones de DTR, que es lo que resetea la placa
        self._dtr = True
    @property
    def dtr(self):
        return self._dtr
    @dtr.setter
    def dtr(self, value):
        self._dtr = value
        self.edges.append(value)
    def readline(self):
        return b''
    def read(self, n=1):
        return b''
    @property
    def in_waiting(self):
        return 0
    def write(self, data):
        return len(data)
    def flush(self):
        pass
    def reset_input_buffer(self):
        pass
    def close(self):
        self.is_open = False


_opened = []
_real_serial, _real_sync = _cl.serial.Serial, _cl.CtrlLink.sync


def _mute_serial(*args, **kwargs):
    _opened.append(_MutePort())
    return _opened[-1]


def _mute_sync(self, timeout=4.0):
    raise _cl.CtrlLinkError('no hubo respuesta a "id"')


_cl.serial.Serial, _cl.CtrlLink.sync = _mute_serial, _mute_sync
try:
    check('un descubrimiento fallido se explica',
          _raises(lambda: _cl.CtrlLink('COM9', reset_wait=0.02), _cl.CtrlLinkError))
    _raises(lambda: _cl.CtrlLink('COM9', reset_wait=0), _cl.CtrlLinkError)
finally:
    _cl.serial.Serial, _cl.CtrlLink.sync = _real_serial, _real_sync

check('un descubrimiento fallido no se queda con el puerto',
      bool(_opened) and not _opened[0].is_open)

# Lo que resetea un UNO es el flanco de bajada de DTR, no que DTR quede activado.
# Abrir el puerto no alcanza: la linea puede venir activada de la conexion
# anterior, y entonces no hay flanco y la placa sigue corriendo con el estado que
# le dejo la corrida pasada.
check('la placa se resetea con un flanco de DTR y no con abrir el puerto',
      _opened[0].edges == [False, True], str(_opened[0].edges))
check('reset_wait=0 se engancha a un sketch que ya corre, sin resetear',
      _opened[1].edges == [], str(_opened[1].edges))

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
