"""Simulación del dispositivo, fiel byte a byte a lo que Banco pone en el cable.

Vive aparte de las pruebas que la usan porque ya son dos --el enlace y la
calibración-- y un archivo de pruebas que importa a otro corre al otro entero.
"""
import sys, time, struct
sys.path.insert(0, __import__("os").path.dirname(__file__) or ".")

import numpy as np

# nombre -> (tipo de cable, bits fraccionarios, valor crudo almacenado). `kq` y
# `alpha` se guardan en punto fijo tal como un sketch guarda sus ganancias, así
# que la conversión de la computadora se ejercita en lugar de darse por buena.
PARAMS = {'dec': ('u16', 0, 1), 'kp': ('f32', 0, 0.5), 'ki': ('f32', 0, 0.0),
          'ref': ('i16', 0, 0), 'mode': ('u8', 0, 0),
          'kq': ('i32', 22, 0), 'alpha': ('i32', 16, 65536),
          # Contadores de salud, con los nombres que les pone Banco. La
          # computadora los descubre por nombre y los pone en cero antes de cada
          # captura. Los dos primeros son del lazo y los conoce ctrllink; los otros
          # dos son del sensor, asi que solo los pide quien sepa que hay un sensor.
          'loop_missed': ('u16', 0, 0), 'loop_late': ('u16', 0, 0),
          'ang_busovr': ('u16', 0, 0), 'ang_buserr': ('u16', 0, 0),
          # Estado, no cuentas: la computadora los lee despues de una captura
          # pero no los pone en cero antes.
          'ang_present': ('u8', 0, 1), 'ang_status': ('u8', 0, 0x20)}
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
            self.println('# id CtrlLink 1 Banco chans=4 row=21 dt_us=1000')
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


def connect(uno, diagnostico=None):
    """Un CtrlLink enganchado a un dispositivo de mentira.

    `diagnostico` es el colaborador que sepa qué equipo hay del otro lado. Sin él
    el enlace informa lo del lazo y nada más, que es justamente lo que hay que
    poder verificar: que ctrllink no sabe nada de ningún sensor.
    """
    import ctrllink
    dev = ctrllink.CtrlLink.__new__(ctrllink.CtrlLink)
    dev._diag = diagnostico
    dev.ser = FakeSerial(uno)
    dev.info = dev.sync()
    dev._params = dev._read_params()
    dev.channels = dev._read_channels()
    # Permite que una verificacion mire lo que el dispositivo realmente guardo, en
    # lugar de lo que informa la computadora despues de reescalarlo.
    dev._uno_raw = lambda name: uno.params[name][2]
    return dev

