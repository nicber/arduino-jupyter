"""Lado computadora del protocolo CtrlLink.

Se comunica por un puerto serie con un Arduino que corre la biblioteca CtrlLink.
El dispositivo describe por sí mismo sus parámetros ajustables y sus canales de
telemetría, así que este módulo no sabe nada de ningún sketch en particular:
agregar una ganancia al firmware la hace aparecer acá sin tocar una línea de este
lado.

    from ctrllink import CtrlLink

    dev = CtrlLink()                    # o CtrlLink('COM3'), o '/dev/ttyACM0'
    dev.kp, dev.ki = 2.5, 0.1
    df = dev.step('ref', 1024, pre=0.1, post=0.9)
    df.plot(x='t', y=['ref', 'y'])

Las filas de telemetría son hexadecimal de ancho fijo, así que una captura se
decodifica en una sola llamada a numpy en lugar de interpretarse línea por línea;
recibir un flujo de 1 kHz no cuesta casi nada.

Acá los parámetros están siempre en unidades reales. El dispositivo guarda cada
uno en la forma de punto fijo que su aritmética prefiera y dice cuántos bits
fraccionarios son; este lado multiplica a la ida y divide a la vuelta, así que el
lazo que corre en un microcontrolador de 8 bits nunca ejecuta una instrucción de
punto flotante y quien lo maneja nunca ve una cuenta cruda.

El enlace se limpia solo. Si una celda se corta por el medio —el botón de parar
en mitad de una captura, una excepción a mitad de un `set`—, la operación
siguiente encuentra el dispositivo callado y el puerto vacío en lugar de heredar
un flujo a medio terminar, así que nunca hace falta reiniciar el kernel para
recuperar el control de la placa.
"""

from __future__ import annotations

import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass

import numpy as np
import serial

# Tipo del cable -> dtype big-endian de numpy. El ancho hexadecimal de un campo es
# el doble de su itemsize, y todos los anchos son pares, así que una tanda de
# filas se convierte de hexadecimal en un solo bloque.
_TYPES = {
    'i8': np.dtype('>i1'),
    'u8': np.dtype('>u1'),
    'i16': np.dtype('>i2'),
    'u16': np.dtype('>u2'),
    'i32': np.dtype('>i4'),
    'u32': np.dtype('>u4'),
    'f32': np.dtype('>f4'),
}

# Respuestas que dan por terminada la contestación a un comando.
_TERMINATORS = ('# ok', '# err', '# data')

# Errores que significan que el dispositivo no recibió lo que se le mandó, y no
# que lo recibió y objetó. Sólo vale la pena reintentar éstos.
_GARBLED = ('comando desconocido', 'comando demasiado largo', 'necesita')

# Parámetros de salud, si el sketch los declara. Ninguno es parte del protocolo:
# los nombres son una convención, y un dispositivo que no expone ninguno
# simplemente no informa nada. Son totales acumulados y no lecturas instantáneas,
# así que una captura los pone en cero primero y lo que vuelve describe esa
# captura y nada más.
#
#   missed   períodos de control que el lazo nunca atendió
#   maxlate  peor retardo entre el disparo de un tick y el momento en que el lazo
#            lo levanta, en us
#   sovr     muestras del sensor que el bus no llegó a seguir
#   serr     transferencias del sensor que fallaron
_HEALTH = ('missed', 'maxlate', 'sovr', 'serr')

# Parámetros de estado. Se leen después de una captura como los de salud, pero
# NO se ponen en cero antes: no son cuentas acumuladas sino el estado del equipo
# en este momento, y ponerlos en cero sería inventar una lectura.
#
#   spres    el sensor contesta en el bus (0 = no está)
#   mstat    registro STATUS del AS5600: imán detectado, muy débil, muy fuerte
_STATUS = ('spres', 'mstat')

# Fracción del período de control a partir de la cual vale la pena mencionar un
# retardo de atención, aunque todavía no se haya perdido nada.
_LATE_WARN = 0.5

# Segundos entre los bytes de un comando saliente.
#
# La telemetría corre a 1 Mbaud sin problemas, pero el camino de vuelta es frágil:
# llega un byte cada 10 us, el USART del AVR guarda dos, y entre el muestreador de
# 5 kHz y la interrupción de TWI de nI2C mantienen las interrupciones
# deshabilitadas durante más que eso. Enviados de corrido, unos pocos por ciento
# de los bytes de comando se pierden sin más. Espaciarlos lo soluciona por
# completo, y los comandos son demasiado raros y cortos como para que el retardo
# importe: un `set` tarda unos 6 ms en enviarse.
_BYTE_GAP = 0.0005

# Los retardos más cortos que esto se esperan en vacío en lugar de dormirse. Ver
# _pause().
_SPIN_UNDER = 0.002


def _pause(seconds):
    """Un retardo corto que de verdad es corto.

    Antes de Python 3.11, time.sleep() en Windows redondea hacia arriba hasta el
    tic del temporizador del sistema —15,6 ms por omisión, treinta veces la
    separación entre bytes de comando—. Dormir _BYTE_GAP ahí convertiría un `set`
    de seis milisegundos en uno de seiscientos, y a capture(), que envía ocho
    comandos alrededor de cada corrida, en algo que parecería roto.

    Así que cualquier cosa por debajo de un par de milisegundos se espera en vacío
    sobre perf_counter(), que tiene alta resolución en todas partes. Cuesta tener
    la CPU ocupada durante los veinte milisegundos que tarda en enviarse un
    comando, que es un precio justo por comportarse igual en cualquier máquina.
    """
    if seconds >= _SPIN_UNDER:
        time.sleep(seconds)
        return

    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        pass


class CtrlLinkError(RuntimeError):
    pass


def find_port(hint=None):
    """Adivina en qué puerto serie está la placa.

    Los adaptadores Bluetooth y las consolas de depuración también se presentan
    como puertos serie, así que la búsqueda se limita a los de USB, y por
    identificador de fabricante USB, que todas las plataformas informan y que no
    dice nada sobre cómo se *llama* el puerto. Windows los llama COM3, macOS
    /dev/cu.usbmodem1101 y Linux /dev/ttyACM0, y buscar por esos nombres encuentra
    una placa en una máquina y nada en la siguiente.

    `hint` acota todavía más por subcadena, que es lo que hay que usar cuando hay
    más de una placa enchufada.
    """
    from serial.tools import list_ports

    found = list(list_ports.comports())
    usb = [p for p in found if p.vid is not None]

    if not usb:
        # Algunas plataformas y algunas versiones viejas de pyserial dejan vid sin
        # cargar. Se recurre entonces a los nombres que suele tener un puerto serie
        # USB, puertos COM incluidos.
        usb = [p for p in found
               if any(tag in p.device for tag in
                      ('usbserial', 'usbmodem', 'ttyUSB', 'ttyACM',
                       'wchusbserial', 'COM'))]

    ports = [p.device for p in usb]

    if hint:
        ports = [p for p in ports if hint in p]

    if not ports:
        raise CtrlLinkError('no se encontro ningun puerto serie USB '
                            '-- esta enchufada la placa?')
    if len(ports) > 1:
        raise CtrlLinkError(f'se encontraron varios puertos serie USB '
                            f'({", ".join(ports)}); pasar uno explicitamente '
                            f'o acotarlo con un hint')
    return ports[0]


def _ended(buf):
    """True cuando la respuesta completa a `stop` está en el buffer.

    El dispositivo contesta "# end rows=... drops=..." y **después** "# ok".
    Darse por satisfecho con la línea "# end" deja el "# ok" todavía en el
    puerto, y entonces el comando siguiente lo lee como si fuera su propio
    terminador y vuelve sin datos: de ahí un "no hay valor en la respuesta para
    'missed'" intermitente al terminar una captura. Así que se espera también la
    línea que cierra.
    """
    at = buf.find(b'# end ')
    if at < 0:
        return False

    end_of_line = buf.find(b'\n', at)
    if end_of_line < 0:
        return False

    return buf.find(b'\n', end_of_line + 1) >= 0


@dataclass
class Param:
    """Un parámetro del dispositivo, y el formato de punto fijo en que se guarda.

    `frac` es cuántos bits fraccionarios lleva el entero del dispositivo, así que
    las unidades reales son `raw / 2**frac`. Una potencia de dos en lugar de una
    escala arbitraria, para que la conversión sea exacta en los dos sentidos y no
    se pierda nada al imprimirla: ningún número fijo de decimales sirve a la vez
    para una escala Q22 (2,4e-7) y para una Q30 (9,3e-10).
    """
    name: str
    type: str
    frac: int

    @property
    def integral(self) -> bool:
        """True si el dispositivo lo guarda como entero."""
        return self.type != 'f32'

    @property
    def scale(self) -> float:
        """Unidades reales por cuenta almacenada. Exacto: una potencia de dos."""
        return 2.0 ** -self.frac


@dataclass
class Column:
    name: str
    type: str
    scale: float
    unit: str

    @property
    def dtype(self) -> np.dtype:
        return _TYPES[self.type]

    @property
    def width(self) -> int:
        """Ancho de este campo, en caracteres hexadecimales."""
        return self.dtype.itemsize * 2


class CtrlLink:
    # Estado del enlace. Son atributos de clase para que también valgan en un
    # objeto armado sin pasar por __init__.
    _depth = 0       # operaciones anidadas: sólo la de más afuera limpia
    _broken = False  # una operación se cortó por el medio y dejó el enlace sucio

    def __init__(self, port=None, baud=1_000_000, reset_wait=1.8, timeout=1.0):
        if port is None:
            port = find_port()

        # Abrir el puerto activa DTR, lo que resetea un UNO. Nada de lo que diga
        # el dispositivo antes de rearrancar y correr setup() vale la pena leerse.
        self.ser = serial.Serial(port, baud, timeout=timeout)

        try:
            time.sleep(reset_wait)
            self.ser.reset_input_buffer()

            self.info = self.sync()
            self._params = self._read_params()
            self.channels = self._read_channels()
        except BaseException:
            # El puerto ya está abierto, y un puerto serie es exclusivo. En un
            # notebook el traceback de la celda sobrevive en sys.last_traceback,
            # así que este objeto a medio construir no se recolecta y se queda
            # con el puerto: el intento siguiente falla con «no se pudo abrir el
            # puerto» y parece un problema distinto del que realmente pasó.
            self.ser.close()
            raise

    # ------------------------------------------------------------- cañerías

    def close(self):
        if not self.ser.is_open:
            return

        # Cerrar en medio de una captura deja al dispositivo emitiendo contra un
        # puerto que ya nadie lee. Reabrir lo resetea, así que no es fatal, pero
        # callarlo cuesta un comando y no vale la pena dejarlo hablando solo.
        if self._broken or self._depth:
            try:
                self._send('stop')
                self._drain(timeout=0.3)
            except Exception:
                pass

        self.ser.close()
        self._broken = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # --------------------------------------------------------- toma del enlace
    #
    # Una celda de notebook se interrumpe en cualquier parte: el botón de parar
    # en medio de una captura, una excepción a mitad de un `set`, un traceback
    # que sale de algo que no tiene nada que ver. El enlace queda entonces en un
    # estado que ninguna de las dos puntas conoce del todo —el dispositivo
    # emitiendo filas que nadie lee, media línea de comando en su buffer de
    # entrada, media respuesta en el nuestro— y la celda siguiente hereda el
    # desastre: los datos de una captura aparecen como respuesta a un `get`, y
    # el enlace parece pedir un reinicio del kernel.
    #
    # Así que toda operación toma el enlace, y si sale por una excepción lo deja
    # limpio antes de dejarla pasar. Si ni siquiera eso se logra —una segunda
    # interrupción encima de la primera, la placa desenchufada— el enlace queda
    # marcado y la operación siguiente lo intenta de nuevo antes de mandar nada.

    @contextmanager
    def _hold(self, heal=True):
        """Toma el enlace para una operación y lo deja limpio pase lo que pase.

        `heal=False` es para la limpieza misma, que no puede llamarse a sí misma.
        """
        if self._depth:
            yield                 # anidada dentro de otra operación: ya está tomado
            return

        self._depth = 1
        try:
            if heal and self._broken:
                self._resync()    # levanta si el dispositivo no se deja limpiar

            try:
                yield
            except BaseException:
                self._broken = True
                if heal:
                    self._heal()
                raise
            else:
                self._broken = False
        finally:
            self._depth = 0

    def _heal(self):
        """Limpia el enlace sin levantar nada.

        Corre mientras una excepción está saliendo, así que taparla con otra
        sería cambiar un problema por uno peor: si no puede, deja el enlace
        marcado y la operación siguiente vuelve a intentarlo.
        """
        try:
            self._resync()
        except Exception:
            return
        self._broken = False

    def resync(self, timeout=6.0):
        """Deja el enlace en un estado conocido: nada a medio enviar, el
        dispositivo callado y contestando.

        Se llama sola cuando hace falta; está expuesta para poder forzarla a
        mano después de algo que este módulo no vio pasar.
        """
        with self._hold(heal=False):
            self._resync(timeout)

    def _resync(self, timeout=6.0):
        deadline = time.monotonic() + timeout

        # Una línea vacía termina el comando que haya quedado a medio escribir,
        # que si no se pegaría adelante del siguiente. El dispositivo descarta
        # las líneas vacías, así que también es inofensiva si no había nada a
        # medias.
        self._send('')

        # `stop` es idempotente, y el dispositivo lo contesta esté emitiendo o
        # no, así que su respuesta es la prueba de que paró. El silencio no
        # alcanza: con `dec` alto una fila puede tardar más que cualquier ventana
        # de silencio razonable, y entonces un puerto callado no significa nada.
        # El `stop` mismo se puede perder en el camino de ida como cualquier otro
        # comando, así que se repite hasta que llegue la confirmación.
        seen = bytearray()

        for _ in range(4):
            self._send('stop')
            self._drain(into=seen)
            if _ended(seen):
                break
        else:
            raise CtrlLinkError('el dispositivo no contesta a "stop" -- sigue '
                                'emitiendo, o dejo de escuchar; desenchufar y '
                                'volver a enchufar la placa')

        return self.sync(timeout=max(1.0, deadline - time.monotonic()))

    def _drain(self, into=None, settle=0.15, timeout=1.0):
        """Lee del puerto hasta que no quede nada que leer.

        Que no quede nada es que el puerto pase `settle` segundos callado, o que
        `into` —donde se va acumulando lo leído, si se lo pasa— ya contenga la
        respuesta completa a un `stop`, que es lo último que el dispositivo tiene
        para decir. Devuelve False si se agotó `timeout` con el dispositivo
        todavía hablando, que es lo que pasa cuando el flujo no paró.
        """
        deadline = time.monotonic() + timeout
        quiet_since = time.monotonic()

        while True:
            now = time.monotonic()
            waiting = self.ser.in_waiting

            if waiting:
                chunk = self.ser.read(waiting)
                if into is not None:
                    into += chunk
                    if _ended(into):
                        return True
                quiet_since = now
            elif now - quiet_since >= settle:
                return True
            elif now >= deadline:
                return False
            else:
                time.sleep(0.005)

    # ------------------------------------------------------------- comandos

    def _readline(self, deadline) -> str | None:
        """Una línea, o None una vez pasado `deadline`."""
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if raw:
                return raw.decode('ascii', 'replace').rstrip('\r\n')
        return None

    def _send(self, line):
        """Escribe un comando con sus bytes espaciados. Ver _BYTE_GAP."""
        for byte in (line + '\n').encode('ascii'):
            self.ser.write(bytes([byte]))
            if _BYTE_GAP:
                _pause(_BYTE_GAP)
        self.ser.flush()

    def cmd(self, line, timeout=2.0, tries=3) -> list[str]:
        """Envía un comando y devuelve las líneas de su respuesta, terminador incluido.

        Una respuesta que diga que el dispositivo no entendió el comando significa
        que se perdieron bytes en el camino de ida, así que el comando se
        reenvía. Un error que signifique que el dispositivo entendió y objetó se
        levanta de inmediato.

        Las filas de telemetría que lleguen mientras la respuesta está en vuelo se
        descartan, así que es seguro llamar a esto en medio de una captura; pero
        durante una captura conviene el argumento `events` de capture(), que
        conserva las filas.
        """
        with self._hold():
            return self._cmd(line, timeout, tries)

    def _cmd(self, line, timeout, tries):
        for attempt in range(tries):
            self._send(line)

            deadline = time.monotonic() + timeout
            reply = []

            while True:
                text = self._readline(deadline)
                if text is None:
                    raise CtrlLinkError(f'se agoto la espera de una respuesta a {line!r}')
                if not text.startswith('#'):
                    continue  # una fila de telemetria que se adelanto a la respuesta
                reply.append(text)
                if not text.startswith(_TERMINATORS):
                    continue

                if not text.startswith('# err'):
                    return reply

                reason = text[6:]
                if attempt + 1 < tries and any(g in reason for g in _GARBLED):
                    break  # el comando se deformo en transito; se manda de nuevo
                raise CtrlLinkError(f'{line!r}: {reason}')

    def sync(self, timeout=4.0) -> str:
        """Vacía lo que el dispositivo estuviera diciendo y confirma que escucha."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.ser.reset_input_buffer()
            try:
                for text in self.cmd('id', timeout=0.5):
                    if text.startswith('# id '):
                        return text[5:]
            except CtrlLinkError:
                continue
        raise CtrlLinkError('no hubo respuesta a "id" -- puerto equivocado, '
                            'velocidad equivocada, o el sketch no esta '
                            'corriendo CtrlLink')

    # ----------------------------------------------------------- descubrimiento

    def _read_params(self) -> dict:
        """nombre -> Param, tal como los declara el dispositivo.

        El formato es el del propio dispositivo: guarda cada parámetro en la forma
        de punto fijo que su aritmética prefiera y dice cuántos bits fraccionarios
        son. Todo lo que está por encima de esta línea trabaja en unidades
        naturales y nunca ve el entero.
        """
        params = {}
        for text in self.cmd('params'):
            if not text.startswith('# p '):
                continue

            fields = text[4:].split(None, 3)
            if len(fields) < 4:
                # Antes de que los parámetros cruzaran el cable en punto fijo no
                # existía la columna de bits fraccionarios. Vale la pena decirlo
                # con todas las letras: lo que se ve si no es un ValueError de
                # desempaquetado, que no lleva a ninguna parte.
                raise CtrlLinkError(
                    f'el dispositivo declara sus parametros en un formato '
                    f'anterior ({text!r}): tiene grabado un sketch viejo. '
                    f'Volver a grabarlo -- sync_board(force_upload=True) lo hace.')

            name, type_, frac, _value = fields
            params[name] = Param(name, type_, int(frac))
        return params

    def _read_channels(self) -> list[Column]:
        chans = []
        for text in self.cmd('chans'):
            if text.startswith('# c '):
                _, name, type_, scale, *unit = text[4:].split()
                chans.append(Column(name, type_, float(scale),
                                    unit[0] if unit else ''))
        return chans

    @property
    def dt(self) -> float:
        """Período de control en segundos, preguntado al dispositivo y no supuesto.

        Se mueve cuando se mueve `tickdiv`, y cualquier conversión entre
        magnitudes de tiempo continuo y por muestra necesita el valor vigente.
        """
        for field in self.cmd('id')[0].split():
            if field.startswith('dt_us='):
                return int(field[6:]) * 1e-6
        raise CtrlLinkError('el dispositivo no informo su periodo de control')

    @property
    def _health(self) -> tuple:
        """Los contadores de salud que este sketch en particular exponga.

        Se deduce de la tabla de parámetros en lugar de guardarse en caché, así
        que acompaña a un dispositivo que se haya conectado a mano y no a través
        de __init__.
        """
        return tuple(name for name in _HEALTH if name in self._params)

    @property
    def _state(self) -> tuple:
        """Los parámetros de estado que este sketch exponga. Ver _STATUS."""
        return tuple(name for name in _STATUS if name in self._params)

    @property
    def params(self) -> dict:
        """Cada parámetro y su valor actual, releídos del dispositivo."""
        return {name: self.get(name) for name in self._params}

    def get(self, name):
        """El valor del parámetro, en unidades reales."""
        for text in self.cmd(f'get {name}'):
            if text.startswith('# v '):
                _, value = text[4:].split(None, 1)
                return self._coerce(name, value)
        raise CtrlLinkError(f'no hay valor en la respuesta para {name!r}')

    def set(self, name, value, tries=3):
        """Fija un parámetro, en unidades reales, y confirma lo que guardó el dispositivo.

        Un comando corrompido en general se rechaza de plano, pero un *valor*
        deformado se aceptaría en silencio, así que el valor devuelto se verifica
        en lugar de darse por bueno. La verificación contempla la cuantización del
        propio dispositivo: un parámetro guardado en punto fijo no puede contener
        todos los valores que se le pidan, y redondear al representable más cercano
        es lo correcto, no un error.
        """
        stored = None

        for attempt in range(tries):
            for text in self.cmd(f'set {name} {self._encode(name, value)}'):
                if not text.startswith('# v '):
                    continue
                _, echoed = text[4:].split(None, 1)
                stored = self._coerce(name, echoed)
                if self._agrees(name, stored, value):
                    return stored
                break
            if attempt + 1 == tries:
                raise CtrlLinkError(
                    f'se fijo {name} en {value!r} pero el dispositivo '
                    f'informa {stored!r}')
        return None

    def _encode(self, name, value):
        """Unidades reales -> el entero (o float) que el dispositivo quiere en el cable."""
        param = self._params[name]

        if not param.integral:
            return float(value)
        return int(round(float(value) * 2.0 ** param.frac))

    def _coerce(self, name, text):
        """El entero (o float) del cable -> unidades reales."""
        param = self._params[name]

        if not param.integral:
            return float(text)
        raw = int(text)
        return raw * param.scale if param.frac else raw

    def _agrees(self, name, stored, requested):
        try:
            wanted = float(requested)
        except (TypeError, ValueError):
            return False

        param = self._params[name]

        # Medio paso de lo que el dispositivo realmente puede representar, más
        # lugar para los seis decimales con los que imprime los float.
        slack = max(1e-6, abs(wanted) * 1e-6, abs(param.scale) / 2)
        return abs(float(stored) - wanted) <= slack

    # Los parámetros como atributos, para que en un notebook se lea
    # `dev.kp = 2.5`. Todo lo que no esté en la tabla del dispositivo cae en el
    # manejo normal de atributos.
    def __getattr__(self, name):
        params = self.__dict__.get('_params', {})
        if name in params:
            return self.get(name)
        raise AttributeError(name)

    def __setattr__(self, name, value):
        params = self.__dict__.get('_params', {})
        if name in params:
            self.set(name, value)
        else:
            super().__setattr__(name, value)

    def __dir__(self):
        return list(super().__dir__()) + list(self.__dict__.get('_params', {}))

    # -------------------------------------------------------------- captura

    def capture(self, duration, events=(), poll=0.005, warn=True):
        """Emite durante `duration` segundos y devuelve un DataFrame.

        `events` es una secuencia de (retardo_s, nombre, valor): cada parámetro se
        fija esa cantidad de segundos después de que arranca el flujo. El
        dispositivo informa el tick exacto en el que cayó cada set, así que la
        fluctuación de programación de la computadora no entra en la medición; ver
        `df.attrs['marks']`.

        Los contadores de salud del dispositivo se ponen en cero antes de la
        corrida y se leen después, así que `df.attrs` dice si el lazo realmente
        llegó mientras se producían estas filas en particular. Todo lo que ande mal
        además se imprime, porque una captura que perdió períodos en silencio se ve
        exactamente igual que una que no hasta que uno va a fijarse. Pasar
        `warn=False` para tener los números sin el comentario.

        Si la captura se interrumpe —el botón de parar del notebook, un
        Ctrl-C—, el dispositivo queda callado igual antes de que la excepción
        llegue a la celda: ver _hold().
        """
        with self._hold():
            return self._capture(duration, events, poll, warn)

    def _capture(self, duration, events, poll, warn):
        pending = sorted(events, key=lambda e: e[0])

        for name in self._health:
            self.set(name, 0)

        self.ser.reset_input_buffer()
        header = self.cmd('start')
        columns, dt_us, dec = self._parse_header(header)

        buf = bytearray()
        started = time.monotonic()
        deadline = started + duration

        while True:
            now = time.monotonic()

            while pending and now - started >= pending[0][0]:
                _, name, value = pending.pop(0)
                self._send(f'set {name} {value}')

            if now >= deadline:
                break

            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                buf += chunk
            elif not pending:
                time.sleep(poll)

        # Seguir leyendo hasta que el dispositivo confirme que paró, para tener a
        # mano tanto la cola del flujo como los recuentos finales de filas. Tiene
        # que estar la línea entera, no sólo sus primeros bytes: esperar apenas el
        # prefijo "# end" le entrega al decodificador una línea cortada a mitad de
        # campo.
        #
        # Un "stop" se puede perder en el camino de ida como cualquier otro
        # comando, y no hay respuesta contra la cual reintentar hasta que el flujo
        # efectivamente termine, así que simplemente se repite hasta que el
        # dispositivo conteste.
        for _ in range(4):
            self._send('stop')

            tail_deadline = time.monotonic() + 1.5
            while not _ended(buf) and time.monotonic() < tail_deadline:
                chunk = self.ser.read(max(1, self.ser.in_waiting))
                if chunk:
                    buf += chunk
                else:
                    time.sleep(poll)

            if _ended(buf):
                break
        else:
            raise CtrlLinkError('el dispositivo no dejo de emitir')

        # Duración real de la ventana de emisión, medida entre el momento en que
        # el dispositivo confirmó `start` y aquel en que confirmó `stop`. Es la
        # única referencia de tiempo independiente que hay: los ticks los cuenta
        # el dispositivo y avanzan una vez por período *atendido*, así que
        # dividir filas por ticks da el período nominal pase lo que pase y no
        # puede delatar un lazo que no llega.
        wall = time.monotonic() - started

        df = self._decode(buf, columns, dt_us, dec)
        df.attrs['wall'] = wall

        # Se leen una vez que el flujo paró, no durante: un `get` en medio de una
        # captura cuesta milisegundos de tráfico de comandos, que es justamente lo
        # que se está midiendo.
        df.attrs.update({name: self.get(name)
                         for name in self._health + self._state})
        df.attrs['health'] = self._health_notes(df)

        if warn:
            for note in df.attrs['health']:
                print(f'ctrllink: {note}', file=sys.stderr)

        return df

    def health(self):
        """Los contadores de salud del dispositivo y su estado, como dict.

        Vacío si el sketch no expone ninguno.
        """
        return {name: self.get(name) for name in self._health + self._state}

    def _health_notes(self, df):
        """Quejas en castellano llano sobre una captura, la peor primero.

        Todo lo que hay acá es una forma de que la serie temporal esté mal sin
        parecerlo: un período perdido es una muestra que el controlador nunca
        calculó, y una fila descartada es una que calculó y nunca envió. Ninguna de
        las dos deja marca en los datos mismos.
        """
        notes = []
        dt_us = df.attrs['dt_us']
        rate = 1e6 / dt_us if dt_us else 0

        # Primero, porque si el sensor no está todo lo demás que se mida es
        # consecuencia de eso y no un problema por derecho propio.
        absent = df.attrs.get('spres') == 0
        if absent:
            notes.append(
                'el AS5600 no contesta en el bus I2C: revisar SDA (A4), '
                'SCL (A5), la alimentacion y los pull-ups. El lazo sigue '
                'corriendo, pero el angulo queda congelado y todo lo que se '
                'mida de posicion no significa nada.')

        missed = df.attrs.get('missed') or 0
        if missed:
            notes.append(
                f'se perdieron {missed} periodo(s) de control '
                f'({missed / max(rate, 1):.3f} s de tiempo de lazo): el '
                f'muestreador volvio a pasar antes de que se atendiera el tick '
                f'anterior, asi que esos periodos directamente no corrieron. '
                f'Subir tickdiv, o sacarle trabajo al paso de control.')

        late = df.attrs.get('maxlate')
        if late is not None and dt_us and late > dt_us * _LATE_WARN:
            notes.append(
                f'peor retardo de atencion {late} us contra un periodo de '
                f'{dt_us} us ({late / dt_us:.0%}): el lazo llego, pero por poco.')

        drops = df.attrs.get('drops') or 0
        if drops:
            notes.append(
                f'se descartaron {drops} fila(s) de telemetria: el dispositivo no '
                f'tenia lugar en su buffer de transmision. Subir dec, o emitir '
                f'menos canales.')

        sent = df.attrs.get('rows')
        if sent and len(df) < sent:
            notes.append(
                f'{sent - len(df)} de {sent} fila(s) enviadas nunca llegaron: se '
                f'perdieron bytes entre el dispositivo y aca.')

        gaps = df.attrs.get('gaps') or 0
        if gaps and not (drops or (sent and len(df) < sent)):
            notes.append(f'{gaps} hueco(s) en la secuencia de ticks: faltan filas '
                         f'en la serie temporal.')

        sovr = df.attrs.get('sovr') or 0
        if sovr:
            notes.append(
                f'{sovr} desborde(s) del sensor: una transferencia de I2C no habia '
                f'terminado cuando vencia la muestra siguiente, asi que esa muestra '
                f'repite la anterior.')

        # Con el sensor ausente las fallas son las del sondeo espaciado, que ya
        # quedaron explicadas arriba; contarlas de nuevo sólo agrega ruido. Con
        # el sensor presente, en cambio, son intermitencias y ésas sí importan.
        serr = df.attrs.get('serr') or 0
        if serr and not absent:
            notes.append(f'fallaron {serr} transferencia(s) del sensor -- revisar '
                         f'el cableado y los pull-ups del bus.')

        return notes

    def step(self, name, value, pre=0.1, post=0.9, back=None, warn=True):
        """Captura una respuesta al escalón, con `t = 0` en el escalón mismo.

        Mantiene `pre` segundos, pone `name` en `value`, y mantiene `post`
        segundos más. Si se da `back`, el parámetro se restituye al final.
        """
        try:
            df = self.capture(pre + post, events=[(pre, name, value)], warn=warn)
        except BaseException:
            # El escalón ya salió: interrumpir la captura no lo deshace, y dejar
            # una referencia en pie contra un motor no es un estado en el que
            # convenga abandonar el equipo. El enlace ya quedó limpio para este
            # punto, así que restituir es un comando común; si aun así no se
            # puede, la excepción que viene saliendo es la noticia importante.
            if back is not None:
                try:
                    self.set(name, back)
                except Exception:
                    pass
            raise

        marks = [m for m in df.attrs['marks'] if m[1] == name]
        if marks:
            # Se desplaza en el espacio de ticks, no en segundos: restar dos float
            # que son cada uno un tick por un período deja un residuo de redondeo,
            # y un t de -5e-17 pone la muestra del escalón del lado equivocado de
            # t < 0.
            origin = self._mark_tick(df, marks[0][0])
            df['t'] = (df.attrs['tick'] - origin) * (df.attrs['dt_us'] * 1e-6)

        if back is not None:
            self.set(name, back)

        return df

    @staticmethod
    def _mark_tick(df, raw_tick):
        """Tick desenrollado de una marca, que el dispositivo informa en 16 bits crudos.

        Los ticks del propio cuadro ya están desenrollados, así que los dos viven
        en espacios distintos y no se pueden restar sin más.
        """
        tick = df.attrs['tick']

        if len(tick) == 0:
            return 0

        here = np.flatnonzero((tick & 0xFFFF) == raw_tick)
        if len(here):
            return int(tick[here[0]])

        # La diezmación o una fila descartada pueden dejar al tick marcado sin
        # fila propia; se lo ubica por cuánto queda pasado el primer tick de la
        # captura.
        return int(tick[0] + ((raw_tick - (tick[0] & 0xFFFF)) & 0xFFFF))

    # ---------------------------------------------------------- decodificación

    @staticmethod
    def _parse_header(lines):
        columns, dt_us, dec = [], None, 1

        for text in lines:
            if text.startswith('# col '):
                name, type_, scale, *unit = text[6:].split()
                columns.append(Column(name, type_, float(scale),
                                      unit[0] if unit else ''))
            elif text.startswith('# rate '):
                fields = dict(f.split('=') for f in text[7:].split())
                dt_us = int(fields['dt_us'])
                dec = int(fields['dec'])

        if not columns or dt_us is None:
            raise CtrlLinkError('el dispositivo no envio un encabezado de flujo utilizable')

        return columns, dt_us, dec

    def _decode(self, buf, columns, dt_us, dec):
        import pandas as pd

        dtype = np.dtype([(c.name, c.dtype) for c in columns])
        width = sum(c.width for c in columns)

        # println() emite CRLF y las filas emiten LF pelado; el hexadecimal nunca
        # contiene CR, así que descartar todos los CR de entrada uniformiza los dos
        # tipos de línea.
        lines = buf.replace(b'\r', b'').split(b'\n')

        marks, notes = [], []
        stats = {'rows': None, 'drops': None}
        rows = []

        for line in lines:
            if line.startswith(b'#'):
                text = line.decode('ascii', 'replace')
                if text.startswith('# mark '):
                    tick, name, value = text[7:].split(None, 2)
                    marks.append((int(tick), name, self._coerce(name, value)))
                elif text.startswith('# note '):
                    notes.append(text[7:])
                elif text.startswith('# end '):
                    stats.update((k, int(v)) for k, v in
                                 (f.split('=', 1) for f in text[6:].split()
                                  if '=' in f))
            elif len(line) == width:
                rows.append(line)

        raw = bytes.fromhex(b''.join(rows).decode('ascii')) if rows else b''

        # El cable es big-endian, así que frombuffer devuelve campos big-endian.
        # pandas se niega a indexarlos en una máquina little-endian, y algunos de
        # sus caminos devuelven los valores equivocados en silencio en lugar de dar
        # error, así que el arreglo se lleva al orden nativo antes de que nada más
        # lo toque.
        arr = np.frombuffer(raw, dtype=dtype).astype(dtype.newbyteorder('='))

        tick = self._unwrap(arr['tick'].astype(np.int64))

        df = pd.DataFrame({'t': tick * dt_us * 1e-6})
        for column in columns:
            if column.name == 'tick':
                continue
            values = arr[column.name]
            df[column.name] = values * column.scale if column.scale != 1.0 else values

        # Un hueco en la secuencia de ticks es una fila que el dispositivo descartó
        # o una que se perdió en el camino. En cualquier caso es un agujero en la
        # serie temporal, no una pausa.
        gaps = int(np.count_nonzero(np.diff(tick) != dec)) if len(tick) > 1 else 0

        df.attrs.update(dt_us=dt_us, dec=dec, marks=marks, notes=notes,
                        gaps=gaps, tick=tick,
                        units={c.name: c.unit for c in columns},
                        **stats)
        return df

    @staticmethod
    def _unwrap(tick):
        """Deshace la vuelta al cero del tick de 16 bits del dispositivo."""
        if len(tick) < 2:
            return tick
        wraps = np.concatenate([[0], np.cumsum(np.diff(tick) < 0)])
        return tick + wraps * 65536
