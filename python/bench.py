"""El banco: compilación, conexión y las verificaciones de este equipo.

Todo lo que hay acá es cañería. Vive fuera del notebook para que una celda del
notebook contenga un experimento y nada más.

    from bench import *

    dev = sync_board()
    dev.uff = 120                                   # un comando sobre el puente
    df = dev.step('uff', 200, pre=0.3, post=1.2)    # un escalón, t = 0 en el escalón

`sync_board()` compila si cambió algún archivo fuente, carga si cambió el binario,
y reabre el enlace, lo que resetea la placa. Todas las celdas lo llaman, así que
cada celda arranca desde los valores por omisión del propio sketch y ninguna
depende de que se haya corrido la de arriba.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
import time
from pathlib import Path

import serial

from ctrllink import CtrlLink, CtrlLinkError, find_port

__all__ = ['sync_board', 'sync_board_cal', 'Bench', 'CtrlLinkError', 'CALIBRACION']

FQBN = 'arduino:avr:uno'

# Con qué bootloader habla la placa. Un UNO escucha a 115200; muchos clones
# baratos traen el bootloader viejo del Nano, que escucha a 57600, y elegir mal no
# da un error legible sino diez líneas de «not in sync». Se prueban en orden y se
# recuerda cuál anduvo, por puerto, en `sync-state.json`.
UPLOAD_FQBNS = [
    ('arduino:avr:uno',                    'UNO'),
    ('arduino:avr:nano:cpu=atmega328old',  'clon con bootloader viejo'),
]

_HERE     = Path(__file__).resolve().parent.parent
SKETCH    = _HERE / 'Banco'
LIBRARIES = _HERE / 'libraries'
BUILD_DIR = _HERE / 'build'

# La calibración del sensor de este banco. No entra en el repositorio --es un dato
# del banco y no del proyecto-- y la ruta se resuelve desde este archivo, así que
# no depende de desde dónde se corra el notebook. Ver sync_board_cal().
CALIBRACION = _HERE / 'notebooks' / 'calibracion.json'

# El core de AVR compila con `-Os` --optimizar por tamaño--, y este sketch quiere
# ciclos y no bytes: el muestreador de 5 kHz y la telemetría comparten el tiempo
# con el manejador del bus. `-O2` es `-Os` más todo lo que agrande el código, y
# no `-O3`, que en un AVR de 32 kB se paga con casi todo el espacio que queda.
# Las banderas van como `extra_flags` para heredar todo lo demás del core.
BUILD_PROPERTIES = [
    'compiler.c.extra_flags=-O2',
    'compiler.cpp.extra_flags=-O2',
    'compiler.c.elf.extra_flags=-O2',
]

# Bits del registro STATUS del AS5600.
_MAGNET_STRONG, _MAGNET_WEAK, _MAGNET_PRESENT = 0x08, 0x10, 0x20

# La placa cuenta la corriente en 12 bits, sea cual sea su ADC. Dónde tiene que
# reposar el sensor depende de cuál sea y de cómo esté alimentado, así que acá no
# se juzga el valor: se juzga que quede fuera de los rieles --contra un riel no
# hay una corriente grande sino una entrada al aire-- y que sobre margen para que
# una corriente tenga adónde crecer.
_ADC_FULL     = 4095
_ADC_RAIL     = 80    # a menos de esto de cualquiera de los dos extremos
_ADC_HEADROOM = 400   # cuentas de margen que se le piden al reposo

# Menos que esto no es un motor que giró: es el ruido del ángulo.
_GIRO_MINIMO = 0.05   # vueltas

_link = None


# -------------------------------------------------------------- el dispositivo

class Bench(CtrlLink):
    """Un CtrlLink que sabe qué significan los números de este equipo."""

    def channel(self, name):
        for column in self.channels:
            if column.name == name:
                return column
        raise CtrlLinkError(f'no hay ningun canal llamado {name!r}')

    def rest(self):
        """Comando en cero. Donde tendría que terminar todo experimento.

        Con el comando en cero el sketch deja ENA en bajo, así que el puente queda
        abierto y el motor en punto muerto: no frena el eje, sólo deja de
        empujarlo.
        """
        self.uff = 0

    def zero_current(self, seconds=0.3):
        """Toma la corriente que se mida ahora como el cero. Devuelve `izero`.

        Con el puente abierto no circula corriente, así que lo que marque el sensor
        es su offset: el suyo propio, más la tolerancia de su alimentación y la de
        cualquier divisor que haya en el medio. Un cero corrido es un error que
        después se integra en toda medición, y no hay forma de conocerlo salvo
        midiéndolo acá.

        `i` se lee como `adc - izero`, así que sumarle a `izero` la lectura actual
        pone el cero sobre la cuenta actual y `i` en cero. Deja el motor en reposo,
        que es la condición bajo la cual la medición significa algo.
        """
        self.rest()
        df = self.capture(seconds, warn=False)
        self.izero = round(self.izero + df['i'].mean() / self.channel('i').scale)
        return self.izero

    def _tiron(self, u, seconds=0.4, espera=6.0, quieto=5.0):
        """`u` sobre el puente por un instante, y de vuelta a reposo.

        Devuelve (vueltas, mA de pico). Las vueltas van con signo. Espera primero
        a que el eje esté realmente quieto: con el puente abierto el motor no
        frena, sigue por inercia varios segundos, y midiendo enseguida lo que se
        mide es el giro anterior. `quieto` es el umbral en grados por segundo.
        """
        self.rest()

        limite = time.monotonic() + espera
        while True:
            reposo = self.capture(0.2, warn=False)
            arrastre = abs(reposo['y_uw'].iloc[-1] - reposo['y_uw'].iloc[0]) / 0.2
            if arrastre < quieto or time.monotonic() > limite:
                break

        self.uff = u
        df = self.capture(seconds, warn=False)
        self.rest()

        if arrastre >= quieto:
            print(f'  OJO: el eje seguia girando a {arrastre:.0f} grados/s al empezar '
                  f'esta medicion')

        vueltas = (df['y_uw'].iloc[-1] - df['y_uw'].iloc[0]) / 360.0
        return vueltas, df['i'].abs().max()

    # ------------------------------------------------------- puesta en marcha

    def bringup(self, motor=True, u=120):
        """Verifica el hardware, un subsistema por vez. Devuelve si pasó todo.

        Cada línea es algo que puede estar mal por su cuenta: el ritmo de las
        filas, el sensor, el imán, el bus I2C, el cero de la medición de
        corriente --que de paso se calibra-- y el actuador. Conviene correrlo
        primero, y después de cualquier cambio en el cableado.

        El signo del ángulo no se corrige acá: un comando positivo que hace
        bajar el ángulo es de qué lado están los cables del motor y de qué lado
        mira el imán, y en lazo abierto se arregla con un signo del lado de la
        computadora. Ver `ensayo.signo()`.
        """
        results = []

        def report(label, ok, detail):
            results.append(ok)
            tag = {True: 'ok', False: 'FALLA', None: 'nota'}[ok]
            print(f'  [{tag:>5}]  {label:<18}  {detail}')

        print(f'puesta en marcha: {self.info}')

        self.rest()

        # 1. El ritmo de las filas, medido contra el reloj de esta máquina. Con
        #    el contador de ticks del dispositivo no se puede: avanza una vez por
        #    período *atendido*, así que filas sobre ticks devuelve el período
        #    nominal aunque se pierda la mitad de los períodos.
        df = self.capture(1.0, warn=False)
        served = len(df) * df.attrs['dec']
        missed = df.attrs['missed']
        rate   = served / df.attrs['wall']
        want   = 1e6 / df.attrs['dt_us']
        report('muestreo', missed == 0 and abs(rate - want) < want * 0.05,
               f'{rate:.0f} Hz reales contra {want:.0f} nominales, '
               f'{missed} perdidos, {df.attrs["drops"]} descartados')

        late   = df.attrs['maxlate']
        margin = late / df.attrs['dt_us']
        report('margen de tiempo',
               True if margin < 0.5 else (None if margin < 1.0 else False),
               f'peor retardo de atencion {late} us de {df.attrs["dt_us"]} us '
               f'({margin:.0%})')

        # 2. El sensor, antes que el imán: si el AS5600 no contesta en el bus, lo
        #    que diga su registro del imán no significa nada.
        present = bool(df.attrs.get('spres', 1))
        report('sensor', present,
               'contesta en el bus' if present else
               'no contesta -- revisar SDA (A4), SCL (A5), alimentacion y pull-ups')

        # 3. El imán, tal como lo ve el propio AS5600.
        status = df.attrs.get('mstat', 0)
        if not present:
            report('iman', None, 'no se puede evaluar sin el sensor')
        elif not status & _MAGNET_PRESENT:
            report('iman', False, 'no se detecta -- esta montado sobre el chip?')
        elif status & _MAGNET_WEAK:
            report('iman', False, 'muy debil (AGC al maximo) -- acercarlo')
        elif status & _MAGNET_STRONG:
            report('iman', False, 'muy fuerte (AGC al minimo) -- alejarlo')
        else:
            report('iman', True, 'detectado, AGC en rango')

        # 4. El bus. Un desborde no es un error: es una muestra que el bus no
        #    llegó a entregar antes del tick siguiente, y un puñado por segundo
        #    es normal --la lectura del estado del imán no entra en 200 us--. Lo
        #    que es una falla es que el bus no llegue de manera sostenida.
        if present:
            muestras = df.attrs['wall'] * 1e6 / df.attrs['dt_us'] * self.tickdiv
            tasa = df.attrs['sovr'] / max(muestras, 1)
            report('bus i2c', df.attrs['serr'] == 0 and tasa < 0.005,
                   f'{df.attrs["serr"]} errores de transferencia, '
                   f'{df.attrs["sovr"]} desbordes ({tasa:.2%} de las muestras)')
        else:
            report('bus i2c', None,
                   f'{df.attrs["serr"]} fallas, todas del sondeo al sensor ausente')

        spread = df['y_uw'].max() - df['y_uw'].min()
        report('angulo', None if spread < 0.5 else True,
               f'{df["y_uw"].iloc[-1]:.1f} grados, se movio {spread:.2f} grados '
               f'en el segundo' + ('  (girar el iman para verlo seguir)'
                                   if spread < 0.5 else ''))

        # 5. El cero de la medición de corriente, que acá se mide y se calibra.
        #    Una entrada al aire termina contra un riel del ADC: eso es una
        #    ausencia, no un offset, y calibrarla dejaría un canal que informa
        #    ceros perfectos sin haber medido nada. Y un reposo pegado a un
        #    extremo no deja lugar para medir, aunque no llegue al riel.
        lsb = self.channel('i').scale
        adc = self.izero + df['i'].mean() / lsb
        sensed = _ADC_RAIL <= adc <= (_ADC_FULL - _ADC_RAIL)
        rest_ma = 0.0

        if not sensed:
            report('cero de i', None,
                   f'entrada contra el riel del ADC ({adc:.0f} de {_ADC_FULL}): no '
                   f'parece haber nada conectado en A0')
        else:
            up, down = _ADC_FULL - adc, adc
            report('cero de i', min(up, down) >= _ADC_HEADROOM,
                   f'{adc:.0f} de {_ADC_FULL}, margen +{up * lsb / 1000:.1f} A / '
                   f'-{down * lsb / 1000:.1f} A'
                   + ('' if min(up, down) >= _ADC_HEADROOM else
                      '  -- el reposo esta muy cerca del tope: sin lugar para medir'))

            self.zero_current()
            zeroed  = self.capture(0.3, warn=False)
            rest_ma = zeroed['i'].mean()
            noise   = zeroed['i'].std()

            # Un canal demasiado quieto es tan sospechoso como uno ruidoso: un
            # ruido de cero exacto es una señal más chica que un escalón del ADC.
            report('calibracion de i',
                   abs(rest_ma) < lsb and 0.1 * lsb < noise < 8 * lsb,
                   f'izero = {self.izero}, {lsb:.1f} mA por cuenta, quedan '
                   f'{rest_ma:+.1f} mA en reposo (ruido {noise / lsb:.2f} cuentas)'
                   + ('' if noise > 0.1 * lsb else
                      '  -- sin dither: la senal no llega a un escalon del ADC'))

        # 6. El actuador, y con él toda la cadena: un comando que sale, movimiento
        #    y corriente que vuelven. Sólo se usa la evidencia cuyo sensor está.
        if not motor:
            report('motor', None, 'omitido (motor=False)')
        else:
            print(f'  accionando el motor con u = {u:+} durante 0,4 s ...')
            vueltas, pico = self._tiron(u)

            evidence = ([f'{abs(vueltas):.2f} vueltas'] if present else []) + \
                       ([f'{pico:.0f} mA de pico'] if sensed else [])

            if not evidence:
                report('motor', None, 'no se puede evaluar: no hay sensor de '
                                      'angulo ni medicion de corriente')
            else:
                report('motor',
                       (present and abs(vueltas) > _GIRO_MINIMO) or
                       (sensed and pico > abs(rest_ma) + 50),
                       ', '.join(evidence))

            if present and vueltas < -_GIRO_MINIMO:
                report('signo', None,
                       'un comando positivo hace BAJAR el angulo: ensayo.signo() '
                       'lo da vuelta al procesar, o dar vuelta los dos cables '
                       'del motor')

        bad = results.count(False)
        print(f'\n{"todas las verificaciones pasaron" if not bad else f"FALLARON {bad} verificacion(es)"}')
        return not bad


# ------------------------------------------------------- compilación y carga

def _sources_hash():
    """Huella digital de todo aquello a partir de lo cual se construye el sketch.

    Por contenido y no por marca de tiempo: un checkout de git reescribe las
    mtime sin cambiar una línea, y si no dispararía una recompilación al pedo.
    Las banderas de compilación entran en la huella junto con las fuentes.
    """
    digest = hashlib.sha256()
    digest.update(repr(BUILD_PROPERTIES).encode())
    files = sorted(list(SKETCH.glob('*.ino')) + list(SKETCH.glob('*.h')) +
                   [p for p in LIBRARIES.rglob('*') if p.suffix in ('.h', '.cpp', '.c')])
    for path in files:
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _run(argv, what):
    """Corre una herramienta de compilación y devuelve su salida, o explica por qué no pudo.

    La codificación se fija en lugar de dejarla al locale: arduino-cli emite
    UTF-8, y una consola de Windows con cp1252 por omisión convierte un carácter
    perdido en un diagnóstico del compilador en un UnicodeDecodeError que esconde
    el error de verdad.
    """
    try:
        done = subprocess.run(argv, capture_output=True,
                              encoding='utf-8', errors='replace')
    except FileNotFoundError:
        raise RuntimeError(
            f'{argv[0]} no se encontro en el PATH, asi que no se puede {what} el '
            f'sketch.\n'
            f'Instalar el Arduino CLI y asegurarse de que la terminal que arranco '
            f'este kernel lo vea: en Windows eso normalmente significa reabrir la '
            f'terminal despues de instalarlo, porque el PATH se lee una sola vez '
            f'al arrancar.'
        ) from None

    if done.returncode:
        raise RuntimeError(f'fallo al {what}:\n{(done.stdout + done.stderr).strip()}')
    return done.stdout + done.stderr


def _load_state():
    try:
        return json.loads((BUILD_DIR / 'sync-state.json').read_text())
    except (OSError, ValueError):
        return {}


def _save_state(state):
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    (BUILD_DIR / 'sync-state.json').write_text(json.dumps(state, indent=1))


def _upload(port, state, say):
    """Carga el binario, averiguando sola con qué bootloader habla esta placa.

    Devuelve el FQBN que anduvo, y lo deja anotado en el estado para la próxima
    vez. Si la anotación quedó vieja el intento falla y se sigue con los otros
    perfiles, así que la memoria acelera pero no decide.
    """
    recordado = state.get('bootloader', {}).get(port)
    orden = ([f for f in UPLOAD_FQBNS if f[0] == recordado] +
             [f for f in UPLOAD_FQBNS if f[0] != recordado])

    fallas = []
    for fqbn, nombre in orden:
        if fallas:
            say(f'  no era {dict(UPLOAD_FQBNS)[fallas[-1][0]]}; probando '
                f'{nombre} ...')
        try:
            _run(['arduino-cli', 'upload', '--fqbn', fqbn, '-p', port,
                  '--input-dir', str(BUILD_DIR), str(SKETCH)], 'cargar')
        except RuntimeError as exc:
            fallas.append((fqbn, exc))
            continue

        state.setdefault('bootloader', {})[port] = fqbn
        _save_state(state)
        return fqbn

    detalle = '\n\n'.join(f'--- como {dict(UPLOAD_FQBNS)[f]}:\n{e}'
                          for f, e in fallas)
    raise RuntimeError(
        f'no se pudo cargar el sketch en {port} con ninguno de los bootloaders '
        f'conocidos ({", ".join(n for _, n in UPLOAD_FQBNS)}).\n'
        f'«not in sync» en todos suele ser la placa tomada por otro programa --el '
        f'monitor serie del IDE, un kernel viejo-- o un cable de solo '
        f'alimentacion. Si la placa es de un tipo que no esta en la lista, '
        f'agregarlo a UPLOAD_FQBNS en bench.py.\n\n{detalle}')


def _wait_for_port(hint=None, timeout=2.0):
    """find_port(), pero tolerante con una placa que todavía se está reenumerando."""
    deadline = time.monotonic() + timeout
    while True:
        try:
            return find_port(hint)
        except CtrlLinkError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.3)


def sync_board(port=None, force_compile=False, force_upload=False, verbose=True):
    """Pone al día la placa y el enlace, y reconecta. Devuelve un Bench.

    Compila sólo cuando algún archivo fuente cambió de verdad, carga sólo cuando
    el binario resultante difiere del que este puerto recibió por última vez, y
    siempre reabre el enlace, lo que resetea la placa, así que el sketch arranca
    desde sus valores por omisión haya hecho falta o no grabar. Ese reset es el
    motivo de llamarlo al principio de cada celda.

    force_compile y force_upload saltean cada uno su propia verificación.
    """
    global _link

    def say(message):
        if verbose:
            print(message)

    state = _load_state()
    hex_file = BUILD_DIR / f'{SKETCH.name}.ino.hex'
    sources = _sources_hash()
    notes = []

    if force_compile or not hex_file.exists() or state.get('sources') != sources:
        build_flags = []
        for prop in BUILD_PROPERTIES:
            build_flags += ['--build-property', prop]

        _run(['arduino-cli', 'compile', '--fqbn', FQBN,
              '--libraries', str(LIBRARIES),
              '--build-path', str(BUILD_DIR)] +
             build_flags +
             [str(SKETCH)], 'compilar')
        notes.append('compilado')
        state['sources'] = sources
        _save_state(state)

    binary = hashlib.sha256(hex_file.read_bytes()).hexdigest()

    if port is None:
        try:
            port = _wait_for_port()
        except CtrlLinkError:
            raise CtrlLinkError(
                'el sketch esta compilado, pero no hay ninguna placa alcanzable: '
                'no se encontro ningun puerto serie USB. Enchufarla y correr esto '
                'de nuevo; la compilacion esta en cache, asi que va a ir derecho a '
                'la carga.') from None

    uploaded = state.get('uploaded', {})

    if force_upload or uploaded.get(port) != binary:
        # La carga necesita el puerto para sí sola, y resetea la placa igual.
        if _link is not None:
            _link.close()
            _link = None
        perfil = _upload(port, state, say)
        notes.append('cargado' if perfil == UPLOAD_FQBNS[0][0] else
                     f'cargado como {dict(UPLOAD_FQBNS)[perfil]}')
        uploaded[port] = binary
        state['uploaded'] = uploaded
        _save_state(state)
        # algunos puentes se caen del bus mientras se resetean
        port = _wait_for_port(timeout=15.0)

    if _link is not None:
        _link.close()
        _link = None

    try:
        _link = Bench(port)
    except serial.SerialException as exc:
        raise CtrlLinkError(
            f'no se pudo abrir {port}: {exc}\n'
            f'Algo mas lo tiene tomado: el monitor serie del IDE de Arduino, o un '
            f'kernel de una sesion anterior. Cerrarlo, o reiniciar este kernel, y '
            f'volver a correr esta celda. Un puerto serie es exclusivo en todas '
            f'las plataformas, e implacable al respecto en Windows.'
        ) from None

    say(f'{port}: {_link.info}' + (f'  ({", ".join(notes)})' if notes else ''))
    return _link


def sync_board_cal(*args, calibracion=None, **kw):
    """`sync_board()` y, encima, la calibración del sensor de este banco.

    `sync_board()` resetea la placa, y la placa arranca siempre **sin calibrar**
    --a propósito: una tabla vieja aplicándose en silencio es peor que ninguna--,
    así que sin este paso cada celda mediría con el error de ángulo crudo del
    sensor. Cargar la tabla son 64 escrituras de parámetro, del orden de un
    segundo. Si no hay archivo de calibración lo dice y sigue: el banco anda
    igual, sólo que sobre un ángulo torcido.

    `calibracion` es la ruta del archivo; por omisión el de este repositorio.
    """
    dev = sync_board(*args, **kw)

    ruta = Path(calibracion) if calibracion else CALIBRACION
    verbose = kw.get('verbose', True)

    if not ruta.exists():
        if verbose:
            print(f'  sin calibracion del sensor ({ruta.name} no existe): '
                  f'el angulo va crudo')
        return dev

    import calib
    calib.asegurar(dev, ruta)
    if verbose:
        print(f'  calibracion del sensor aplicada desde {ruta.name}')
    return dev
