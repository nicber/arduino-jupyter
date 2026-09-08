"""El banco: las convenciones de compilación, conexión y unidades de este equipo.

Todo lo que hay acá es cañería. Vive fuera del notebook para que una celda del
notebook contenga un controlador y un experimento y nada más.

    from bench import *

    dev = sync_board()
    dev.gains(kp=0.002, ki=0.05)
    dev.ref = dev.deg(45)
    df = dev.step('ref', dev.deg(90))

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

__all__ = ['sync_board', 'Bench', 'CtrlLinkError',
           'MODE_OPEN', 'MODE_PID', 'MODE_RAMP', 'POSITION', 'CURRENT']

FQBN      = 'arduino:avr:uno'
_HERE     = Path(__file__).resolve().parent.parent
SKETCH    = _HERE / 'ControlDemo'
LIBRARIES = _HERE / 'libraries'
BUILD_DIR = _HERE / 'build'

# `mode` elige el controlador, `target` elige la realimentación sobre la que
# cierra.
MODE_OPEN, MODE_PID, MODE_RAMP = 0, 1, 2
POSITION,  CURRENT             = 0, 1

# Bits del registro STATUS del AS5600.
_MAGNET_STRONG, _MAGNET_WEAK, _MAGNET_PRESENT = 0x08, 0x10, 0x20

# La corriente se mide como `SENSE_ZERO - adc`, así que su recorrido es +/-512
# LSB y los extremos son los rieles del ADC. Una lectura ahí arriba no es una
# corriente grande: es una entrada al aire, o un cable suelto.
_ADC_RAILED = 460

_link = None


# -------------------------------------------------------------- el dispositivo

class Bench(CtrlLink):
    """Un CtrlLink que sabe qué significan los números de este equipo.

    Los parámetros de la placa están todos en sus propias unidades: cuentas, LSBs
    del ADC, ganancias por muestra. Éstos los convierten a las unidades en las que
    se diseña un experimento.
    """

    def channel(self, name):
        for column in self.channels:
            if column.name == name:
                return column
        raise CtrlLinkError(f'no hay ningun canal llamado {name!r}')

    # ------------------------------------------------------------ referencias

    def deg(self, degrees):
        """Grados de ángulo del eje -> un `ref` para target = POSITION."""
        return degrees / self.channel('y_uw').scale

    def ma(self, milliamps):
        """Miliamperes -> un `ref` para target = CURRENT."""
        return milliamps / self.channel('i').scale

    def rev_per_s(self, revs):
        """Vueltas por segundo -> un `refrate`, para target = POSITION.

        `refrate` se le suma a `ref` una vez por período de control, así que la
        velocidad que produce una pendiente dada depende de qué tan rápido corre
        el lazo.
        """
        return self.deg(revs * 360.0) * self.dt

    # El otro sentido, para los dos canales cuyas unidades siguen a `target`.
    def as_deg(self, values):
        """`ref` o `e`, capturados con target = POSITION -> grados."""
        return values * self.channel('y_uw').scale

    def as_ma(self, values):
        """`ref` o `e`, capturados con target = CURRENT -> miliamperes."""
        return values * self.channel('i').scale

    # --------------------------------------------------------------- ajuste

    def gains(self, kp=0.0, ki=0.0, kd=0.0):
        """Ganancias del PID en tiempo continuo: ki por segundo, kd en segundos.

        Las ganancias de la placa son por muestra, porque eso es lo que hace su
        aritmética. dt hace la conversión. Para trabajar en los términos de la
        propia placa, fijarlas directamente con `dev.kp`.
        """
        dt = self.dt
        self.kp, self.ki, self.kd = kp, ki * dt, kd / dt

    def smooth(self, which, tau):
        """Fija un filtro por constante de tiempo en segundos; tau = 0 lo apaga.

        `which` es 'y' (posición), 'i' (corriente) o 'e' (el error que ve el
        término derivativo). La placa guarda el polo en sí, alpha, porque es por
        lo que multiplica el filtro.
        """
        dt = self.dt
        self.set(f'alpha_{which}', dt / (tau + dt) if tau > 0 else 1.0)

    def zero(self):
        """Toma la posición actual del eje como cero.

        `y` se lee como `offset - counts` con vuelta, así que bajar `offset` en el
        `y` actual pone `offset` sobre la cuenta actual y `y` en cero.
        """
        self.offset = (self.offset - self.y) % 4096
        self.y_uw = 0

    def rest(self):
        """Lazo abierto, comando en cero. Donde tendría que terminar todo experimento."""
        self.mode = MODE_OPEN
        self.uff = 0

    # ------------------------------------------------------- puesta en marcha

    def bringup(self, motor=True, u=120):
        """Verifica el hardware, un subsistema por vez.

        Cada línea es algo que puede estar mal por su cuenta: el enlace, el lazo,
        el imán, el bus I2C, la medición de corriente, el actuador. Conviene
        correrlo primero, y después de cualquier cambio en el cableado: un
        controlador ajustado contra un sensor que no está leyendo es una tarde
        larga.
        """
        results = []

        def report(label, ok, detail):
            results.append(ok)
            tag = {True: 'ok', False: 'FALLA', None: 'nota'}[ok]
            print(f'  [{tag:>5}]  {label:<18}  {detail}')

        print(f'puesta en marcha: {self.info}')

        self.rest()

        # 1. El lazo de control, medido contra el reloj de esta máquina. Con el
        #    contador de ticks del dispositivo no se puede: avanza una vez por
        #    período *atendido*, así que filas sobre ticks devuelve el período
        #    nominal aunque se pierda la mitad de los períodos, y la verificación
        #    daría siempre por bueno lo que tiene que detectar.
        df = self.capture(1.0, warn=False)
        served = len(df) * df.attrs['dec']
        missed = df.attrs['missed']
        rate   = served / df.attrs['wall']
        want   = 1e6 / df.attrs['dt_us']
        report('lazo de control', missed == 0 and abs(rate - want) < want * 0.05,
               f'{rate:.0f} Hz reales contra {want:.0f} nominales, '
               f'{missed} perdidos, {df.attrs["drops"]} descartados')

        # El margen es un aviso, no un veredicto: mientras no se pierda ningun
        # periodo el lazo esta llegando, y con seis canales a 1 kHz el retardo
        # ronda el 60 % del periodo por el solo costo de emitir la fila. Lo que
        # si es una falla es no tener margen alguno.
        late   = df.attrs['maxlate']
        margin = late / df.attrs['dt_us']
        report('margen de tiempo',
               True if margin < 0.5 else (None if margin < 1.0 else False),
               f'peor retardo de atencion {late} us de {df.attrs["dt_us"]} us '
               f'({margin:.0%})')

        # 2. El sensor, antes que el imán: si el AS5600 no contesta en el bus, lo
        #    que diga su registro del imán no significa nada, y conviene decir
        #    cuál de los dos problemas es.
        present = bool(df.attrs.get('spres', 1))
        report('sensor', present,
               'contesta en el bus' if present else
               'no contesta -- revisar SDA (A4), SCL (A5), alimentacion y pull-ups')

        # 3. El imán, tal como lo ve el propio AS5600. Ésta es la verificación que
        #    detecta un imán montado demasiado lejos del chip, que si no aparece
        #    sólo como un ángulo ruidoso en el que nadie confía.
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

        # 4. El bus que transporta el ángulo, por separado del imán que está en la
        #    otra punta: un problema de pull-ups y uno de montaje se ven igual en
        #    los datos y se arreglan en lugares distintos. Con el sensor ausente
        #    las fallas son las del sondeo espaciado, así que no dicen nada nuevo.
        if present:
            report('bus i2c', df.attrs['serr'] == 0 and df.attrs['sovr'] == 0,
                   f'{df.attrs["serr"]} errores de transferencia, '
                   f'{df.attrs["sovr"]} desbordes')
        else:
            report('bus i2c', None,
                   f'{df.attrs["serr"]} fallas, todas del sondeo al sensor ausente')

        spread = df['y_uw'].max() - df['y_uw'].min()
        report('angulo', None if spread < 0.5 else True,
               f'{df["y_uw"].iloc[-1]:.1f} grados, se movio {spread:.2f} grados '
               f'en el segundo' + ('  (girar el iman para verlo seguir)'
                                   if spread < 0.5 else ''))

        # 5. La medición de corriente en reposo. Un sensor que lee lejos de cero
        #    sin nada accionado es un offset que se va a integrar en toda medición
        #    posterior. Pero antes hay que separar el caso en que no hay nada
        #    conectado: una entrada al aire termina contra un riel del ADC, y eso
        #    da una lectura fuera de escala que no es un offset sino una ausencia.
        rest_ma = df['i'].mean()
        rest_lsb = rest_ma / self.channel('i').scale
        sensed   = abs(rest_lsb) <= _ADC_RAILED
        if not sensed:
            report('medicion de i', None,
                   f'entrada contra el riel del ADC ({rest_ma:+.0f} mA, fuera de '
                   f'escala): no parece haber nada conectado en A0')
        else:
            report('medicion de i', abs(rest_ma) < 50,
                   f'{rest_ma:+.1f} mA en reposo (ruido {df["i"].std():.1f} mA)')

        # 6. El actuador, y con él toda la cadena: un comando que sale, movimiento
        #    y corriente que vuelven. Hay dos evidencias posibles y cada una
        #    depende de su propio sensor, así que sólo se usa la que esté
        #    disponible: dar por bueno un motor porque la corriente se movió,
        #    cuando la entrada de corriente está al aire, es peor que no medir.
        if not motor:
            report('motor', None, 'omitido (motor=False)')
        else:
            print(f'  accionando el motor con u = {u} durante 0,4 s ...')
            self.zero()
            self.uff = u
            spun = self.capture(0.4, warn=False)
            self.rest()

            turned = abs(spun['y_uw'].iloc[-1] - spun['y_uw'].iloc[0]) / 360.0
            drawn  = spun['i'].abs().max()

            evidence = ([f'{turned:.2f} vueltas'] if present else []) + \
                       ([f'{drawn:.0f} mA de pico'] if sensed else [])

            if not evidence:
                report('motor', None, 'no se puede evaluar: no hay sensor de '
                                      'angulo ni medicion de corriente')
            else:
                report('motor',
                       (present and turned > 0.05) or
                       (sensed and drawn > rest_ma + 50),
                       ', '.join(evidence))

        bad = results.count(False)
        print(f'\n{"todas las verificaciones pasaron" if not bad else f"FALLARON {bad} verificacion(es)"}')
        return not bad


# ------------------------------------------------------- compilación y carga

def _sources_hash():
    """Huella digital de todo aquello a partir de lo cual se construye el sketch.

    Por contenido y no por marca de tiempo: un checkout de git reescribe las
    mtime sin cambiar una línea, y si no dispararía una recompilación al pedo.
    """
    digest = hashlib.sha256()
    files = sorted(list(SKETCH.glob('*.ino')) +
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
    el error de verdad. `errors` se fija por la misma razón: un byte mal formado no
    tendría que ser lo que impida informar una falla de compilación.
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


def _wait_for_port(hint=None, timeout=2.0):
    """find_port(), pero tolerante con una placa que todavía se está reenumerando.

    Por omisión la espera es corta porque una placa ausente tiene que informarse
    enseguida; la espera larga sólo vale la pena justo después de una carga,
    cuando el puente puede tardar genuinamente unos segundos en volver.
    """
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
    siempre reabre el enlace, lo que resetea la placa, así que el lazo arranca
    desde los valores por omisión del sketch haya hecho falta o no grabar. Ese
    reset es el motivo de llamarlo al principio de cada celda.

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
        output = _run(['arduino-cli', 'compile', '--fqbn', FQBN,
                       '--libraries', str(LIBRARIES),
                       '--build-path', str(BUILD_DIR),
                       str(SKETCH)], 'compilar')
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
        _run(['arduino-cli', 'upload', '--fqbn', FQBN, '-p', port,
              '--input-dir', str(BUILD_DIR), str(SKETCH)], 'cargar')
        notes.append('cargado')
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
