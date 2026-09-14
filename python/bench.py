"""El banco: compilación, conexión y las verificaciones de este equipo.

Todo lo que hay acá es cañería. Vive fuera del notebook para que una celda del
notebook contenga un experimento y nada más.

    from bench import *

    BIDIR = False                                              # qué actuador tiene este banco
    dev = sync_board(bidir=BIDIR)
    dev.ctl_uff = 100                                          # un comando sobre el actuador
    df = dev.step('ctl_uff', 200, pre=0.3, post=1.2, back=0)   # un escalón, t = 0 en el escalón

`sync_board()` compila si cambió algún archivo fuente, carga si cambió el binario,
y reabre el enlace, lo que resetea la placa. Todas las celdas lo llaman, así que
cada celda arranca desde los valores por omisión del propio sketch y ninguna
depende de que se haya corrido la de arriba.

La placa arranca sin saber nada del cableado, así que al conectar se le dice: si
el actuador acciona en los dos sentidos (`bidir`, que lo sabe quien armó el banco),
los signos del imán y del sensor de corriente (que midió `bringup()` y quedaron
en `CABLEADO`) y el cero de la corriente, que se mide ahí mismo.

Lo que se hace con una captura --derivar la velocidad, pasar a unidades de un
modelo, guardarla-- no está acá: está en `ensayo.py`.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

import serial

import catalogo
import ensayo
from ctrllink import CtrlLink, CtrlLinkError, find_port

__all__ = ['sync_board', 'sync_board_cal', 'Bench', 'CtrlLinkError', 'CALIBRACION',
           'CABLEADO']

FQBN = 'arduino:avr:uno'

# Con qué perfil *cargar*. Compilar es siempre lo mismo --el binario es el mismo
# ATmega328P a 16 MHz en todos los casos--, pero el bootloader que lo recibe no:
# un UNO escucha a 115200 y muchos clones baratos traen el bootloader viejo del
# Nano, que escucha a 57600. Elegir mal no da un error legible sino diez líneas
# de «not in sync». Así que se prueban en orden y se recuerda cuál anduvo, por
# puerto, en `sync-state.json`.
UPLOAD_FQBNS = [
    ('arduino:avr:uno',                    'UNO'),
    ('arduino:avr:nano:cpu=atmega328old',  'clon con bootloader viejo'),
]

_HERE     = Path(__file__).resolve().parent.parent
LIBRARIES = _HERE / 'libraries'
BUILD_DIR = _HERE / 'build'

# El sketch que se graba si no se pide otro. `sync_board(sketch=...)` acepta el
# nombre de una carpeta del repositorio --'Banco', 'ControlDemo'-- o una ruta.
SKETCH    = _HERE / 'Banco'

# La calibración del sensor de este banco. No entra en el repositorio --es un dato
# del banco y no del proyecto-- y la ruta se resuelve desde este archivo, así que
# no depende de desde dónde se corra el notebook. Ver sync_board_cal().
CALIBRACION = _HERE / 'notebooks' / 'calibracion.json'

# Los signos de este banco: de qué lado miran el imán y el sensor de corriente. Los
# mide `bringup()` y los carga `sync_board()`. Tampoco entra en el repositorio, por
# lo mismo que la calibración. `bidir` no está acá: no se mide, lo declara quien
# conecta, y `bringup()` verifica que sea cierto.
CABLEADO = _HERE / 'notebooks' / 'cableado.json'

# El core de AVR compila con `-Os` --optimizar por tamaño--, y este sketch quiere
# ciclos y no bytes: el muestreador de 5 kHz y la telemetría comparten el tiempo
# con el manejador del bus. `-O2` es `-Os` más todo lo que agrande el código, y no
# `-O3`, que en un AVR de 32 kB se paga con casi todo el espacio que queda.
#
# Las banderas van como `extra_flags` y no reemplazando `compiler.*.flags` porque
# el recipe las pega después de las propias del core, y la última `-O` de la línea
# es la que manda: así se hereda todo lo demás en lugar de copiarlo a mano. El
# enlace también lleva `-O2`, porque con `-flto` el grueso de la generación de
# código pasa ahí.
BUILD_PROPERTIES = [
    'compiler.c.extra_flags=-O2',
    'compiler.cpp.extra_flags=-O2',
    'compiler.c.elf.extra_flags=-O2',
]

# Bits del registro STATUS del AS5600.
_MAGNET_STRONG, _MAGNET_WEAK, _MAGNET_PRESENT = 0x08, 0x10, 0x20

# La placa cuenta la corriente en 12 bits, sea cual sea su ADC. Dónde tiene que
# reposar el sensor depende de cuál sea y de cómo esté alimentado, así que acá no
# se juzga el valor: se juzga que quede fuera de los rieles --contra un riel no hay
# una corriente grande sino una entrada al aire-- y que sobre margen para que una
# corriente tenga adónde crecer.
_ADC_FULL     = 4095
_ADC_RAIL     = 80    # a menos de esto de cualquiera de los dos extremos
_ADC_HEADROOM = 400   # cuentas de margen que se le piden al reposo

# Cuánto tiene que girar el eje en un tirón para que signifique algo. Por debajo de
# esto lo que se mide es el ruido del sensor, no un motor que arrancó.
_GIRO_MINIMO = 0.05   # vueltas

# Cuánto residuo se le tolera al cero de la corriente, en cuentas. No una: medido en
# el banco del clon, doce ciclos de medir el cero y volver a mirar dan de -2,1 a
# +1,7 cuentas, una deriva lenta entre captura y captura que no se promedia. Pedir
# menos que la repetibilidad del canal hacía fallar la verificación en un equipo
# sano, que es la peor clase de verificación: la que enseña a ignorarla.
_RESIDUO_MAX = 3.0

_link = None


def leer_cableado(ruta=CABLEADO):
    """Los signos guardados por `bringup()`, como dict, o None si no hay."""
    try:
        return json.loads(Path(ruta).read_text(encoding='utf-8'))
    except (OSError, ValueError, TypeError):
        return None


# ------------------------------------------------------- el diagnóstico del banco

class DiagnosticoDeBanco:
    """Lo que hay que saber de este equipo para creerle una captura.

    Existe porque `ctrllink` no tiene que saber qué hay del otro lado del cable. El
    protocolo habla de períodos perdidos y de filas descartadas; que además haya un
    AS5600 en un bus I2C, con un imán que puede estar torcido, es de este banco. El
    enlace lo recibe como colaborador y le pregunta; ver el comentario de
    `diagnostico` en ctrllink.py.

    Es de sólo leer: trabaja sobre lo que la captura ya trajo en `df.attrs` y no le
    pide nada a la placa.
    """

    # Cuentas acumuladas: una captura las pone en cero antes y lo que vuelve
    # describe esa captura. (clave con la que quedan en df.attrs, parámetro).
    _SALUD = (('sovr', 'ang_busovr'),
              ('serr', 'ang_buserr'))

    # Lecturas de ahora. No se ponen en cero: hacerlo sería inventar una lectura.
    _ESTADO = (('spres', 'ang_present'),
               ('mstat', 'ang_status'),
               ('agc',   'ang_agc'),
               ('mag',   'ang_mag'))

    def parametros_de_salud(self):
        return self._SALUD

    def parametros_de_estado(self):
        return self._ESTADO

    def notas_primero(self, df):
        """Va antes que las del muestreo: si el sensor no está, lo demás es consecuencia."""
        if df.attrs.get('spres') != 0:
            return []

        return ['el AS5600 no contesta en el bus I2C: revisar SDA (A4), '
                'SCL (A5), la alimentacion y los pull-ups. La placa sigue '
                'emitiendo, pero el angulo queda congelado y todo lo que se '
                'mida de posicion no significa nada.']

    def notas_despues(self, df):
        """Lo del sensor que importa menos que un período perdido."""
        notas = []

        sovr = df.attrs.get('sovr') or 0
        if sovr:
            notas.append(
                f'{sovr} desborde(s) del sensor: una transferencia de I2C no habia '
                f'terminado cuando vencia la muestra siguiente, asi que esa muestra '
                f'repite la anterior.')

        # Con el sensor ausente las fallas son las del sondeo espaciado, que ya
        # quedaron explicadas en notas_primero(); contarlas de nuevo sólo agrega
        # ruido. Con el sensor presente, en cambio, son intermitencias.
        serr = df.attrs.get('serr') or 0
        if serr and df.attrs.get('spres') != 0:
            notas.append(f'fallaron {serr} transferencia(s) del sensor -- revisar '
                         f'el cableado y los pull-ups del bus.')

        return notas


# -------------------------------------------------------------- el dispositivo

class Bench:
    """Un banco: un enlace CtrlLink más lo que significan los números de este equipo.

    Tiene un enlace en lugar de ser uno. Lo que este objeto sabe --que el ángulo se
    mide con un AS5600 en un bus I2C, que la corriente pasa por un sensor analógico--
    no tiene por qué poder meterse adentro del protocolo. Todo lo que el enlace sabe
    hacer sigue estando acá, delegado: `capture`, `step`, `set`, `get`, `close`, y
    cada parámetro de la placa como atributo.
    """

    # Los atributos que son de este objeto y no del enlace. Todo lo demás se delega.
    _PROPIOS = frozenset({'link'})

    def __init__(self, port=None, **kw):
        # object.__setattr__ porque __setattr__ consulta el enlace, que todavía no
        # existe.
        object.__setattr__(self, 'link',
                           CtrlLink(port, diagnostico=DiagnosticoDeBanco(), **kw))

    # ----------------------------------------------------------- delegación

    def __getattr__(self, name):
        link = self.__dict__.get('link')
        if link is None:
            raise AttributeError(name)
        return getattr(link, name)

    def __setattr__(self, name, value):
        link = self.__dict__.get('link')

        if link is not None and name not in self._PROPIOS and name in link._params:
            link.set(name, value)
        else:
            object.__setattr__(self, name, value)

    def __dir__(self):
        return sorted(set(super().__dir__()) | set(dir(self.link)))

    # Los dunder del protocolo de contexto se buscan en el tipo y no pasan por
    # __getattr__, así que hay que escribirlos.
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def channel(self, name):
        for column in self.channels:
            if column.name == name:
                return column
        raise CtrlLinkError(f'no hay ningun canal llamado {name!r}')

    # ------------------------------------------------------ contarse solo
    #
    # La placa declara su tabla de parámetros al conectarse, así que la lista de
    # perillas no está escrita en ninguna parte de este lado: se pregunta. En un
    # notebook alcanza con poner `dev` en una celda.

    def describe(self, filtro=''):
        """Las perillas y las lecturas de la placa, agrupadas, como texto.

        `filtro` es una subcadena: `dev.describe('ang')` muestra sólo lo del sensor
        de ángulo.
        """
        return catalogo.texto(*self._para_describir(filtro))

    def _para_describir(self, filtro=''):
        nombres = [n for n in sorted(self._params) if filtro in n]
        canales = ([(c.name, c.scale, c.unit) for c in self.channels]
                   if not filtro else ())
        resumen = (f'filas a {1 / self.dt:.0f} Hz, '
                   f'{len(self.channels)} canales de telemetría')
        return self.info, resumen, nombres, self._valor_legible, canales

    def _valor_legible(self, nombre):
        """El valor de ahora, o un signo de pregunta si la placa no lo contesta."""
        try:
            valor = self.get(nombre)
        except Exception:
            return '?'

        return f'{valor:.6g}' if isinstance(valor, float) else str(valor)

    def __repr__(self):
        try:
            return self.describe()
        except Exception as exc:
            return f'<Bench sin describir: {type(exc).__name__}: {exc}>'

    def _repr_html_(self):
        return catalogo.html(*self._para_describir())

    # ------------------------------------------------------------- reposo

    def rest(self):
        """Comando en cero. Donde tendría que terminar todo experimento.

        Con el comando en cero el sketch deja ENA en bajo, así que el actuador queda
        abierto: no frena el eje, sólo deja de empujarlo.
        """
        self.ctl_uff = 0

    def zero_current(self, seconds=0.3, canales=None):
        """Toma la corriente que se mida ahora como el cero. Devuelve `cur_zero`.

        Con el actuador abierto no circula corriente, así que lo que marque el
        sensor es su offset: el suyo propio, más la tolerancia de su alimentación y
        la de cualquier divisor que haya en el medio. Deja el motor en reposo, que
        es la condición bajo la cual la medición significa algo.

        `canales` tiene que ser el mismo que va a usar el ensayo. El cero depende
        de cuánta telemetría sale: transmitir le carga la alimentación a la placa
        --el LED de TX, el conversor USB-serie-- y medido en el banco la corriente
        leída se corre unos -5,6 mA por cada 1000 bytes/s, casi 60 mA entre emitir
        todos los canales y emitir uno solo. Por omisión, todos, que es lo que emite
        `capture()` si no se le pide otra cosa.
        """
        self.cur_zero = round(self._reposo_de_corriente(seconds, canales))
        return self.cur_zero

    def _reposo_de_corriente(self, seconds=0.3, canales=None):
        """La cuenta del ADC con el actuador abierto, promediada.

        `i` se publica como `s * (adc - cur_zero)`, con `s` el signo de `cur_inv`,
        así que la cuenta cruda se reconstruye deshaciendo las dos cosas. No
        importa si el eje sigue girando por inercia: con el actuador abierto la
        fuerza contraelectromotriz no encuentra camino y no circula corriente.
        """
        self.rest()
        df = self.capture(seconds, warn=False, canales=canales)
        return self.cur_zero + self._signo_corriente() * df['i'].mean() / self.channel('i').scale

    def _signo_corriente(self):
        """-1 si la placa publica la corriente dada vuelta. Banco lo llama `cur_inv`,
        ControlDemo `cur_invert`, y un sketch sin ninguno de los dos no la da vuelta."""
        for nombre in ('cur_inv', 'cur_invert'):
            if nombre in self.link._params:
                return -1 if self.get(nombre) else 1
        return 1

    def _tiron(self, u, seconds=0.4):
        """`u` sobre el actuador por un instante, desde el eje quieto.

        Devuelve (vueltas, captura). Espera primero a que el eje pare: con el
        actuador abierto el motor no frena, y midiendo enseguida lo que se mide es
        el giro anterior. Las vueltas van con signo.
        """
        ensayo.esperar_quieto(self, limite=15.0)
        self.ctl_uff = u
        df = self.capture(seconds, warn=False)
        self.rest()

        return (df['y_uw'].iloc[-1] - df['y_uw'].iloc[0]) / 360.0, df

    # ------------------------------------------------------- el cableado

    def configurar(self, bidir=None, cableado=CABLEADO, cero=True, say=print):
        """Le dice a la placa lo que no puede saber sola. Lo llama `sync_board()`.

        `bidir` es qué actuador tiene el banco, y lo declara quien lo conecta: un
        puente en H acciona en los dos sentidos, un transistor en uno. Sin
        declararlo la placa queda en un solo cuadrante, que nunca invierte un motor
        que no lo esperaba. Los signos salen de `cableado`, que escribe
        `bringup()`. `cero` mide el cero de la corriente, que cada reset pierde.
        """
        params = self.link._params

        if 'mot_bidir' in params:
            if bidir is None:
                say('  OJO: no se declaro bidir, asi que el actuador queda en un solo '
                    'cuadrante y un comando negativo sale como cero. Conectar con '
                    'sync_board(bidir=True) o bidir=False.')
            else:
                self.mot_bidir = int(bool(bidir))

        guardado = leer_cableado(cableado) or {}

        if 'cur_sagd' in params:
            if 'cur_sagd' in guardado:
                self.cur_sagc = int(guardado['cur_sagc'])
                self.cur_sagd = int(guardado['cur_sagd'])
                say(f'  caida de AVCC compensada: {self.cur_sagc / 100:.2f} % mientras '
                    f'conmuta + {self.cur_sagd / 100:.2f} % a fondo')
            else:
                say('  sin compensar la caida de AVCC con el PWM: la corriente lee de mas '
                    'mientras el transistor conduce. Con la fuente del motor APAGADA, '
                    'correr dev.calibrar_caida().')

        if 'cur_red' in params and 'cur_red' in guardado:
            self.cur_red = int(guardado['cur_red'])
            say(f'  la red vista desde la placa: {self.cur_red / 100:.2f} Hz (para el notch, '
                f'cur_notch)')

        if 'ang_inv' in params:
            if 'ang_inv' not in guardado:
                say(f'  sin signos medidos ({Path(cableado).name} no los tiene): el '
                    f'angulo y la corriente van con el signo del cableado. Correr '
                    f'dev.bringup().')
            else:
                self.ang_inv = int(guardado['ang_inv'])
                self.cur_inv = int(guardado['cur_inv'])
                say(f'  signos del banco: ang_inv = {self.ang_inv}, cur_inv = '
                    f'{self.cur_inv} ({Path(cableado).name})')

        # Contra un riel no hay un offset que medir sino una entrada al aire, y
        # "calibrarla" dejaría un canal que informa ceros perfectos. Eso lo explica
        # bringup(); acá sólo no se toca.
        if cero and 'cur_zero' in params:
            adc = self._reposo_de_corriente()
            if _ADC_RAIL <= adc <= _ADC_FULL - _ADC_RAIL:
                self.cur_zero = round(adc)

    def calibrar_red(self, segundos=20.0, armonicos=3, guardar=True):
        """Mide la frecuencia de la red tal como la ve la placa y se la pone al notch.

        Con el motor quieto y sin comando: lo que se mida es la corriente en reposo,
        que es donde los tonos de la red se ven solos. Se captura sin notch y sin
        promediar filas, y se busca la frecuencia que junta más energía en la
        fundamental y sus `armonicos` a la vez, sobre el tiempo de la propia placa:
        la red medida con el cristal del clon no da 50 Hz, y eso es lo que el notch
        necesita. Devuelve (Hz, amplitudes en mA de cada armónico).
        """
        import numpy as np

        ensayo.esperar_quieto(self, limite=40.0)
        antes = (self.cur_filas, self.cur_notch)
        self.cur_filas, self.cur_notch = 1, 0
        try:
            time.sleep(0.2)
            df = self.capture(segundos, warn=False, canales=['i'])
        finally:
            self.cur_filas, self.cur_notch = antes

        t = df['t'].to_numpy(dtype=float)
        x = df['i'].to_numpy(dtype=float)
        x = x - x.mean()

        def energia(f):
            return sum(abs(np.sum(x * np.exp(-2j * np.pi * k * f * t))) ** 2
                       for k in range(1, armonicos + 1))

        # Grueso de a 0,01 Hz alrededor de 50, fino de a 0,001 alrededor del máximo.
        grilla = np.arange(49.0, 51.0, 0.01)
        f = grilla[int(np.argmax([energia(g) for g in grilla]))]
        fina = np.arange(f - 0.01, f + 0.01, 0.001)
        f = float(fina[int(np.argmax([energia(g) for g in fina]))])

        amplitudes = [2 * abs(np.sum(x * np.exp(-2j * np.pi * k * f * t))) / len(x)
                      for k in range(1, armonicos + 1)]
        print(f'red vista desde la placa: {f:.3f} Hz; tonos de '
              + ', '.join(f'{a:.1f} mA' for a in amplitudes)
              + f' en {", ".join(f"{k * f:.1f}" for k in range(1, armonicos + 1))} Hz')

        self.cur_red = int(round(f * 100))
        if guardar:
            _actualizar_cableado(cur_red=self.cur_red)
        return f, amplitudes

    def calibrar_caida(self, us=(40, 80, 120, 160, 200, 230, 255), seconds=1.0, guardar=True):
        """Mide cuánto sube la lectura de corriente con el PWM sin corriente en el motor.

        **Con la fuente del motor apagada**, o el motor desconectado: si circula
        corriente, se la calibra como si fuera error.

        Mientras el transistor conduce, su corriente de base carga la alimentación
        del micro, y como el ADC mide contra ella la lectura sube. Sin corriente en
        el motor todo lo que se lea es eso. Se ajusta la recta
        `e(D) = sagc · [0 < D < 1] + sagd · D` en partes por diez mil, se la pone en
        la placa y se la guarda en `CABLEADO`. Devuelve (sagc, sagd).
        """
        import numpy as np

        lsb = self.channel('i').scale
        self.cur_sagc = self.cur_sagd = 0
        self.rest()
        time.sleep(0.3)

        def cuentas():
            df = self.capture(seconds, warn=False, canales=['i'])
            return self.cur_zero + self._signo_corriente() * df['i'].mean() / lsb

        base = cuentas()
        D, e = [], []
        try:
            for u in us:
                self.ctl_uff = u
                time.sleep(0.3)
                D.append(u / 255.0)
                e.append((cuentas() - base) / base * 10000)
        finally:
            self.rest()

        D, e = np.array(D), np.array(e)
        conmuta = D < 1.0
        A = np.column_stack([conmuta.astype(float), D])
        sagc, sagd = np.linalg.lstsq(A, e, rcond=None)[0]
        resto = e - A @ [sagc, sagd]
        sagc, sagd = int(round(max(sagc, 0))), int(round(max(sagd, 0)))

        print(f'caida de AVCC: {sagc / 100:.2f} % mientras conmuta + {sagd / 100:.2f} % a '
              f'fondo (residuo del ajuste {np.abs(resto).max():.0f} por diez mil)')
        for d, x in zip(D, e):
            print(f'  {d:4.0%}: {x / 100:+.2f} %')

        self.cur_sagc, self.cur_sagd = sagc, sagd
        if guardar:
            _actualizar_cableado(cur_sagc=sagc, cur_sagd=sagd)
        return sagc, sagd

    # ------------------------------------------------------- puesta en marcha

    def bringup(self, motor=True, u=120):
        """Verifica el hardware, un subsistema por vez. Devuelve si pasó todo.

        Cada línea es algo que puede estar mal por su cuenta: el ritmo de las filas,
        el sensor, el imán, el bus I2C, el cero de la medición de corriente --que de
        paso se calibra-- y el actuador. Conviene correrlo primero, y después de
        cualquier cambio en el cableado.

        Con el motor, además, mide los signos del banco --de qué lado miran el imán
        y el sensor de corriente--, se los pone a la placa y los guarda en
        `CABLEADO`, de donde los carga `sync_board()`. Y verifica que el actuador
        haga lo que dice `mot_bidir`: con un puente, un comando negativo invierte
        el giro; con uno solo cuadrante, sale como cero.
        """
        results = []

        def report(label, ok, detail):
            results.append(ok)
            tag = {True: 'ok', False: 'FALLA', None: 'nota'}[ok]
            print(f'  [{tag:>5}]  {label:<18}  {detail}')

        print(f'puesta en marcha: {self.info}')

        self.rest()

        # 1. El ritmo de las filas, medido contra el reloj de esta máquina. Con el
        #    contador de ticks del dispositivo no se puede: avanza una vez por
        #    período *atendido*, así que filas sobre ticks devuelve el período
        #    nominal aunque se pierda la mitad. Un reloj de placa cuatro veces lento
        #    --un clon que arrancó dividido-- aparece acá como 125 Hz.
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
               f'peor retardo de atencion {late} us de {df.attrs["dt_us"]:.0f} us '
               f'({margin:.0%})')

        # 2. El sensor, antes que el imán: si el AS5600 no contesta en el bus, lo
        #    que diga su registro del imán no significa nada.
        present = bool(df.attrs.get('spres', 1))
        report('sensor', present,
               'contesta en el bus' if present else
               'no contesta -- revisar SDA (A4), SCL (A5), alimentacion y pull-ups')

        # 3. El imán, tal como lo ve el propio AS5600. El AGC en números, porque es
        #    la compuerta de la calibración: contra un borde el imán está a la
        #    distancia equivocada y ninguna tabla arregla nada.
        status = df.attrs.get('mstat', 0)
        agc = df.attrs.get('agc')
        if not present:
            report('iman', None, 'no se puede evaluar sin el sensor')
        elif not status & _MAGNET_PRESENT:
            report('iman', False, 'no se detecta -- esta montado sobre el chip?')
        elif status & _MAGNET_WEAK:
            report('iman', False, 'muy debil (AGC al maximo) -- acercarlo')
        elif status & _MAGNET_STRONG:
            report('iman', False, 'muy fuerte (AGC al minimo) -- alejarlo')
        elif agc is None:
            report('iman', True, 'detectado, AGC en rango')
        else:
            comodo = 32 <= agc <= 224
            report('iman', True if comodo else None,
                   f'detectado, AGC {agc:.0f}/255, campo {df.attrs.get("mag", 0):.0f}'
                   + ('' if comodo else '  (cerca del borde: centrar la distancia)'))

        # 4. El bus. Un desborde no es un error: es una muestra que el bus no llegó
        #    a entregar antes del tick siguiente, y un puñado por segundo es normal
        #    --leer el estado del imán no entra en 200 us--. Lo que es una falla es
        #    que el bus no llegue de manera sostenida.
        if present:
            muestras = df.attrs['wall'] * 1e6 / df.attrs['dt_us'] * self.loop_div
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
        #    ausencia, no un offset, y calibrarla dejaría un canal que informa ceros
        #    perfectos sin haber medido nada. Y un reposo pegado a un extremo no
        #    deja lugar para medir, aunque no llegue al riel.
        lsb = self.channel('i').scale
        adc = self.cur_zero + df['i'].mean() / lsb
        sensed = _ADC_RAIL <= adc <= (_ADC_FULL - _ADC_RAIL)
        rest_ma = noise = 0.0

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

            # Con el eje quieto, aunque con el actuador abierto no debería importar:
            # si importa, es algo del montaje que conviene ver en este residuo.
            ensayo.esperar_quieto(self, limite=15.0)
            self.zero_current()
            zeroed  = self.capture(0.3, warn=False)
            rest_ma = zeroed['i'].mean()
            noise   = zeroed['i'].std()

            # Un canal demasiado quieto es tan sospechoso como uno ruidoso: un ruido
            # de cero exacto es una señal más chica que un escalón del ADC.
            report('calibracion de i',
                   abs(rest_ma) < _RESIDUO_MAX * lsb and noise > 0.1 * lsb,
                   f'cur_zero = {self.cur_zero}, {lsb:.1f} mA por cuenta, quedan '
                   f'{rest_ma:+.1f} mA en reposo, ruido {noise:.0f} mA RMS'
                   + ('' if noise > 0.1 * lsb else
                      '  -- sin dither: la senal no llega a un escalon del ADC'))

        # 6. El actuador, y con él toda la cadena: un comando que sale, movimiento y
        #    corriente que vuelven. Sólo se usa la evidencia cuyo sensor está.
        if not motor:
            report('motor', None, 'omitido (motor=False)')
        else:
            self._verificar_actuador(report, u, present, sensed, rest_ma, noise)

        bad = results.count(False)
        print(f'\n{"todas las verificaciones pasaron" if not bad else f"FALLARON {bad} verificacion(es)"}')
        return not bad

    def _verificar_actuador(self, report, u, present, sensed, rest_ma, noise):
        """La parte de bringup() que mueve el motor: el actuador, `bidir` y los signos.

        Primero en crudo --sin los signos del banco y con los dos sentidos
        habilitados--, un tirón con +u y otro con -u, que es lo único que deja ver
        qué hay: un puente invierte el giro, un transistor empuja igual. Con eso se
        verifica lo que se declaró en `bidir` y se miden los signos. Después se
        aplican y se repiten los dos tirones, que ahora tienen que dar lo que un
        usuario espera: +u sube el ángulo y la corriente, y -u baja el ángulo o no
        hace nada, según el actuador.
        """
        bidir = bool(self.mot_bidir)
        antes = (self.ang_inv, self.cur_inv)
        lsb = self.channel('i').scale

        print(f'  accionando con u = +{u} y u = -{u}, 0,4 s cada uno y con el eje '
              f'quieto antes, dos veces ...')

        self.ang_inv = self.cur_inv = 0
        self.mot_bidir = 1
        try:
            v_pos, df_pos = self._tiron(u)
            v_neg, _ = self._tiron(-u)
        except BaseException:
            self.ang_inv, self.cur_inv = antes
            raise
        finally:
            self.mot_bidir = int(bidir)

        pico = df_pos['i'].abs().max()
        i_pos = df_pos['i'].mean()

        evidence = ([f'{abs(v_pos):.2f} vueltas'] if present else []) + \
                   ([f'{pico:.0f} mA de pico'] if sensed else [])
        gira = present and abs(v_pos) > _GIRO_MINIMO

        if not evidence:
            report('motor', None, 'no se puede evaluar: no hay sensor de angulo ni '
                                  'medicion de corriente')
        else:
            report('motor', gira or (sensed and pico > abs(rest_ma) + 5 * noise),
                   ', '.join(evidence))

        # Que el motor gire no quiere decir que el canal de corriente lo vea. Un
        # motor chico consume decenas de mA, y contra el ruido de un sensor de
        # varios amperes eso puede no ser nada: mejor saberlo antes de sacar
        # conclusiones de `i`.
        if sensed and gira and pico < 5 * noise:
            report('canal de i', None,
                   f'el pico del arranque ({pico:.0f} mA) no se despega de cinco '
                   f'veces el ruido ({5 * noise:.0f} mA): este canal no resuelve la '
                   f'corriente de este motor')

        # ---- el actuador contra lo declarado
        if not gira:
            report('actuador', None, 'el eje no giro con +u: no se puede verificar bidir')
        elif abs(v_neg) <= _GIRO_MINIMO:
            report('actuador', False,
                   f'con -{u} el eje no gira ({v_neg:+.2f} vueltas): '
                   + ('revisar IN1 (6) e IN2 (7)' if bidir else
                      'un transistor tendria que empujar igual que con +u'))
        else:
            invierte = (v_pos > 0) != (v_neg > 0)
            tipo = ('invierte el giro: puente en H (B)' if invierte else
                    'empuja para el mismo lado: un solo cuadrante (B′)')
            if invierte == bidir:
                report('actuador', True, f'-u {tipo}, como declara bidir = {bidir}')
            else:
                report('actuador', False,
                       f'-u {tipo}, pero se conecto con bidir = {bidir}. '
                       f'Conectar con sync_board(bidir={invierte}).')

        # ---- los signos
        if not gira:
            self.ang_inv, self.cur_inv = antes
            report('signos', None, 'sin giro no se pueden medir: quedan los de antes')
            return

        ang_inv = int(v_pos < 0)

        # La media de todo el tirón y no el pico, que con el sensor al revés también
        # es grande. Tiene que despegarse de lo que puede dar el residuo del cero.
        umbral = max(3 * lsb, 5 * noise / max(len(df_pos), 1) ** 0.5)
        mide_i = sensed and abs(i_pos) > umbral
        cur_inv = int(i_pos < 0) if mide_i else antes[1]

        self.ang_inv, self.cur_inv = ang_inv, cur_inv
        self._guardar_cableado(ang_inv, cur_inv, mide_i, invierte=(v_pos > 0) != (v_neg > 0)
                               if abs(v_neg) > _GIRO_MINIMO else None)

        v_pos, df_pos = self._tiron(u)
        v_neg, df_neg = self._tiron(-u)
        i_pos = df_pos['i'].mean()

        ok = v_pos > _GIRO_MINIMO and (not mide_i or i_pos > 0)
        report('signos', ok,
               f'ang_inv = {ang_inv}, cur_inv = {cur_inv}'
               + ('' if mide_i else ' (sin medir: la corriente no se despega del cero)')
               + f'; +u da {v_pos:+.2f} vueltas y {i_pos:+.0f} mA de media. '
               f'Guardados en {CABLEADO.name}')

        u_neg = df_neg['u'].min()
        if bidir:
            report('-u', v_neg < -_GIRO_MINIMO and u_neg == -u,
                   f'sale u = {u_neg:+.0f} y el eje da {v_neg:+.2f} vueltas')
        else:
            report('-u', abs(v_neg) <= _GIRO_MINIMO and u_neg == 0,
                   f'sale u = {u_neg:+.0f} (recortado a cero) y el eje da '
                   f'{v_neg:+.2f} vueltas')

    def _guardar_cableado(self, ang_inv, cur_inv, cur_medido, invierte):
        """Guarda los signos en `CABLEADO`, sin tocar lo demás que haya."""
        _actualizar_cableado(
            ang_inv=int(ang_inv),
            cur_inv=int(cur_inv),
            cur_inv_medido=bool(cur_medido),
            invierte_con_u_negativo=None if invierte is None else bool(invierte),
            placa=self.info)


def _actualizar_cableado(**campos):
    """Agrega o reemplaza `campos` en `CABLEADO`, con la fecha."""
    from datetime import datetime

    datos = leer_cableado() or {}
    datos.update(campos)
    datos['medido'] = datetime.now().isoformat(timespec='seconds')
    CABLEADO.write_text(json.dumps(datos, indent=1, ensure_ascii=False), encoding='utf-8')


# ------------------------------------------------------- compilación y carga

def _sketch_dir(sketch=None):
    """La carpeta del sketch: `SKETCH` si no se pide otro, una carpeta del
    repositorio si se da un nombre, o la ruta tal cual si se da una ruta.
    """
    if sketch is None:
        ruta = SKETCH
    elif isinstance(sketch, Path) or '/' in sketch or '\\' in sketch:
        ruta = Path(sketch).resolve()
    else:
        ruta = _HERE / sketch

    if not (ruta / f'{ruta.name}.ino').exists():
        raise ValueError(f'no hay ningun sketch en {ruta}: falta {ruta.name}.ino')
    return ruta


def _sources_hash(sketch):
    """Huella digital de todo aquello a partir de lo cual se construye el sketch.

    Por contenido y no por marca de tiempo: un checkout de git reescribe las
    mtime sin cambiar una línea, y si no dispararía una recompilación al pedo.
    Las banderas de compilación entran en la huella junto con las fuentes: un
    `build/` que quedó de una corrida con otra optimización tiene las mismas
    fuentes y un binario que ya no es el que corresponde.
    """
    digest = hashlib.sha256()
    digest.update(repr(BUILD_PROPERTIES).encode())
    files = sorted(list(sketch.glob('*.ino')) + list(sketch.glob('*.h')) +
                   [p for p in LIBRARIES.rglob('*') if p.suffix in ('.h', '.cpp', '.c')])
    for path in files:
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _arduino_cli():
    """La ruta del Arduino CLI: el del PATH si hay uno, y si no el que trae el IDE.

    El IDE 2 compila con un arduino-cli propio, guardado adentro de su instalación
    y fuera del PATH. Buscarlo ahí es lo que permite que alcance con instalar el
    IDE, sin tocar variables de entorno. Los dos comparten los cores instalados,
    así que el core de AVR que baja el IDE sirve igual.
    """
    found = shutil.which('arduino-cli')
    if found:
        return found

    backend = Path('resources', 'app', 'lib', 'backend', 'resources')
    candidates = [Path('/Applications/Arduino IDE.app/Contents/Resources/app/lib/'
                       'backend/resources/arduino-cli')]
    for var, sub in (('LOCALAPPDATA', Path('Programs', 'arduino-ide')),
                     ('ProgramFiles', Path('Arduino IDE'))):
        if os.environ.get(var):
            candidates.append(Path(os.environ[var]) / sub / backend / 'arduino-cli.exe')

    for path in candidates:
        if path.is_file():
            return str(path)
    return 'arduino-cli'        # que _run() explique que no está


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
            f'no se encontro arduino-cli, asi que no se puede {what} el sketch.\n'
            f'Instalar el Arduino IDE (https://www.arduino.cc/en/software) y abrirlo '
            f'una vez. Si esta instalado en un lugar poco comun, instalar tambien el '
            f'Arduino CLI y reiniciar el editor, porque el PATH se lee una sola vez '
            f'al arrancar.'
        ) from None

    if done.returncode:
        output = (done.stdout + done.stderr).strip()
        if 'platform not installed' in output:
            output += ('\n\nFalta el soporte para placas AVR. Abrir el Arduino IDE, '
                       'ir a Herramientas > Placa > Gestor de placas, buscar '
                       '"Arduino AVR Boards" e instalarlo.')
        raise RuntimeError(f'fallo al {what}:\n{output}')
    return done.stdout + done.stderr


def _load_state():
    try:
        return json.loads((BUILD_DIR / 'sync-state.json').read_text())
    except (OSError, ValueError):
        return {}


def _save_state(state):
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    (BUILD_DIR / 'sync-state.json').write_text(json.dumps(state, indent=1))


def _upload(port, state, say, sketch, build):
    """Carga el binario, averiguando sola con qué bootloader habla esta placa.

    Devuelve el FQBN que anduvo, y lo deja anotado en el estado para la próxima
    vez. Si la anotación quedó vieja --se cambió la placa de puerto, o el puerto
    de placa-- el intento falla y se sigue con los otros perfiles, así que la
    memoria acelera pero no decide.
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
            _run([_arduino_cli(), 'upload', '--fqbn', fqbn, '-p', port,
                  '--input-dir', str(build), str(sketch)], 'cargar')
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
    """find_port(), pero tolerante con una placa que todavía se está reenumerando.

    Por omisión la espera es corta porque una placa ausente tiene que informarse
    enseguida; la espera larga sólo vale la pena justo después de una carga.
    """
    deadline = time.monotonic() + timeout
    while True:
        try:
            return find_port(hint)
        except CtrlLinkError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.3)


def sync_board(port=None, force_compile=False, force_upload=False, verbose=True,
               sketch=None, bidir=None):
    """Pone al día la placa y el enlace, y reconecta. Devuelve un Bench.

    Compila sólo cuando algún archivo fuente cambió de verdad, carga sólo cuando
    el binario resultante difiere del que este puerto recibió por última vez, y
    siempre reabre el enlace, lo que resetea la placa, así que el sketch arranca
    desde sus valores por omisión haya hecho falta o no grabar. Ese reset es el
    motivo de llamarlo al principio de cada celda.

    `bidir` es qué actuador tiene el banco: True para un puente en H, False para un
    transistor. Junto con los signos medidos y el cero de la corriente se le carga
    a la placa después del reset; ver `Bench.configurar()`.

    force_compile y force_upload saltean cada uno su propia verificación.

    `sketch` elige qué se graba: el nombre de una carpeta del repositorio
    --'ControlDemo'-- o una ruta. Por omisión `SKETCH`. Cada sketch compila en su
    propia carpeta de `build/`, así que alternar entre dos no recompila ninguno.
    """
    global _link

    def say(message):
        if verbose:
            print(message)

    sketch = _sketch_dir(sketch)
    build = BUILD_DIR / sketch.name

    state = _load_state()
    hex_file = build / f'{sketch.name}.ino.hex'
    sources = _sources_hash(sketch)
    notes = []

    # Antes había una sola huella para un solo sketch; un estado de entonces se
    # descarta y cuesta una compilación.
    if not isinstance(state.get('sources'), dict):
        state['sources'] = {}

    if (force_compile or not hex_file.exists()
            or state['sources'].get(sketch.name) != sources):
        build_flags = []
        for prop in BUILD_PROPERTIES:
            build_flags += ['--build-property', prop]

        _run([_arduino_cli(), 'compile', '--fqbn', FQBN,
              '--libraries', str(LIBRARIES),
              '--build-path', str(build)] +
             build_flags +
             [str(sketch)], 'compilar')
        notes.append('compilado')
        state['sources'][sketch.name] = sources
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
        perfil = _upload(port, state, say, sketch, build)
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
    _link.configurar(bidir=bidir, say=say)
    return _link


def sync_board_cal(*args, calibracion=None, **kw):
    """`sync_board()` y, encima, la calibración del sensor de este banco.

    `sync_board()` resetea la placa, y la placa arranca siempre **sin calibrar**
    --a propósito: una tabla vieja aplicándose en silencio es peor que ninguna--,
    así que sin este paso cada celda mediría con el error de ángulo crudo del
    sensor. Cargar la tabla son 64 escrituras de parámetro, del orden de un
    segundo. Si no hay archivo de calibración lo dice y sigue: el banco anda
    igual, sólo que sobre un ángulo torcido.

    `calibracion` es la ruta del archivo; por omisión el de este repositorio, que
    se resuelve desde acá y no desde el directorio de trabajo.
    """
    dev = sync_board(*args, **kw)

    ruta = Path(calibracion) if calibracion else CALIBRACION
    verbose = kw.get('verbose', True)

    if not ruta.exists():
        if verbose:
            print(f'  sin calibracion ({ruta.name} no existe): el angulo va crudo. '
                  f'Correr notebooks/calibracion.ipynb para medirla.')
        return dev

    # El import va acá adentro y no arriba: calib trae el ajuste por mínimos
    # cuadrados, y nada de eso hace falta para hablar con la placa.
    import calib

    cal = calib.asegurar(dev, ruta)

    if verbose:
        pico = max(abs(v) for v in cal.lut) / calib.OCTAVOS
        print(f'  calibracion "{cal.banco}" del {cal.creada[:10]}: '
              f'{len(cal.armonicos)} armonicos, corrige hasta '
              f'{pico * calib.GRADOS_POR_CUENTA:.1f} grados')

    return dev
