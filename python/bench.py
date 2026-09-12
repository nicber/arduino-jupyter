"""El banco: las convenciones de compilación, conexión y unidades de este equipo.

Todo lo que hay acá es cañería. Vive fuera del notebook para que una celda del
notebook contenga un controlador y un experimento y nada más.

    from bench import *

    dev = sync_board()
    dev.gains(kp=0.002, ki=0.05)
    dev.ref = dev.deg(45)
    df = dev.step('ctl_ref', dev.deg(90))

`sync_board()` compila si cambió algún archivo fuente, carga si cambió el binario,
y reabre el enlace, lo que resetea la placa. Todas las celdas lo llaman, así que
cada celda arranca desde los valores por omisión del propio sketch y ninguna
depende de que se haya corrido la de arriba.

Lo único que no puede salir del sketch es lo que describe a este banco y no al
programa: la calibración del sensor y los tres números del cableado --si el puente
invierte, y los dos signos que hacen que un comando positivo dé velocidad positiva
y corriente positiva. Los mide `dev.bringup()` una vez, los deja anotados, y
`sync_board()` los vuelve a poner después de cada reset. Ver `Cableado`.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import NamedTuple

import serial

import catalogo
from ctrllink import CtrlLink, CtrlLinkError, find_port

__all__ = ['sync_board', 'sync_board_cal', 'Bench', 'Cableado', 'Giro',
           'CtrlLinkError', 'CALIBRACION', 'CABLEADO',
           'MODE_OPEN', 'MODE_PID', 'MODE_RAMP', 'POSITION', 'CURRENT']

FQBN      = 'arduino:avr:uno'

# Con qué perfil *cargar*. Compilar es siempre lo mismo --el binario es el mismo
# ATmega328P a 16 MHz en todos los casos--, pero el bootloader que lo recibe no:
# un UNO escucha a 115200 y muchos clones baratos traen el bootloader viejo del
# Nano, que escucha a 57600. Elegir mal no da un error legible sino diez líneas
# de «not in sync», que es de las cosas que más tiempo le hacen perder a alguien
# que recién empieza.
#
# Así que se prueban en orden y se recuerda cuál anduvo, por puerto, en
# `sync-state.json`. El primer intento cuesta unos segundos una sola vez; a
# partir de ahí la placa conocida va derecho al perfil que ya le funcionó.
UPLOAD_FQBNS = [
    ('arduino:avr:uno',                    'UNO'),
    ('arduino:avr:nano:cpu=atmega328old',  'clon con bootloader viejo'),
]

_HERE     = Path(__file__).resolve().parent.parent
SKETCH    = _HERE / 'ControlDemo'
LIBRARIES = _HERE / 'libraries'
BUILD_DIR = _HERE / 'build'

# La calibración del sensor de este banco. No entra en el repositorio --es un dato
# del banco y no del proyecto-- y la ruta se resuelve desde este archivo, así que
# no depende de desde dónde se corra el notebook. Ver sync_board_cal().
CALIBRACION = _HERE / 'notebooks' / 'calibracion.json'

# Y el cableado de este banco, por la misma razón y con las mismas reglas. Lo
# escribe bringup() y lo aplica sync_board(). Ver Cableado.
CABLEADO = _HERE / 'notebooks' / 'cableado.json'

# El core de AVR compila con `-Os` --optimizar por tamaño--, que es lo razonable
# para un sketch cualquiera y no es lo que quiere éste: acá el paso de control
# corre dentro de un período de 2 ms compartido con el muestreador de 5 kHz, y
# los ciclos valen más que los bytes. `-O2` es optimización plena: `-Os` es
# justamente `-O2` menos todo lo que agrande el código --el inlining amplio, la
# alineación, el reordenamiento de bloques--, y eso es lo que se recupera acá.
#
# Las banderas van como `extra_flags` y no reemplazando `compiler.*.flags`
# porque el recipe las pega después de las propias del core, y la última `-O`
# de la línea es la que manda. Así se hereda todo lo demás --`-flto`, las
# banderas de aviso, el estándar del lenguaje-- en lugar de copiarlo a mano y
# que se pudra en la próxima versión del core.
#
# `-O2` y no `-O3`: medido sobre este sketch, `-Os` da 17018 bytes de flash,
# `-O2` da 18986 (58 % del UNO) y `-O3` da 28676 (88 %). Lo que `-O3` agrega es
# inlining y desenrollado agresivos, que en un AVR de 32 kB se pagan con casi
# todo el espacio que queda para que el sketch crezca, y sobre un lazo que ya
# entra holgado en su período no compran nada que se pueda medir.
#
# El enlace también lleva `-O2`: con `-flto` el grueso de la generación de
# código pasa en el enlazado, y dejarlo en `-Os` ahí desharía lo anterior.
BUILD_PROPERTIES = [
    'compiler.c.extra_flags=-O2',
    'compiler.cpp.extra_flags=-O2',
    'compiler.c.elf.extra_flags=-O2',
]

# `ctl_mode` elige el controlador, `ctl_target` elige la realimentación sobre la que
# cierra. Se definen en catalogo.py, que no depende de nada, y se reexportan acá
# para que `from bench import *` siga trayéndolos.
from catalogo import MODE_OPEN, MODE_PID, MODE_RAMP, POSITION, CURRENT

# Bits del registro STATUS del AS5600.
_MAGNET_STRONG, _MAGNET_WEAK, _MAGNET_PRESENT = 0x08, 0x10, 0x20

# El reloj de la placa. La frecuencia del PWM se guarda como el TOP del Timer1,
# que es por lo que cuenta el hardware, así que la conversión a Hz pasa por acá.
_F_CPU = 16_000_000

# La placa informa siempre en cuentas de 12 bits, tenga el ADC de 10 bits del UNO
# o el de 12 del clon: normaliza ella, y publica en `adcfs` cuál de los dos es.
# Ver ControlDemo.ino. Dónde tiene que reposar el sensor depende de cuál sea y de
# cómo esté alimentado, así que acá no se juzga el valor: se juzga que quede fuera
# de los rieles --contra un riel no hay una corriente grande sino una entrada al
# aire-- y que sobre margen hacia arriba para que una corriente tenga adónde
# crecer.
_ADC_FULL     = 4095
_ADC_RAIL     = 80   # a menos de esto de cualquiera de los dos extremos
_ADC_HEADROOM = 400  # cuentas de margen que se le piden al reposo

# Qué le pasa a cada línea del bus I2C, medido por la placa al arrancar. Los
# códigos son los del enum de BoardStart.h.
_BUS_LINEA = {
    0: 'ok',
    1: 'hay algo colgado pero sin alimentacion',
    2: 'el cable no llega a ningun lado',
    3: 'sujeta contra masa',
}

# Los dos cables cambiados entre sí, que no es de una línea sino de las dos.
_BUS_INVERTIDO = 0x10

# Cuánto tiene que girar el eje en un tirón de lazo abierto para que el signo de
# lo que giró signifique algo. Por debajo de esto lo que se mide es el arrastre
# del giro anterior o el ruido del sensor, no el sentido en el que empuja el
# puente. Veinte grados: bastante más que el ruido y bastante menos que lo que
# da cualquier motor que realmente arranque.
_GIRO_MINIMO = 0.05     # vueltas

# Y cuánta corriente de pico por encima del reposo cuenta como «el motor consumio
# algo». Es el otro testigo de que el actuador existe, para el banco que no tiene
# sensor de angulo.
_I_MOTOR_MA = 50

_link = None


# ------------------------------------------------------------------ el cableado

class Giro(NamedTuple):
    """Lo que deja un tirón de lazo abierto: ver `Bench.spin()`."""

    vueltas: float    # con signo: es lo que hace verificable el sentido
    corriente: float  # mA, con signo, la muestra de mayor |i| de todo el tirón


@dataclass
class Cableado:
    """Lo que en este banco se fija con cables y la placa olvida en cada reset.

    Son tres números, y los tres describen un banco y no el programa:

      - `bidir`, si el puente acciona en los dos sentidos o en uno solo;
      - `uinvert`, el signo que hace que un comando positivo *suba* el ángulo;
      - `iinvert`, el que hace que ese mismo comando dé una corriente *positiva*.

    Hacen falta los dos signos y no uno: `uinvert` da vuelta el puente, así que da
    vuelta el ángulo y la corriente a la vez. Si con el ángulo ya derecho la
    corriente sigue saliendo al revés, lo que está dado vuelta es por dónde entra
    el sensor de corriente, y eso no hay `uinvert` que lo arregle.

    El sketch arranca siempre en los mismos valores --a propósito: una placa tiene
    que arrancar diciendo lo que es y no lo que alguien le dejó puesto-- y
    `sync_board()` resetea la placa en cada celda. Así que estos tres números hay
    que volver a ponerlos cada vez, y el único que los sabe es esta máquina. Es la
    misma división de trabajo que la calibración del sensor, y por eso se guardan
    igual: un archivo del banco, fuera del repositorio.

    `bringup()` los mide y devuelve este objeto; `sync_board()` lo aplica solo.
    """

    bidir: int = 1
    uinvert: int = 0
    iinvert: int = 0

    # El veredicto de la corrida de bringup() que armó este objeto. No es parte
    # del cableado --no se guarda, y un Cableado leído del archivo lo trae en
    # True-- pero vive acá para que `if dev.bringup():` siga significando lo que
    # significaba cuando bringup() devolvía un booleano.
    ok: bool = True

    _GUARDADOS = ('bidir', 'uinvert', 'iinvert')

    # Cómo se llama cada uno en la placa. Los nombres de acá son los del banco
    # --los que se leen en un archivo y en un mensaje-- y los de allá llevan
    # prefijo de módulo. Separarlos es lo que permite que un `cableado.json` escrito
    # antes del cambio de nombres siga sirviendo.
    _PARAMETROS = {
        'bidir':   'mot_bidir',
        'uinvert': 'mot_invert',
        'iinvert': 'cur_invert',
    }

    def __bool__(self):
        return self.ok

    def aplicar(self, dev):
        """Escribe los tres parámetros en la placa. Devuelve `dev`.

        `iinvert` puede no estar: es más nuevo que los otros dos, y una placa con
        un sketch de antes no lo declara. Se saltea en lugar de fallar, porque el
        resto del cableado sigue siendo aplicable y decirlo es tarea de bringup().
        """
        for nombre in self._GUARDADOS:
            param = self._PARAMETROS[nombre]
            if param in dev._params:
                setattr(dev, param, getattr(self, nombre))
        return dev

    def guardar(self, ruta=None):
        """Lo deja escrito para las próximas celdas. Devuelve la ruta."""
        ruta = Path(ruta) if ruta else CABLEADO
        ruta.parent.mkdir(parents=True, exist_ok=True)
        datos = {k: v for k, v in asdict(self).items() if k in self._GUARDADOS}
        ruta.write_text(json.dumps(datos, indent=1) + '\n')
        return ruta

    @classmethod
    def cargar(cls, ruta=None):
        """El cableado guardado, o None si no hay archivo.

        Un archivo ilegible es lo mismo que no tenerlo: el banco anda igual, sólo
        que con los valores por omisión del sketch, y eso es preferible a que una
        celda no arranque por un JSON a medio escribir.
        """
        ruta = Path(ruta) if ruta else CABLEADO
        try:
            datos = json.loads(ruta.read_text())
        except (OSError, ValueError):
            return None
        return cls(**{k: int(v) for k, v in datos.items() if k in cls._GUARDADOS})

    def __str__(self):
        return (f'bidir = {self.bidir}, uinvert = {self.uinvert}, '
                f'iinvert = {self.iinvert}')


# ------------------------------------------------------- el diagnóstico del banco

class DiagnosticoDeBanco:
    """Lo que hay que saber de este equipo para creerle una captura.

    Existe porque `ctrllink` no tiene que saber qué hay del otro lado del cable. El
    protocolo habla de períodos perdidos y de filas descartadas; que además haya un
    AS5600 en un bus I2C, con un imán que puede estar torcido, es de este banco. El
    enlace lo recibe como colaborador y le pregunta; ver el comentario de
    `diagnostico` en ctrllink.py.

    Es de sólo leer: trabaja sobre lo que la captura ya trajo en `df.attrs` y no le
    pide nada a la placa. Eso lo hace barato de llamar y trivial de probar.
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
        """Va antes que las del lazo: si el sensor no está, lo demás es consecuencia."""
        if df.attrs.get('spres') != 0:
            return []

        return ['el AS5600 no contesta en el bus I2C: revisar SDA (A4), '
                'SCL (A5), la alimentacion y los pull-ups. El lazo sigue '
                'corriendo, pero el angulo queda congelado y todo lo que se '
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
        # ruido. Con el sensor presente, en cambio, son intermitencias y ésas sí
        # importan. Por eso las dos notas tienen que vivir juntas: una condiciona a
        # la otra, y partirlas entre dos módulos rompía ese acoplamiento.
        serr = df.attrs.get('serr') or 0
        if serr and df.attrs.get('spres') != 0:
            notas.append(f'fallaron {serr} transferencia(s) del sensor -- revisar '
                         f'el cableado y los pull-ups del bus.')

        return notas


# -------------------------------------------------------------- el dispositivo

class Bench:
    """Un banco: un enlace CtrlLink más lo que significan los números de este equipo.

    Tiene un enlace en lugar de ser uno. La diferencia importa: lo que este objeto
    sabe --que el ángulo se mide en cuentas de un AS5600, que la corriente pasa por
    un ACS712, que el actuador es un puente de un modelo que no tolera modular
    rápido-- no tiene por qué poder meterse adentro del protocolo, y cuando podía,
    se metió: las quejas sobre una captura nombraban el sensor y sus pines desde
    ctrllink.py. Con el enlace guardado en un atributo esa fuga deja de ser posible
    en lugar de quedar prohibida por costumbre.

    Todo lo que el enlace sabe hacer sigue estando acá, delegado: `capture`, `step`,
    `set`, `get`, `close`, y cada parámetro de la placa como atributo. Los
    parámetros están en las unidades de la placa --cuentas, LSBs del ADC, ganancias
    por muestra-- y los métodos de esta clase los convierten a las unidades en las
    que se diseña un experimento.
    """

    # Los atributos que son de este objeto y no del enlace. Todo lo demás se
    # delega, así que agregar uno acá es la manera de que no se vaya al enlace.
    _PROPIOS = frozenset({'link', 'channels'})

    def __init__(self, port=None, **kw):
        # object.__setattr__ porque __setattr__ consulta el enlace, que todavía no
        # existe.
        link = CtrlLink(port, diagnostico=DiagnosticoDeBanco(), **kw)
        object.__setattr__(self, 'link', link)
        object.__setattr__(self, 'channels', self._canales(link))

    @staticmethod
    def _canales(link):
        """Los canales de la placa, con la escala de `i` corregida por placa.

        La escala que viaja en la tabla de canales es una constante compilada, y la
        referencia del ADC no lo es: el bandgap del ATmega anda por 1093 mV y la
        referencia interna del clon vale 1024. La placa no puede corregir su propia
        tabla --vive en flash-- pero sí calcula la escala al arrancar y la publica en
        `cur_ma_lsb`, en mA por cuenta. Acá se la cree a ella y no a la tabla. Ver el
        comentario de ADC_REF_MV_LGT8F en ControlDemo.ino.

        OJO que esto corrige `self.channels`, que es lo que usan deg(), ma(),
        as_deg() y as_ma(). Las columnas de un DataFrame capturado se escalan con lo
        que viene en el encabezado del flujo, o sea con la constante compilada: en el
        UNO las dos coinciden y en el clon no. Arreglarlo cambiaría los números de
        una captura, así que no se hace de paso.
        """
        chans = list(link.channels)
        ma = link.params.get('cur_ma_lsb') and link.get('cur_ma_lsb')

        if ma:
            for columna in chans:
                if columna.name == 'i':
                    columna.scale = ma

        return chans

    # ----------------------------------------------------------- delegación
    #
    # Lo que no sea de este objeto es del enlace, y eso incluye cada parámetro de
    # la placa: `dev.pid_kp = 0.5` termina en un `set` como antes.

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
    # perillas no está escrita en ninguna parte de este lado: se pregunta. Lo que
    # sigue la muestra junto con el valor de ahora, la unidad y qué significa, para
    # que no haya que ir a buscar a otro archivo qué se puede tocar.
    #
    # En un notebook alcanza con poner `dev` en una celda.

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

        resumen = (f'lazo a {1 / self.dt:.0f} Hz, PWM a {self.pwm_hz:.0f} Hz, '
                   f'{len(self.channels)} canales de telemetría')

        return self.info, resumen, nombres, self._valor_legible, canales

    def _valor_legible(self, nombre):
        """El valor de ahora, o un signo de pregunta si la placa no lo contesta.

        Un parámetro que no se pueda leer no puede hacer fallar la descripción
        entera: lo que uno quiere justo en ese momento es ver la tabla para entender
        qué está pasando.
        """
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
        self.pid_kp, self.pid_ki, self.pid_kd = kp, ki * dt, kd / dt

    # Los tres filtros que el sketch expone, y de qué módulo es cada uno. No se
    # arma el nombre con una f-string porque los tres polos viven en módulos
    # distintos y cada prefijo es el de su dueño; un diccionario lo dice y una
    # interpolación lo esconde.
    _FILTROS = {'y': 'ang_alpha', 'i': 'cur_alpha', 'e': 'pid_alpha'}

    def smooth(self, which, tau):
        """Fija un filtro por constante de tiempo en segundos; tau = 0 lo apaga.

        `which` es 'y' (posición), 'i' (corriente) o 'e' (el error que ve el
        término derivativo). La placa guarda el polo en sí, alpha, porque es por
        lo que multiplica el filtro.
        """
        try:
            param = self._FILTROS[which]
        except KeyError:
            raise CtrlLinkError(
                f'no hay ningun filtro {which!r}; son '
                f'{", ".join(sorted(self._FILTROS))}') from None

        dt = self.dt
        self.set(param, dt / (tau + dt) if tau > 0 else 1.0)

    def pwm(self, hz):
        """Fija la frecuencia del PWM del puente, en Hz, y devuelve la que quedó.

        La placa guarda el TOP del Timer1, que es por lo que cuenta el
        temporizador: `f = 16 MHz / (2 * pwmtop)`. El TOP es entero, así que no
        toda frecuencia es representable; se toma la más cercana y se informa cuál
        quedó, en lugar de dejar creer que se fijó la pedida.

        El recorrido va de 122 Hz a 31,4 kHz. Por omisión son 1 kHz, que es lo que
        tolera un L298N alimentado con 5 V --más arriba las pérdidas de conmutación
        se comen un tiempo de encendido que ya viene escaso y el motor no arranca.
        Con un puente MOSFET conviene subirla bien por encima del rango audible.
        """
        top = min(65535, max(255, round(_F_CPU / (2 * hz))))
        self.mot_top = top
        return _F_CPU / (2 * top)

    @property
    def pwm_hz(self):
        """La frecuencia del PWM del puente, en Hz, tal como quedó en la placa."""
        return _F_CPU / (2 * self.mot_top)

    def zero(self):
        """Toma la posición actual del eje como cero.

        `y` se lee como `offset - counts` con vuelta, así que bajar `offset` en el
        `y` actual pone `offset` sobre la cuenta actual y `y` en cero.
        """
        self.ang_offset = (self.ang_offset - self.ang_y) % 4096
        self.ang_y_uw = 0

    def zero_current(self, seconds=0.3):
        """Toma la corriente que se mida ahora como el cero. Devuelve `izero`.

        Con el puente abierto no circula corriente, así que lo que marque el sensor
        es su offset: el suyo propio, más la tolerancia de su alimentación y la de
        cualquier divisor que haya en el medio. Un cero corrido es un error que
        después se integra en toda medición, y no hay forma de conocerlo salvo
        midiéndolo acá.

        `i` se lee como `adc - izero`, así que sumarle a `izero` la lectura actual
        pone el cero sobre la cuenta actual y `i` en cero. Es exactamente lo que
        hace zero() con el ángulo. Deja el motor en reposo, que es la condición
        bajo la cual la medición significa algo.

        Con `iinvert` puesto la placa informa `izero - adc`, así que la corrección
        va para el otro lado. El offset se mide en cuentas del ADC y el signo se
        aplica después, que es el orden en el que lo hace la placa: ver measure()
        en ControlDemo.ino.
        """
        self.rest()
        df = self.capture(seconds, warn=False)
        self.cur_zero = round(self.adc_de_i(df['i'].mean()))
        return self.cur_zero

    def adc_de_i(self, ma):
        """Los mA que informa el canal `i` -> la cuenta cruda del ADC que los produjo.

        Deshace las dos cosas que la placa le hace a la lectura --el offset y, si
        está puesto, el signo-- y es lo que permite juzgar si la entrada está
        contra un riel, que es una pregunta sobre el ADC y no sobre la corriente.
        """
        cuentas = ma / self.channel('i').scale
        return self.cur_zero + (-cuentas if self._iinvert else cuentas)

    @property
    def _iinvert(self):
        """`iinvert`, o 0 si la placa tiene grabado un sketch que no lo declara."""
        return int(self.cur_invert) if 'cur_invert' in self._params else 0

    def rest(self):
        """Lazo abierto, comando en cero. Donde tendría que terminar todo experimento.

        Con el comando en cero el sketch deja ENA en bajo, así que el puente queda
        abierto y el motor en punto muerto: no frena el eje, sólo deja de
        empujarlo.
        """
        self.ctl_mode = MODE_OPEN
        self.ctl_uff = 0

    def spin(self, u, seconds=0.4, espera=6.0, quieto=5.0):
        """Lazo abierto con `u` sobre el puente por un instante, y de vuelta a reposo.

        Devuelve un `Giro`: vueltas, corriente y ruido. Las vueltas van con signo,
        que es lo que hace verificable el sentido.

        De la corriente se informa la muestra de mayor módulo de todo el tirón, con
        su signo, y no el promedio. El promedio de un tirón es casi todo el régimen,
        y un motor sin carga en régimen no consume nada: la fuerza contraelectromotriz
        le cancela el comando, así que la corriente vive en el arranque y en ningún
        otro lado. Medido en este banco, con el eje parado y un escalón a u = 255:
        925 mA de pico contra 125 mA ya girando, y a u = 120, 317 mA contra cero. Un
        promedio sobre los 0,4 s enteros diluye eso hasta el ruido, y entonces el
        signo que informa es el del ruido.

        Contra qué se juzga esa muestra no sale de acá: sale del ruido del canal
        en reposo, con el puente abierto. La dispersión de este tirón no sirve,
        porque adentro está la propia corriente que se quiere medir --arranca en el
        pico y cae a nada-- así que crece justo cuando hay más señal. Ver
        `bringup()`. Que el sensor vea o no el signo depende además de en qué parte
        del circuito esté insertado --uno unipolar da positivo en los dos
        sentidos-- y eso también es algo que hay que medir, no suponer.

        Espera primero a que el eje esté realmente quieto, y esa espera no es una
        precaución de más: con el puente abierto el motor no frena, sigue por
        inercia, y en este banco tarda segundos en parar desde las decenas de
        vueltas por segundo a las que llega. Midiendo enseguida, lo que se mide es
        la inercia de la medición anterior. Así se equivocó de signo la
        verificación de polaridad --dos veces, y con el sentido opuesto cada vez--,
        que es justamente la que existe porque probando no se descubre.

        `quieto` es el umbral en grados por segundo por debajo del cual se
        considera parado.
        """
        self.rest()

        limite = time.monotonic() + espera
        arrastre = None

        while True:
            reposo = self.capture(0.2, warn=False)
            arrastre = abs(reposo['y_uw'].iloc[-1] - reposo['y_uw'].iloc[0]) / 0.2

            if arrastre < quieto or time.monotonic() > limite:
                break

        self.zero()
        self.ctl_uff = u
        df = self.capture(seconds, warn=False)
        self.rest()

        vueltas = (df['y_uw'].iloc[-1] - df['y_uw'].iloc[0]) / 360.0

        # Si no llegó a parar, el número que sigue no significa lo que dice, y es
        # mejor que quien mira lo sepa que un signo inventado con cara de dato.
        if arrastre >= quieto:
            print(f'  OJO: el eje seguia girando a {arrastre:.0f} grados/s al empezar '
                  f'esta medicion; el sentido que informa puede ser el de la inercia')

        # La muestra de mayor módulo, con su signo: es el instante en el que el
        # sensor tiene algo que decir, y el único en el que la relación entre señal
        # y ruido de este canal alcanza para creerle un signo.
        i = df['i']
        return Giro(vueltas, i.iloc[i.abs().to_numpy().argmax()])

    # ------------------------------------------------------- puesta en marcha

    def bringup(self, motor=True, u=120):
        """Verifica el hardware, un subsistema por vez. Devuelve un `Cableado`.

        Cada línea es algo que puede estar mal por su cuenta: el enlace, el lazo,
        el imán, el bus I2C, el cero de la medición de corriente --que de paso se
        calibra--, el actuador y el sentido en el que empuja. Conviene
        correrlo primero, y después de cualquier cambio en el cableado: un
        controlador ajustado contra un sensor que no está leyendo es una tarde
        larga.

        No sólo mide los dos signos del banco: los *deja puestos*. Un diagnóstico
        que dice «poner uinvert = 1» y deja la placa como estaba hace falla la
        celda siguiente, y la que la corre ya se olvidó de lo que decía la línea.
        Así que bringup() corrige lo que encuentra, vuelve a medir para
        comprobarlo --el ángulo se mide de nuevo accionando el motor, que es lo
        único que lo prueba-- y devuelve los valores que quedaron, que son los que
        hay que volver a poner después de cada reset. Los guarda además en
        `CABLEADO`, y ahí es donde `sync_board()` los encuentra sola.

        El objeto que devuelve sigue valiendo por su veredicto, así que
        `if dev.bringup():` significa lo mismo que antes.
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
        # periodo el lazo esta llegando, y con seis canales a 500 Hz el retardo
        # ronda el 30 % del periodo por el solo costo de emitir la fila --emitir
        # cuesta lo mismo que a 1 kHz, y el periodo es el doble. Lo que si es una
        # falla es no tener margen alguno.
        late   = df.attrs['maxlate']
        margin = late / df.attrs['dt_us']
        report('margen de tiempo',
               True if margin < 0.5 else (None if margin < 1.0 else False),
               f'peor retardo de atencion {late} us de {df.attrs["dt_us"]} us '
               f'({margin:.0%})')

        # 2. La placa, que en este banco puede ser una de dos y no se distinguen
        #    a simple vista. Importa porque el ADC del clon tiene 12 bits contra
        #    los 10 del UNO: la placa normaliza a 12 y lo dice acá, así que un
        #    canal de corriente cuatro veces grande deja de ser un misterio.
        if 'board_adcfs' in self._params:
            fondo = self.get('board_adcfs')
            bg    = self.get('board_bgadc')
            report('placa', True,
                   'ADC de 12 bits, el del clon LGT8F328P' if fondo >= 4096 else
                   'ADC de 10 bits, el del ATmega328P; las lecturas se corren '
                   '2 bits para contar en 12')

            # La referencia interna, que es la ganancia del canal de corriente
            # cuando se la elige. El divisor de esta cuenta es Vcc en las dos
            # placas --REFS=01 es AVcc en el ATmega y DEFAULT en el LGT8F, y los
            # dos valen 1-- así que la razón significa lo mismo acá y allá. Lo que
            # cambia es el numerador: en el ATmega el canal interno es el bandgap
            # de 1,1 V y la cuenta cierra en ADC_REF_MV; en el clon ese canal no es
            # ese bandgap, así que la misma razón no da ADC_REF_MV y ofrecerla
            # sería ofrecer una calibración que no lo es.
            if bg and fondo <= 1024:
                report('referencia', None,
                       f'bandgap {bg} cuentas de {fondo}: la referencia interna '
                       f'vale AVcc*{bg}/{fondo}, o sea {5000 * bg / fondo:.0f} mV '
                       f'si AVcc fuera 5000 mV. Medir AVcc y poner el resultado '
                       f'en ADC_REF_MV de ControlDemo.ino')
            elif bg:
                report('referencia', None,
                       f'{bg} cuentas de {fondo} contra Vcc, que es contra lo que '
                       f'mide REFS=01 tambien en este chip. Pero el canal interno '
                       f'del clon no es el bandgap de 1,1 V del ATmega, asi que '
                       f'esta razon no da ADC_REF_MV: no usarla para calibrar el '
                       f'canal de corriente. Con la referencia alta no hace falta, '
                       f'porque es Vcc y el ACS712 reposa en media escala sola')

        # 3. Los cables del bus, medidos por la placa antes de encender el TWI.
        #    Va antes que el sensor a propósito: si el sensor no contesta, esta
        #    línea dice si es un cable, una alimentación o un corto, que desde el
        #    protocolo se ven los tres igual.
        #
        #    Pero es una foto del arranque y no una medición de ahora: la placa la
        #    saca una sola vez, antes de que el TWI tome las líneas, porque después
        #    ya no puede --el periférico gobierna los pines. Así que hay que
        #    contrastarla con algo vivo, y ese algo es `spres`: si el sensor está
        #    contestando en este momento, el bus está bien ahora, diga lo que diga
        #    la foto. Sin esto, alguien que ve «SDA y SCL estan cambiados»,
        #    intercambia los cables y vuelve a correr bringup() recibe el mismo
        #    veredicto sobre un cableado que ya arregló, y se pone a buscar en el
        #    lugar equivocado. Lo que hace falta no es tocar más cables: es
        #    resetear la placa para que vuelva a mirar, y eso es sync_board().
        present = bool(df.attrs.get('spres', 1))

        if 'board_bus' in self._params:
            diag = self.get('board_bus')
            sda, scl = diag & 0x03, (diag >> 2) & 0x03
            sano = not (diag & _BUS_INVERTIDO) and sda == 0 and scl == 0

            if present and not sano:
                report('cables del bus', None,
                       'la placa vio un problema en el bus al arrancar y el sensor '
                       'esta contestando igual, asi que esa foto ya no describe el '
                       'bus de ahora: o los cables se arreglaron despues, o la '
                       'medicion del arranque agarro un transitorio. Correr '
                       'sync_board() para que la placa la vuelva a sacar')
            elif diag & _BUS_INVERTIDO:
                report('cables del bus', False,
                       'SDA y SCL estan cambiados entre si: el sensor contesta '
                       'hablandole al reves. Intercambiar los dos cables y correr '
                       'sync_board(), que resetea la placa y la hace medir de nuevo')
            else:
                report('cables del bus', sano,
                       f'SDA {_BUS_LINEA.get(sda, "?")}, '
                       f'SCL {_BUS_LINEA.get(scl, "?")}')

        # 4. El sensor, antes que el imán: si el AS5600 no contesta en el bus, lo
        #    que diga su registro del imán no significa nada, y conviene decir
        #    cuál de los dos problemas es.
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
            # El AGC en números, no sólo "no está en ningún extremo". Es la
            # compuerta de la calibración: contra un borde el imán está a la
            # distancia equivocada, y ahí ninguna tabla arregla nada porque el
            # error deja de ser una función suave del ángulo. La banda cómoda es
            # el tercio del medio; el registro va de 0 a 255 alimentado a 5 V, y
            # de 0 a 128 a 3,3 V.
            agc = df.attrs.get('agc')
            if agc is None:
                report('iman', True, 'detectado, AGC en rango')
            else:
                comodo = 32 <= agc <= 224
                report('iman', True if comodo else None,
                       f'detectado, AGC {agc:.0f}/255, campo {df.attrs.get("mag", 0):.0f}'
                       + ('' if comodo else '  (cerca del borde: centrar la distancia'
                                            ' antes de calibrar)'))

        # 4. El bus que transporta el ángulo, por separado del imán que está en la
        #    otra punta: un problema de pull-ups y uno de montaje se ven igual en
        #    los datos y se arreglan en lugares distintos. Con el sensor ausente
        #    las fallas son las del sondeo espaciado, así que no dicen nada nuevo.
        if present:
            # Un desborde no es un error: es una muestra que el bus no llegó a
            # entregar antes del tick siguiente. Un puñado por segundo es normal y
            # no se puede evitar, porque el diagnóstico del imán lee un registro
            # dos veces por segundo y una lectura con dirección de registro no
            # entra en el período de 200 us; la muestra siguiente además tiene que
            # recargar el puntero. Medido en este banco: unas dos de cada 5000.
            #
            # Exigir cero hacía fallar esta línea en un equipo sano, y una
            # verificación que grita en falso enseña a ignorarla. Lo que sí es
            # una falla es que el bus no llegue de manera sostenida.
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
        #    En reposo el puente está abierto y no circula corriente, así que lo
        #    que marque el sensor es su offset y restarlo es todo lo que hace
        #    falta. Pero hay dos casos que un offset no arregla y que conviene no
        #    tapar calibrándolos:
        #
        #    Una entrada al aire termina contra un riel del ADC. Eso es una
        #    ausencia, no un offset, y calibrarla dejaría un canal que informa
        #    ceros perfectos sin haber medido nada.
        #
        #    Y un reposo pegado a un extremo no deja lugar para medir, aunque no
        #    llegue al riel: la corriente tiene que poder crecer para los dos lados
        #    sin recortar. Eso se juzga por el margen y no por el valor, porque
        #    dónde reposa depende del sensor y de con qué esté alimentado.
        lsb = self.channel('i').scale
        adc = self.adc_de_i(df['i'].mean())
        sensed = _ADC_RAIL <= adc <= (_ADC_FULL - _ADC_RAIL)
        noise = 0.0

        if not sensed:
            # Contra el riel de arriba hay dos causas y se arreglan en lugares
            # distintos, así que conviene nombrar las dos en lugar de adivinar: la
            # entrada al aire, y la entrada que mide bien pero contra una
            # referencia más chica que ella. Un ACS712 alimentado a 5 V reposa en
            # Vcc/2, o sea 2,5 V, y la referencia interna vale 1,1: ese sensor
            # satura en reposo y desde acá se ve igual que un cable que no está. La
            # diferencia la hace de qué riel se trata.
            rest_ma = 0.0
            arriba = adc > _ADC_FULL / 2
            report('cero de i', None,
                   f'entrada contra el riel {"de arriba" if arriba else "de abajo"} '
                   f'del ADC ({adc:.0f} de {_ADC_FULL}): '
                   + ('no hay nada conectado en A0, o lo que hay reposa por encima '
                      'de la referencia y satura: un ACS712 alimentado a 5 V reposa '
                      'en Vcc/2 y la interna vale 1,1 V. Ver README'
                      if arriba else
                      'no parece haber nada conectado en A0'))
        else:
            # Lo que hay que mirar es el margen, no el valor: desde el reposo hasta
            # el tope de escala es todo lo que una corriente puede crecer antes de
            # recortar, y hacia abajo es lo mismo para el otro sentido de giro.
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

            # Un canal demasiado quieto es tan sospechoso como uno ruidoso. Un
            # ruido de cero exacto no es una medicion limpia: es una senal mas
            # chica que un escalon de cuantizacion, y entonces el ADC devuelve
            # siempre la misma cuenta y no hay forma de saber que hay abajo. Con
            # dither de una fraccion de LSB, en cambio, el promedio de la ventana
            # resuelve por debajo del escalon.
            report('calibracion de i',
                   abs(rest_ma) < lsb and 0.1 * lsb < noise < 8 * lsb,
                   f'izero = {self.cur_zero}, {lsb:.2f} mA por cuenta, quedan '
                   f'{rest_ma:+.1f} mA en reposo (ruido {noise / lsb:.2f} cuentas)'
                   + ('' if noise > 0.1 * lsb else
                      '  -- sin dither: la senal no llega a un escalon del ADC'))

        # 6. El actuador, y con él toda la cadena: un comando que sale, movimiento
        #    y corriente que vuelven. Hay dos evidencias posibles y cada una
        #    depende de su propio sensor, así que sólo se usa la que esté
        #    disponible: dar por bueno un motor porque la corriente se movió,
        #    cuando la entrada de corriente está al aire, es peor que no medir.
        if not motor:
            for etiqueta in ('motor', 'polaridad', 'sentido', 'signo de i'):
                report(etiqueta, None, 'omitido (motor=False)')
        else:
            print(f'  accionando el motor con u = {u:+} durante 0,4 s, '
                  f'PWM a {self.pwm_hz / 1000:.1f} kHz ...')
            ida = self.spin(u)

            evidence = ([f'{abs(ida.vueltas):.2f} vueltas'] if present else []) + \
                       ([f'{abs(ida.corriente):.0f} mA de pico'] if sensed else [])

            if not evidence:
                report('motor', None, 'no se puede evaluar: no hay sensor de '
                                      'angulo ni medicion de corriente')
            else:
                report('motor',
                       (present and abs(ida.vueltas) > _GIRO_MINIMO) or
                       (sensed and abs(ida.corriente) > abs(rest_ma) + _I_MOTOR_MA),
                       ', '.join(evidence))

            # 7. La polaridad: un comando positivo tiene que hacer *subir* el
            #    ángulo medido. Si lo hace bajar, el lazo de posición realimenta en
            #    positivo y se escapa con una referencia de cualquier signo, así que
            #    probando no se descubre. Depende de dos cables --los del motor en
            #    el puente, y el sentido en que el imán mira al sensor-- y `uinvert`
            #    es el que los reconcilia.
            #
            #    Va antes que el sentido por dos razones. Una es que define qué
            #    quiere decir «un comando positivo», y todo lo que sigue lo usa. La
            #    otra es que, al revés que el sentido, no necesita que el
            #    puente invierta: `uinvert` da vuelta el sentido adentro de drive()
            #    y eso vale igual con `bidir = 0`, donde el comando nunca es
            #    negativo pero sí sale por la otra entrada. Un banco de un solo
            #    cuadrante también puede tener el imán al revés, y hasta acá se
            #    quedaba sin la única verificación que lo detecta.
            #
            #    Lo que `uinvert` no puede arreglar es un sentido fijado en cobre:
            #    IN1 e IN2 atados a riel, o un actuador de un solo transistor, no
            #    escuchan qué pin levanta drive(). Por eso se mide de nuevo después
            #    de darlo vuelta en lugar de darlo por arreglado: si el ángulo
            #    sigue bajando, el arreglo no es un parámetro sino dos cables.
            if not present:
                report('polaridad', None, 'no se puede evaluar sin el sensor de angulo')
            elif abs(ida.vueltas) <= _GIRO_MINIMO:
                report('polaridad', None,
                       f'no se puede evaluar: con u = {u:+} el eje no se movio')
            else:
                if ida.vueltas < 0:
                    print(f'  un u positivo baja el angulo: poniendo uinvert = '
                          f'{0 if self.mot_invert else 1} y midiendo de nuevo ...')
                    self.mot_invert = 0 if self.mot_invert else 1
                    ida = self.spin(u)

                derecho = ida.vueltas > 0
                report('polaridad', derecho,
                       f'un u positivo hace {"subir" if derecho else "BAJAR"} el '
                       f'angulo, con uinvert = {self.mot_invert}'
                       + ('' if derecho else
                          '  -- dar vuelta uinvert no cambio nada, asi que el '
                          'sentido de este banco esta fijado en cobre y no en '
                          'software: IN1 (6) e IN2 (7) atados, o un actuador de un '
                          'solo transistor. Dar vuelta los dos cables del motor, o '
                          'el iman sobre el sensor'))

            # 8. El sentido, que es una verificación aparte porque falla aparte y
            #    en otro lado: IN1 e IN2 intercambiados, o uno de los dos sin
            #    conectar, dejan pasar todo lo anterior y recién se notan como un
            #    lazo cerrado que se escapa en vez de establecerse. La evidencia
            #    tiene que ser el angulo: la corriente mide lo mismo en los dos
            #    sentidos, asi que no distingue el caso.
            vuelta = None
            reverses = False

            if not self.mot_bidir:
                report('sentido', None, 'bidir = 0, el puente esta declarado de un '
                                        'solo cuadrante: no hay segundo sentido que '
                                        'verificar')
            elif not present:
                report('sentido', None, 'no se puede evaluar sin el sensor de angulo')
            else:
                print(f'  y ahora con u = {-u:+} ...')
                vuelta = self.spin(-u)
                reverses = (ida.vueltas * vuelta.vueltas < 0 and
                            abs(vuelta.vueltas) > _GIRO_MINIMO)

                # Las dos maneras de no invertir se arreglan en lugares distintos
                # y se distinguen en los datos, así que conviene no meterlas en el
                # mismo consejo. Si el eje gira para el mismo lado con las dos
                # polaridades, el puente sí acciona en los dos sentidos y lo que
                # está mal es qué entrada va a qué pin. Si con el comando negativo
                # no se mueve nada, el puente no tiene el segundo cuadrante: no hay
                # nada que arreglar en el cableado, hay que decírselo al lazo para
                # que recorte en cero y el anti-windup se entere.
                #
                # Eso último no se decide acá aunque se vea: un IN2 suelto da
                # exactamente la misma medición que un puente de un solo cuadrante,
                # y poner `bidir = 0` sólo taparía un cable flojo dándole nombre de
                # topología. La diferencia está en el banco, no en los datos.
                if reverses:
                    pista = ''
                elif abs(vuelta.vueltas) <= _GIRO_MINIMO:
                    pista = ('  -- con el comando negativo no se movio: si el puente '
                             'es de un solo cuadrante, poner dev.bidir = 0; si no, '
                             'revisar IN2 (7)')
                else:
                    pista = ('  -- giro para el mismo lado con las dos polaridades: '
                             'el puente no esta escuchando IN1 (6) ni IN2 (7). Si '
                             'estan fijos por cable --el montaje de un solo '
                             'cuadrante-- poner dev.bidir = 0, que es declararlo; '
                             'si no, revisar los dos')

                report('sentido', reverses,
                       f'{ida.vueltas:+.2f} vueltas con u = {u:+}, '
                       f'{vuelta.vueltas:+.2f} con u = {-u:+}' + pista)

            # 9. Y el signo de la corriente, que es el mismo problema que la
            #    polaridad sobre el otro sensor: un comando positivo tiene que dar
            #    una corriente positiva, porque con target = CURRENT el lazo cierra
            #    sobre ella y realimentar con el signo cambiado no se establece.
            #
            #    Hace falta un segundo parámetro y no alcanza con `uinvert` porque
            #    `uinvert` da vuelta el puente, o sea las dos cosas a la vez: si el
            #    ángulo ya quedó derecho y la corriente sigue saliendo al revés, lo
            #    que está dado vuelta es por dónde entra el sensor de corriente.
            #
            #    El umbral no es un número elegido: son cinco veces el ruido que
            #    este canal tiene *en reposo*, con el puente abierto, que es el que
            #    se acaba de medir unas líneas más arriba. O sea la pregunta de si
            #    lo que se está por creer es corriente o es una muestra de ruido.
            #
            #    Tiene que ser el ruido en reposo y no la dispersión del tirón: en
            #    el tirón está adentro la corriente misma, que arranca en el pico y
            #    cae a nada, así que esa dispersión crece justo cuando hay más
            #    señal. Medido en este banco, con el tirón el umbral daba 302 mA
            #    contra un pico de 225 y rechazaba una medición perfectamente buena.
            #
            #    El piso de tres cuentas es para que un canal silencioso --uno que
            #    no llega a un escalón del ADC-- no se cuele por tener el ruido
            #    chico justamente porque no está midiendo nada.
            umbral = max(5 * noise, 3 * lsb)

            if not sensed:
                report('signo de i', None,
                       'no se puede evaluar sin medicion de corriente')
            elif abs(ida.corriente) <= umbral:
                report('signo de i', None,
                       f'{ida.corriente:+.0f} mA de pico con u = {u:+}, que no se '
                       f'despegan del ruido del canal ({umbral:.0f} mA): el motor '
                       f'consume muy poco para que el signo signifique algo')
            elif ida.corriente < 0 and 'cur_invert' not in self._params:
                report('signo de i', False,
                       f'{ida.corriente:+.0f} mA de pico con un u positivo, y este '
                       f'sketch no declara iinvert: regrabar la placa con '
                       f'sync_board(force_upload=True), o dar vuelta el sensor')
            else:
                if ida.corriente < 0:
                    self.cur_invert = 0 if self._iinvert else 1
                    print(f'  la corriente sale negativa con un u positivo: '
                          f'poniendo iinvert = {self._iinvert}')
                    ida = ida._replace(corriente=-ida.corriente)
                    if vuelta is not None:
                        vuelta = vuelta._replace(corriente=-vuelta.corriente)

                # Con los dos sentidos medidos se ve además de qué clase es el
                # sensor, y eso decide qué se puede hacer con el canal: uno
                # unipolar --el que va en la alimentación del puente-- entrega el
                # módulo, así que da positivo en los dos sentidos y no hay `iinvert`
                # que le devuelva un signo que nunca midió.
                #
                # Pero eso sólo se puede concluir si el eje realmente dio vuelta.
                # En un banco cuyo puente no invierte --IN1 e IN2 atados-- la
                # corriente sale igual en las dos pasadas porque circula igual, y
                # leer ahí un sensor unipolar es culpar al sensor de lo que hace el
                # puente. Este banco lo hizo decir exactamente eso.
                bipolar_medible = reverses and vuelta is not None
                unipolar = (bipolar_medible and
                            abs(vuelta.corriente) > umbral and vuelta.corriente > 0)

                detalle = (f'{ida.corriente:+.0f} mA de pico con u = {u:+}, con '
                           f'iinvert = {self._iinvert}')

                if vuelta is not None and abs(vuelta.corriente) > umbral:
                    detalle += f', {vuelta.corriente:+.0f} mA con u = {-u:+}'

                if unipolar:
                    detalle += ('  -- los dos sentidos dan positivo: el sensor mide '
                                'el modulo y no el signo, asi que no se puede cerrar '
                                'el lazo sobre la corriente con signo')
                elif vuelta is not None and not reverses:
                    detalle += ('  -- de que clase es el sensor no se puede decir '
                                'todavia: el puente no invirtio, asi que la '
                                'corriente tampoco tenia por que invertir')

                report('signo de i', None if unipolar else True, detalle)

        bad = results.count(False)
        print(f'\n{"todas las verificaciones pasaron" if not bad else f"FALLARON {bad} verificacion(es)"}')

        # Y lo que hay que volver a poner después de cada reset, que es todo lo que
        # acá se midió y la placa no recuerda. Se devuelve y se guarda: devolverlo
        # es para quien quiera aplicarlo a mano, y guardarlo es para que
        # `sync_board()` lo aplique sin que nadie tenga que acordarse.
        cableado = Cableado(bidir=int(self.mot_bidir), uinvert=int(self.mot_invert),
                            iinvert=self._iinvert, ok=not bad)
        ruta = cableado.guardar()
        print(f'cableado de este banco: {cableado}\n'
              f'  anotado en {ruta.name}; sync_board() lo vuelve a poner en cada celda')
        return cableado


# ------------------------------------------------------- compilación y carga

def _sources_hash():
    """Huella digital de todo aquello a partir de lo cual se construye el sketch.

    Por contenido y no por marca de tiempo: un checkout de git reescribe las
    mtime sin cambiar una línea, y si no dispararía una recompilación al pedo.

    Las banderas de compilación entran en la huella junto con las fuentes: un
    `build/` que quedó de una corrida con otra optimización tiene las mismas
    fuentes y un binario que ya no es el que corresponde.
    """
    digest = hashlib.sha256()
    digest.update(repr(BUILD_PROPERTIES).encode())
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


def _upload(port, state, say):
    """Carga el binario, averiguando sola con qué bootloader habla esta placa.

    Devuelve el FQBN que anduvo, y lo deja anotado en el estado para la próxima
    vez: probar de nuevo cuesta segundos y el resultado no cambia mientras sea la
    misma placa en el mismo puerto. Si la anotación quedó vieja --se cambió la
    placa de puerto, o el puerto de placa-- el intento falla y se sigue con los
    otros perfiles, así que la memoria acelera pero no decide.
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


def sync_board(port=None, force_compile=False, force_upload=False, verbose=True,
               cableado=None):
    """Pone al día la placa y el enlace, y reconecta. Devuelve un Bench.

    Compila sólo cuando algún archivo fuente cambió de verdad, carga sólo cuando
    el binario resultante difiere del que este puerto recibió por última vez, y
    siempre reabre el enlace, lo que resetea la placa, así que el lazo arranca
    desde los valores por omisión del sketch haya hecho falta o no grabar. Ese
    reset es el motivo de llamarlo al principio de cada celda.

    Y es también el motivo de `cableado`. El reset devuelve la placa a los valores
    del sketch, que son los de un banco genérico, y hay tres que no lo son:
    `bidir`, `uinvert` e `iinvert` describen cómo está cableado *este* banco. Sin
    volver a ponerlos, cada celda empieza con el puente declarado bidireccional y
    los dos signos en cero, que para medio banco es al revés. Así que se aplican
    acá: por omisión los que dejó anotados `bringup()` en `CABLEADO`, y si no hay
    archivo, los del sketch, que es lo mismo que había antes.

    `cableado` acepta un `Cableado`, una ruta a un archivo de cableado, o
    `Cableado()` para arrancar con los valores de fábrica a propósito.

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

        output = _run(['arduino-cli', 'compile', '--fqbn', FQBN,
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

    if not isinstance(cableado, Cableado):
        cableado = Cableado.cargar(cableado)

    if cableado is not None:
        cableado.aplicar(_link)
        say(f'  cableado: {cableado}')

    return _link


def sync_board_cal(*args, calibracion=None, **kw):
    """`sync_board()` y, encima, la calibración del sensor de este banco.

    Es lo que conviene llamar al principio de cada celda. `sync_board()` resetea
    la placa, y la placa arranca siempre **sin calibrar** --a propósito: una tabla
    vieja aplicándose en silencio es peor que ninguna--, así que sin este paso
    cada celda mediría con el error de ángulo crudo del sensor. En este banco eso
    son unos veinticinco grados pico a pico.

    Cargar la tabla son 64 escrituras de parámetro, del orden de un segundo, y se
    pagan una vez por celda. Si no hay archivo de calibración lo dice y sigue: el
    lazo anda igual, sólo que sobre un ángulo torcido.

    `calibracion` es la ruta del archivo; por omisión el de este repositorio, que
    se resuelve desde acá y no desde el directorio de trabajo, así que da igual
    desde dónde se corra el notebook.

    El cableado del banco no hace falta pedirlo acá: lo aplica `sync_board()`, que
    es por donde pasan las dos. Ver su `cableado`.
    """
    dev = sync_board(*args, **kw)

    ruta = Path(calibracion) if calibracion else CALIBRACION
    verbose = kw.get('verbose', True)

    if not ruta.exists():
        if verbose:
            print(f'  sin calibracion ({ruta.name} no existe): el angulo va crudo. '
                  f'Correr notebooks/calibracion.ipynb para medirla.')
        return dev

    # El import va acá adentro y no arriba: calib trae numpy y el ajuste por
    # mínimos cuadrados, y nada de eso hace falta para hablar con la placa.
    import calib

    cal = calib.asegurar(dev, ruta)

    if verbose:
        pico = max(abs(v) for v in cal.lut) / calib.OCTAVOS
        print(f'  calibracion "{cal.banco}" del {cal.creada[:10]}: '
              f'{len(cal.armonicos)} armonicos, corrige hasta '
              f'{pico * calib.GRADOS_POR_CUENTA:.1f} grados')

    return dev
