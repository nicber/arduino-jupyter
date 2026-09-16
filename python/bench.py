"""El banco: compilación, conexión y las verificaciones de este equipo.

Todo lo que hay acá es infraestructura. Vive fuera del notebook para que una celda del
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

import json
import time
from pathlib import Path

import serial

import catalogo
import ensayo
import placa
from ctrllink import CtrlLink, CtrlLinkError

__all__ = ['sync_board', 'Bench', 'CtrlLinkError', 'CABLEADO']

_HERE = Path(__file__).resolve().parent.parent

# Los signos de este banco: de qué lado miran el imán y el sensor de corriente. Los
# mide `bringup()` y los carga `sync_board()`. No entra en el repositorio: es un
# dato del banco y no del proyecto. `bidir` no está acá: no se mide, lo declara quien
# conecta, y `bringup()` verifica que sea cierto.
CABLEADO = _HERE / 'notebooks' / 'cableado.json'

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
# +1,7 cuentas, una deriva lenta entre captura y captura que no se promedia. Una
# tolerancia menor que la repetibilidad del canal haría fallar la verificación en
# un equipo sano.
_RESIDUO_MAX = 3.0

_link = None


# Cuánto del principio de una captura no se usa para medir la corriente en reposo.
# Arrancar el flujo son unas líneas de encabezado de golpe por el puerto serie, y el
# tráfico corre la lectura (ver zero_current()); con capturas de pocas décimas de
# segundo eso pesa en el promedio.
_ARRANQUE_S = 0.1


def _sin_arranque(df):
    """La captura sin sus primeros `_ARRANQUE_S` segundos."""
    t = df['t'].to_numpy()
    return df[t >= t[0] + _ARRANQUE_S] if len(t) else df


def leer_cableado(ruta=CABLEADO):
    """Los signos guardados por `bringup()`, como dict, o None si no hay."""
    try:
        return json.loads(Path(ruta).read_text(encoding='utf-8'))
    except (OSError, ValueError, TypeError):
        return None


# ------------------------------------------------------- el diagnóstico del banco

class DiagnosticoDeBanco:
    """Lo que hay que saber de este equipo para validar una captura.

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

    def parametros_de_configuracion(self):
        return catalogo.de_configuracion()

    def notas_primero(self, df):
        """Va antes que las del muestreo: si el sensor no está, lo demás es consecuencia."""
        if df.attrs.get('spres') != 0:
            return []

        return ['el AS5600 no contesta en el bus I2C: revisar SDA (A4), '
                'SCL (A5), la alimentación y los pull-ups. La placa sigue '
                'emitiendo, pero el ángulo queda congelado y las mediciones '
                'de posición no son válidas.']

    def notas_despues(self, df):
        """Lo del sensor que importa menos que un período perdido."""
        notas = []

        sovr = df.attrs.get('sovr') or 0
        if sovr:
            notas.append(
                f'{sovr} desborde(s) del sensor: una transferencia de I2C no había '
                f'terminado cuando vencía la muestra siguiente, así que esa muestra '
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
        raise CtrlLinkError(f'no hay ningún canal llamado {name!r}')

    # ------------------------------------------------------ autodescripción
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

    def zero_current(self, seconds=0.6, canales=None):
        """Toma la corriente que se mida ahora como el cero. Devuelve `cur_zero`.

        Con el actuador abierto no circula corriente, así que lo que marque el
        sensor es su offset: el suyo propio, más la tolerancia de su alimentación y
        la de cualquier divisor que haya en el medio. Deja el motor en reposo, que
        es la condición bajo la cual la medición significa algo.

        `canales` tiene que ser el mismo que va a usar el ensayo, y conviene que sean
        todos. La lectura depende de cuántos se emitan, y bastante: medido en reposo, con
        un solo canal el valor se sortea entre dos niveles separados 26 mA al arrancar
        cada captura, y con los cinco esa dispersión baja a 3,6 mA. Una fila más larga
        sacude la alineación de las conversiones lo suficiente como para que los dos
        estados se promedien adentro de la captura; por qué hay dos, en
        Sense/RowAdc.h. Por omisión, todos, que es lo que emite `capture()` si no se le
        pide otra cosa.
        """
        self.cur_zero = round(self._reposo_de_corriente(seconds, canales))
        return self.cur_zero

    def _reposo_de_corriente(self, seconds=0.6, canales=None):
        """La cuenta del ADC con el actuador abierto, promediada.

        `i` se publica como `s * (adc - cur_zero)`, con `s` el signo de `cur_inv`,
        así que la cuenta cruda se reconstruye deshaciendo las dos cosas. No
        importa si el eje sigue girando por inercia: con el actuador abierto la
        fuerza contraelectromotriz no encuentra camino y no circula corriente.
        """
        self.rest()
        df = self.capture(seconds, warn=False, canales=canales)
        return self.cur_zero + self._signo_corriente() * _sin_arranque(df)['i'].mean() \
            / self._ma_por_cuenta()

    def _signo_corriente(self):
        """-1 si la placa publica la corriente con el signo invertido (`cur_inv`)."""
        return -1 if 'cur_inv' in self.link._params and self.cur_inv else 1

    def _ma_por_cuenta(self):
        """Cuántos mA es una cuenta cruda del conversor.

        No es lo mismo que la escala del canal: `i` puede publicar fracciones de
        cuenta --`Banco` publica dieciseisavos-- y `cur_frac` dice cuántos bits
        fraccionarios son. El cero, los rieles y el margen del conversor se miden en
        cuentas enteras, así que todo lo que hable de ellos pasa por acá. Sin
        `cur_frac` el canal está en cuentas, que es lo que hace `ControlDemo`.
        """
        frac = int(self.cur_frac) if 'cur_frac' in self.link._params else 0
        return self.channel('i').scale * (1 << frac)

    def _tiron(self, u, seconds=0.4):
        """`u` sobre el actuador por un instante, desde el eje quieto.

        Devuelve (vueltas, captura), con `t = 0` en el comando. Espera primero a que
        el eje pare: con el actuador abierto el motor no frena, y midiendo enseguida
        lo que se mide es el giro anterior. Las vueltas van con signo.
        """
        ensayo.esperar_quieto(self, limite=15.0)
        df = self.step('ctl_uff', u, pre=0.05, post=seconds, back=0, warn=False)
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
                say('  Atención: no se declaró bidir, así que el actuador queda en un solo '
                    'cuadrante y un comando negativo sale como cero. Conectar con '
                    'sync_board(bidir=True) o bidir=False.')
            else:
                self.mot_bidir = int(bool(bidir))

        guardado = leer_cableado(cableado) or {}

        if 'cur_div' in params:
            if 'cur_div' in guardado:
                self.cur_div = int(guardado['cur_div'])
                say(f'  corriente contra la alimentación del sensor: divisor de A1 con relación '
                    f'{self.cur_div / 10000:.4f} ({Path(cableado).name})')
                # Un divisor suelto no da error: A1 queda saturado o en cero y la
                # corriente sale de un cociente sin sentido, con un cero corrido.
                time.sleep(0.2)
                a1 = int(self.cur_a1)
                if not 0.05 * (_ADC_FULL + 1) < a1 < 0.95 * (_ADC_FULL + 1):
                    say(f'  Atención: A1 lee {a1} cuentas, fuera de lo que da un divisor de 5 V: está '
                        f'suelto o mal conectado, y la corriente no es válida. Revisar las dos '
                        f'resistencias del divisor.')
            else:
                say('  corriente contra AVCC: no hay divisor de A1 declarado. Si la placa '
                    'funciona a 3,3 V, poner el divisor de 5 V a A1 y declararlo con '
                    'dev.declarar_divisor(arriba_ohm, abajo_ohm).')

        # Cada signo por separado, y sólo los que el sketch declare: uno que cierre un
        # lazo de ángulo puede tener `cur_inv` y no `ang_inv`.
        signos = [n for n in ('ang_inv', 'cur_inv') if n in params]
        if signos:
            if not all(n in guardado for n in signos):
                say(f'  sin signos medidos ({Path(cableado).name} no los tiene): el '
                    f'ángulo y la corriente van con el signo del cableado. Correr '
                    f'dev.bringup().')
            else:
                for n in signos:
                    self.set(n, int(guardado[n]))
                say('  signos del banco: '
                    + ', '.join(f'{n} = {int(guardado[n])}' for n in signos)
                    + f' ({Path(cableado).name})')

        # Contra un riel no hay un offset que medir sino una entrada al aire, y
        # "calibrarla" dejaría un canal que informa ceros perfectos. Eso lo explica
        # bringup(); acá sólo no se toca.
        if cero and 'cur_zero' in params:
            adc = self._reposo_de_corriente()
            if _ADC_RAIL <= adc <= _ADC_FULL - _ADC_RAIL:
                self.cur_zero = round(adc)

    def medir_divisor(self, guardar=True, say=print):
        """Mide la relación del divisor de A1 contra el reposo del sensor.

        El ACS712 es ratiométrico: con el actuador abierto reposa en la mitad de su
        alimentación. Así que, en cuentas del conversor, A0 = V5/2 y A1 = V5·k, y

            k = A1 / (2·A0)

        donde V5 --y la referencia del conversor-- se cancelan. Eso es lo que hace falta
        saber: cuánto vale V5 no interviene en ninguna parte, y medirlo desde adentro del
        micro no se puede (5 V está por encima de AVCC, y el único camino es el divisor
        mismo, que es lo que se quiere medir).

        En términos de lo que la placa publica, con `c` el reposo en cuentas
        equivalentes, es `k = 2000 · cur_div / c`: la cuenta no depende del `cur_div`
        que hubiera puesto, así que una sola medición alcanza y no hay que iterar.
        Verificado en la placa arrancando de 2500, 2817 y 3200: da 2794, 2790 y 2791.

        Qué se gana y qué se pierde. Se gana no depender de la tolerancia de las
        resistencias, que con las del 5 % deja la escala abierta ±7 %; medido en el
        banco, esto se repite dentro del 0,09 % --diez medidas seguidas, cinco resets,
        con una fila y con treinta y dos, emitiendo un canal y todos, y con el motor
        recién girado--. Se pierde que ahora la escala depende de que el sensor repose
        exactamente en la mitad de su alimentación, que es otra hipótesis y tampoco está
        verificada con un multímetro. Que los dos caminos coincidan dentro del 0,9 %
        --2792 medido contra 2817 declarado-- es lo que dice que ninguno de los dos está
        groseramente mal.

        Requiere el eje quieto y el actuador abierto, como `zero_current()`. Devuelve
        `cur_div`.
        """
        if not self.cur_div:
            raise CtrlLinkError(
                'no hay divisor declarado, así que A1 no se está leyendo y no hay nada '
                'que medir. Poner uno cualquiera con dev.cur_div = 2817 --el valor no '
                'importa, la medición no depende de él-- o declarar las resistencias '
                'con dev.declarar_divisor(arriba_ohm, abajo_ohm).')

        ensayo.esperar_quieto(self, limite=15.0)
        antes = int(self.cur_div)
        reposo = self._reposo_de_corriente()
        if reposo <= 0:
            raise CtrlLinkError(
                f'el sensor reposa en {reposo:.0f} cuentas equivalentes: contra un riel '
                f'del conversor no hay nada que medir. Revisar A0 y el divisor de A1.')

        medido = int(round(2000.0 * antes / reposo))
        self.cur_div = medido
        self.zero_current()

        say(f'divisor de A1 medido contra el reposo del sensor: relación '
            f'{medido / 10000:.4f} ({medido}), {medido / antes - 1:+.1%} de lo que '
            f'estaba puesto; el sensor queda en {self.cur_zero} cuentas equivalentes '
            f'(2000 es la mitad de su alimentación)')
        if not 1000 <= medido <= 6000:
            self.cur_div = antes
            raise CtrlLinkError(
                f'la relación medida, {medido / 10000:.4f}, no es la de un divisor que '
                f'lleve 5 V a la entrada de este conversor. O el eje no estaba quieto, o '
                f'A0 no es la salida del sensor, o A1 no cuelga de los 5 V. Queda '
                f'{antes / 10000:.4f}, el de antes.')

        if guardar:
            _actualizar_cableado(cur_div=medido, cur_div_medido=True)
        return medido

    def declarar_divisor(self, arriba_ohm, abajo_ohm, guardar=True):
        """Declara el divisor de los 5 V del sensor en A1, y mide la corriente contra ellos.

        Hace falta cuando la placa no funciona a 5 V --el clon del banco va a 3,3 V--:
        el ACS712 reposa en la mitad de su alimentación, y medido contra la del micro
        la lectura se corre con cualquier consumo de la placa. Con A1 leyendo una
        fracción fija de los 5 V del sensor, la placa usa el cociente A0/A1 y las dos
        alimentaciones se cancelan. Ver Sense/SupplyRatio.h.

        `arriba_ohm` va del pin 5V a A1 y `abajo_ohm` de A1 a GND. La escala de la
        corriente depende de su relación, así que la tolerancia de las resistencias
        entra en ella. Con el eje quieto, verifica que A1 lea algo y que el sensor
        repose cerca de la mitad de su alimentación, mide el cero de nuevo y lo guarda
        en `CABLEADO`. Devuelve la relación en diezmilésimas.
        """
        relacion = abajo_ohm / (arriba_ohm + abajo_ohm)
        self.cur_div = int(round(relacion * 10000))
        ensayo.esperar_quieto(self, limite=15.0)
        self.zero_current()

        a1 = int(self.cur_a1)
        fondo = _ADC_FULL + 1
        print(f'divisor {arriba_ohm:g} / {abajo_ohm:g} ohm: relación {relacion:.4f}; '
              f'A1 lee {a1} cuentas ({a1 / fondo:.0%} de la escala)')
        if a1 < 0.05 * fondo or a1 > 0.97 * fondo:
            self.cur_div = 0
            raise RuntimeError(
                f'A1 lee {a1} cuentas: el divisor no está conectado, o la salida supera la '
                f'alimentación del micro. La placa vuelve a medir contra AVCC; revisar el '
                f'cableado y volver a declararlo.')

        # El sensor reposa en la mitad de su alimentación, que en cuentas equivalentes
        # son 2000. Lejos de eso, la relación declarada no es la del divisor puesto: la
        # misma cuenta, dada vuelta, es la relación que mide `medir_divisor()`.
        desvio = self.cur_zero / 2000 - 1
        medido = 2000 * self.cur_div / max(self.cur_zero, 1)
        print(f'el sensor reposa en {self.cur_zero} cuentas equivalentes: '
              f'{desvio:+.1%} de la mitad de su alimentación, o sea que medido contra '
              f'ese reposo el divisor da {medido / 10000:.4f} ({medido / self.cur_div - 1:+.1%})')
        if abs(desvio) > 0.08:
            print('  Atención: más de un 8 %. O la relación declarada no es la del divisor, o el '
                  'sensor no reposa en la mitad: revisar las resistencias.')
        print('  dev.medir_divisor() usa ese reposo en lugar de las resistencias, y no depende '
              'de la tolerancia de ellas.')

        if guardar:
            _actualizar_cableado(cur_div=self.cur_div)
            _borrar_del_cableado('cur_sag', 'cur_sagc', 'cur_sagd', 'cur_red')
        return self.cur_div

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
               f'peor retardo de atención {late} us de {df.attrs["dt_us"]:.0f} us '
               f'({margin:.0%})')

        # 2. El sensor, antes que el imán: si el AS5600 no contesta en el bus, lo
        #    que diga su registro del imán no significa nada.
        present = bool(df.attrs.get('spres', 1))
        report('sensor', present,
               'contesta en el bus' if present else
               'no contesta -- revisar SDA (A4), SCL (A5), alimentación y pull-ups')

        # 3. El imán, tal como lo ve el propio AS5600. El AGC en números, porque es
        #    la compuerta de la calibración: contra un borde el imán está a la
        #    distancia equivocada y ninguna tabla arregla nada.
        status = df.attrs.get('mstat', 0)
        agc = df.attrs.get('agc')
        if not present:
            report('imán', None, 'no se puede evaluar sin el sensor')
        elif not status & _MAGNET_PRESENT:
            report('imán', False, 'no se detecta -- ¿está montado sobre el chip?')
        elif status & _MAGNET_WEAK:
            report('imán', False, 'muy débil (AGC al máximo) -- acercarlo')
        elif status & _MAGNET_STRONG:
            report('imán', False, 'muy fuerte (AGC al mínimo) -- alejarlo')
        elif agc is None:
            report('imán', True, 'detectado, AGC en rango')
        else:
            comodo = 32 <= agc <= 224
            report('imán', True if comodo else None,
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
        report('ángulo', None if spread < 0.5 else True,
               f'{df["y_uw"].iloc[-1]:.1f} grados, se movió {spread:.2f} grados '
               f'en el segundo' + ('  (girar el imán para verlo seguir)'
                                   if spread < 0.5 else ''))

        # 5. El cero de la medición de corriente, que acá se mide y se calibra.
        #    Una entrada al aire termina contra un riel del ADC: eso es una
        #    ausencia, no un offset, y calibrarla dejaría un canal que informa ceros
        #    perfectos sin haber medido nada. Y un reposo pegado a un extremo no
        #    deja lugar para medir, aunque no llegue al riel.
        lsb = self.channel('i').scale       # mA por unidad publicada de `i`
        cuenta = self._ma_por_cuenta()      # mA por cuenta del conversor
        adc = self.cur_zero + df['i'].mean() / cuenta
        sensed = _ADC_RAIL <= adc <= (_ADC_FULL - _ADC_RAIL)
        rest_ma = noise = 0.0

        if not sensed:
            report('cero de i', None,
                   f'entrada contra el riel del ADC ({adc:.0f} de {_ADC_FULL}): no '
                   f'parece haber nada conectado en A0')
        else:
            up, down = _ADC_FULL - adc, adc
            if 'cur_div' in self.link._params and not self.cur_div and \
                    abs(adc / (_ADC_FULL + 1) - 0.5) > 0.1:
                report('referencia de i', None,
                       f'el sensor reposa en el {adc / (_ADC_FULL + 1):.0%} de la escala y '
                       f'no en la mitad: la placa probablemente no funciona a 5 V. Poner el '
                       f'divisor de 5 V a A1 y dev.declarar_divisor()')
            report('cero de i', min(up, down) >= _ADC_HEADROOM,
                   f'{adc:.0f} de {_ADC_FULL}, margen +{up * cuenta / 1000:.1f} A / '
                   f'-{down * cuenta / 1000:.1f} A'
                   + ('' if min(up, down) >= _ADC_HEADROOM else
                      '  -- el reposo está muy cerca del tope: sin lugar para medir'))

            # Con el eje quieto, aunque con el actuador abierto no debería importar:
            # si importa, es algo del montaje que conviene ver en este residuo.
            ensayo.esperar_quieto(self, limite=15.0)
            self.zero_current()
            zeroed  = _sin_arranque(self.capture(0.6, warn=False))
            rest_ma = zeroed['i'].mean()
            noise   = zeroed['i'].std()

            # Un canal demasiado quieto es tan sospechoso como uno ruidoso: un ruido
            # de cero exacto es una señal más chica que el escalón del canal. El
            # umbral va contra ese escalón y no contra la cuenta del conversor:
            # publicando fracciones de cuenta, una señal por debajo de una cuenta
            # deja de ser invisible, que es para lo que se publica así.
            report('calibración de i',
                   abs(rest_ma) < _RESIDUO_MAX * cuenta and noise > 0.1 * lsb,
                   f'cur_zero = {self.cur_zero}, {cuenta:.1f} mA por cuenta del '
                   f'conversor y {lsb:.2f} mA por unidad de i, quedan '
                   f'{rest_ma:+.1f} mA en reposo, ruido {noise:.1f} mA RMS'
                   + ('' if noise > 0.1 * lsb else
                      '  -- sin dither: la señal no llega a un escalón del canal'))

            # La relación del divisor, medida contra el mismo reposo que se acaba de
            # calibrar: no cuesta ninguna captura más. No se la aplica sola --cambiar
            # la escala de un canal es una decisión de quien arma el banco-- pero sí se
            # dice cuánto difiere de lo declarado. Ver medir_divisor().
            if self.cur_div and self.cur_zero > 0:
                medido = 2000 * int(self.cur_div) / int(self.cur_zero)
                aparta = medido / int(self.cur_div) - 1
                report('divisor de i', None if abs(aparta) > 0.03 else True,
                       f'declarado {int(self.cur_div) / 10000:.4f}, medido contra el '
                       f'reposo del sensor {medido / 10000:.4f} ({aparta:+.1%})'
                       + ('' if abs(aparta) <= 0.03 else
                          '  -- más de un 3 %: revisar las resistencias, o tomar el '
                          'medido con dev.medir_divisor()'))

        # 6. El actuador, y con él toda la cadena: un comando que sale, movimiento y
        #    corriente que vuelven. Sólo se usa la evidencia cuyo sensor está.
        if not motor:
            report('motor', None, 'omitido (motor=False)')
        else:
            self._verificar_actuador(report, u, present, sensed, rest_ma, noise)

        bad = results.count(False)
        print(f'\n{"todas las verificaciones pasaron" if not bad else f"FALLARON {bad} verificación(es)"}')
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

        Sólo los signos que la placa declare: un sketch que cierre un lazo de ángulo
        puede tener `cur_inv` y no `ang_inv`.
        """
        bidir = bool(self.mot_bidir)
        nombres = [n for n in ('ang_inv', 'cur_inv') if n in self.link._params]
        antes = {n: int(self.get(n)) for n in nombres}

        def poner(valores):
            for n, v in valores.items():
                self.set(n, int(v))

        print(f'  accionando con u = +{u} y u = -{u}, 0,4 s cada uno y con el eje '
              f'quieto antes, dos veces ...')

        poner({n: 0 for n in nombres})
        self.mot_bidir = 1
        try:
            v_pos, df_pos = self._tiron(u)
            v_neg, _ = self._tiron(-u)
        except BaseException:
            poner(antes)
            raise
        finally:
            self.mot_bidir = int(bidir)

        post = df_pos[df_pos['t'] >= 0]
        pico = post['i'].abs().max()
        i_pos = _arranque_menos_final(df_pos)

        evidence = ([f'{abs(v_pos):.2f} vueltas'] if present else []) + \
                   ([f'{pico:.0f} mA de pico'] if sensed else [])
        gira = present and abs(v_pos) > _GIRO_MINIMO

        if not evidence:
            report('motor', None, 'no se puede evaluar: no hay sensor de ángulo ni '
                                  'medición de corriente')
        else:
            # Con el sensor de ángulo, que gire es la prueba. La corriente sola no
            # alcanza: sin divisor en A1, el PWM corre la lectura por la caída de AVCC
            # aunque la fuente del motor esté apagada, así que su pico no prueba nada.
            # Sin sensor de ángulo, el arranque contra el final, que cancela esa caída.
            arranca = sensed and abs(i_pos) > abs(rest_ma) + 5 * noise
            report('motor', gira if present else arranca, ', '.join(evidence))

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
            report('actuador', None, 'el eje no giró con +u: no se puede verificar bidir')
        elif abs(v_neg) <= _GIRO_MINIMO:
            report('actuador', False,
                   f'con -{u} el eje no gira ({v_neg:+.2f} vueltas): '
                   + ('revisar IN1 (6) e IN2 (7)' if bidir else
                      'un transistor tendría que empujar igual que con +u'))
        else:
            invierte = (v_pos > 0) != (v_neg > 0)
            tipo = ('invierte el giro: puente en H (B)' if invierte else
                    'empuja para el mismo lado: un solo cuadrante (B′)')
            if invierte == bidir:
                report('actuador', True, f'-u {tipo}, como declara bidir = {bidir}')
            else:
                report('actuador', False,
                       f'-u {tipo}, pero se conectó con bidir = {bidir}. '
                       f'Conectar con sync_board(bidir={invierte}).')

        # ---- los signos
        if not gira:
            poner(antes)
            report('signos', None, 'sin giro no se pueden medir: quedan los de antes')
            return

        medidos = {}
        if 'ang_inv' in antes:
            medidos['ang_inv'] = int(v_pos < 0)

        # No la media del tirón, que mezcla la corriente del motor con lo que corre
        # la lectura el propio PWM --la caída de AVCC, si no hay divisor en A1, son
        # cientos de mA--. El arranque contra el final: los
        # dos con el mismo comando, así que el artefacto es el mismo, pero al
        # arrancar el motor pide mucha más corriente que cuando ya giró.
        umbral = max(3 * self._ma_por_cuenta(),
                     5 * noise * (2 / max(len(df_pos) / 4, 1)) ** 0.5)
        mide_i = sensed and abs(i_pos) > umbral
        if 'cur_inv' in antes:
            medidos['cur_inv'] = int(i_pos < 0) if mide_i else antes['cur_inv']

        poner(medidos)
        invierte = (v_pos > 0) != (v_neg > 0) if abs(v_neg) > _GIRO_MINIMO else None
        self._guardar_cableado(medidos, mide_i, invierte)

        v_pos, df_pos = self._tiron(u)
        v_neg, df_neg = self._tiron(-u)
        i_pos = _arranque_menos_final(df_pos)

        # Sin `ang_inv` la placa no invierte el signo del ángulo, así que sólo se exige que gire.
        sube = v_pos > _GIRO_MINIMO if 'ang_inv' in medidos else abs(v_pos) > _GIRO_MINIMO
        ok = sube and (not mide_i or 'cur_inv' not in medidos or i_pos > 0)
        report('signos', ok,
               ', '.join(f'{n} = {v}' for n, v in medidos.items())
               + ('' if mide_i else ' (la corriente no se despega del cero: sin medir)')
               + f'; +u da {v_pos:+.2f} vueltas, y la corriente arranca {i_pos:+.0f} mA '
               f'por encima de donde termina. '
               f'Guardados en {CABLEADO.name}')

        u_neg = df_neg[df_neg['t'] >= 0.01]['u'].min()
        if bidir:
            gira_al_reves = (v_neg < -_GIRO_MINIMO if 'ang_inv' in medidos else
                             (v_neg > 0) != (v_pos > 0) and abs(v_neg) > _GIRO_MINIMO)
            report('-u', gira_al_reves and u_neg == -u,
                   f'sale u = {u_neg:+.0f} y el eje da {v_neg:+.2f} vueltas')
        else:
            report('-u', abs(v_neg) <= _GIRO_MINIMO and u_neg == 0,
                   f'sale u = {u_neg:+.0f} (recortado a cero) y el eje da '
                   f'{v_neg:+.2f} vueltas')

    def _guardar_cableado(self, signos, cur_medido, invierte):
        """Guarda los signos medidos en `CABLEADO`, sin tocar lo demás que haya."""
        _actualizar_cableado(
            **{n: int(v) for n, v in signos.items()},
            cur_inv_medido=bool(cur_medido),
            invierte_con_u_negativo=None if invierte is None else bool(invierte),
            placa=self.info)


def _arranque_menos_final(df):
    """La corriente al arrancar menos la del final de un tirón con `t = 0` en el comando.

    Con el mismo comando en las dos ventanas, lo que el PWM le hace a la lectura se
    cancela, y queda la corriente del motor: grande al arrancar, chica cuando ya
    gira. Positiva si el canal tiene el signo bien.
    """
    t, i = df['t'].to_numpy(), df['i'].to_numpy()
    fin = t.max()
    arranque = i[(t > 0.01) & (t < 0.10)]
    final = i[t > fin - 0.10]
    if not len(arranque) or not len(final):
        return 0.0
    return float(arranque.mean() - final.mean())


def _actualizar_cableado(**campos):
    """Agrega o reemplaza `campos` en `CABLEADO`, con la fecha."""
    from datetime import datetime

    datos = leer_cableado() or {}
    datos.update(campos)
    datos['medido'] = datetime.now().isoformat(timespec='seconds')
    CABLEADO.write_text(json.dumps(datos, indent=1, ensure_ascii=False), encoding='utf-8')


def _borrar_del_cableado(*claves):
    """Saca `claves` de `CABLEADO`, si están."""
    datos = leer_cableado()
    if datos is None or not any(c in datos for c in claves):
        return
    for c in claves:
        datos.pop(c, None)
    CABLEADO.write_text(json.dumps(datos, indent=1, ensure_ascii=False), encoding='utf-8')


def sync_board(port=None, force_compile=False, force_upload=False, verbose=True,
               sketch=None, bidir=None):
    """Pone al día la placa y el enlace, y reconecta. Devuelve un Bench.

    Compila sólo cuando algún archivo fuente cambió efectivamente, carga sólo cuando
    el binario resultante difiere del que este puerto recibió por última vez, y
    siempre reabre el enlace, lo que resetea la placa, así que el sketch arranca
    desde sus valores por omisión haya hecho falta o no grabar. Ese reset es el
    motivo de llamarlo al principio de cada celda.

    `bidir` es qué actuador tiene el banco: True para un puente en H, False para un
    transistor. Junto con los signos medidos y el cero de la corriente se le carga
    a la placa después del reset; ver `Bench.configurar()`.

    force_compile y force_upload saltean cada uno su propia verificación.

    `sketch` elige qué se graba; ver `placa.poner_al_dia()`.
    """
    global _link

    def say(message):
        if verbose:
            print(message)

    def soltar_puerto():
        global _link
        if _link is not None:
            _link.close()
            _link = None

    port, notes = placa.poner_al_dia(port, force_compile, force_upload, sketch, say,
                                     antes_de_cargar=soltar_puerto)

    if _link is not None:
        _link.close()
        _link = None

    try:
        _link = Bench(port)
    except serial.SerialException as exc:
        raise CtrlLinkError(
            f'no se pudo abrir {port}: {exc}\n'
            f'Otro programa tiene el puerto abierto: el monitor serie del IDE de '
            f'Arduino, o un kernel de una sesión anterior. Cerrarlo, o reiniciar este '
            f'kernel, y volver a correr esta celda. Un puerto serie sólo puede estar '
            f'abierto por un programa a la vez.'
        ) from None

    say(f'{port}: {_link.info}' + (f'  ({", ".join(notes)})' if notes else ''))
    _link.configurar(bidir=bidir, say=say)
    return _link
