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

__all__ = ['sync_board', 'sync_board_cal', 'Bench', 'CtrlLinkError',
           'CALIBRACION',
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

# `mode` elige el controlador, `target` elige la realimentación sobre la que
# cierra.
MODE_OPEN, MODE_PID, MODE_RAMP = 0, 1, 2
POSITION,  CURRENT             = 0, 1

# Bits del registro STATUS del AS5600.
_MAGNET_STRONG, _MAGNET_WEAK, _MAGNET_PRESENT = 0x08, 0x10, 0x20

# El reloj de la placa. La frecuencia del PWM se guarda como el TOP del Timer1,
# que es por lo que cuenta el hardware, así que la conversión a Hz pasa por acá.
_F_CPU = 16_000_000

# El ADC del UNO da 10 bits sobre su referencia. Dónde tiene que reposar el sensor
# depende de cuál sea y de cómo esté alimentado, así que acá no se juzga el valor:
# se juzga que quede fuera de los rieles --contra un riel no hay una corriente
# grande sino una entrada al aire-- y que sobre margen hacia arriba para que una
# corriente tenga adónde crecer.
_ADC_FULL     = 1023
_ADC_RAIL     = 20   # a menos de esto de cualquiera de los dos extremos
_ADC_HEADROOM = 100  # cuentas de margen que se le piden al reposo

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
        self.pwmtop = top
        return _F_CPU / (2 * top)

    @property
    def pwm_hz(self):
        """La frecuencia del PWM del puente, en Hz, tal como quedó en la placa."""
        return _F_CPU / (2 * self.pwmtop)

    def zero(self):
        """Toma la posición actual del eje como cero.

        `y` se lee como `offset - counts` con vuelta, así que bajar `offset` en el
        `y` actual pone `offset` sobre la cuenta actual y `y` en cero.
        """
        self.offset = (self.offset - self.y) % 4096
        self.y_uw = 0

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
        """
        self.rest()
        df = self.capture(seconds, warn=False)
        self.izero = round(self.izero + df['i'].mean() / self.channel('i').scale)
        return self.izero

    def rest(self):
        """Lazo abierto, comando en cero. Donde tendría que terminar todo experimento.

        Con el comando en cero el sketch deja ENA en bajo, así que el puente queda
        abierto y el motor en punto muerto: no frena el eje, sólo deja de
        empujarlo.
        """
        self.mode = MODE_OPEN
        self.uff = 0

    def spin(self, u, seconds=0.4, espera=6.0, quieto=5.0):
        """Lazo abierto con `u` sobre el puente por un instante, y de vuelta a reposo.

        Devuelve (vueltas, mA de pico). Las vueltas van con signo, que es lo que
        hace verificable el sentido; la corriente no, porque que el ACS712 vea o no
        el signo depende de en qué parte del circuito esté insertado, y para el
        pico da igual.

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
        self.uff = u
        df = self.capture(seconds, warn=False)
        self.rest()

        vueltas = (df['y_uw'].iloc[-1] - df['y_uw'].iloc[0]) / 360.0

        # Si no llegó a parar, el número que sigue no significa lo que dice, y es
        # mejor que quien mira lo sepa que un signo inventado con cara de dato.
        if arrastre >= quieto:
            print(f'  OJO: el eje seguia girando a {arrastre:.0f} grados/s al empezar '
                  f'esta medicion; el sentido que informa puede ser el de la inercia')

        return vueltas, df['i'].abs().max()

    # ------------------------------------------------------- puesta en marcha

    def bringup(self, motor=True, u=120):
        """Verifica el hardware, un subsistema por vez.

        Cada línea es algo que puede estar mal por su cuenta: el enlace, el lazo,
        el imán, el bus I2C, el cero de la medición de corriente --que de paso se
        calibra--, el actuador y el sentido en el que empuja. Conviene
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
        adc = self.izero + df['i'].mean() / lsb
        sensed = _ADC_RAIL <= adc <= (_ADC_FULL - _ADC_RAIL)

        if not sensed:
            rest_ma = 0.0

            # Una lectura por encima del fondo de escala no es un sensor mal
            # puesto: es un conversor que no tiene los bits que este código le
            # supone. El LGT8F328P --el clon que también arranca con el reloj
            # dividido; ver BoardStart.h-- trae un ADC de 12 bits en lugar de los
            # 10 del ATmega328P, así que informa cuatro veces más cuentas por la
            # misma tensión. Sin nada conectado en A0 eso no cambia nada, pero con
            # un sensor de corriente los amperes saldrían cuatro veces grandes, y
            # eso es de las cosas que uno prefiere leer antes que descubrir.
            if adc > _ADC_FULL:
                report('cero de i', None,
                       f'la entrada informa {adc:.0f} cuentas y un ADC de 10 bits '
                       f'llega a {_ADC_FULL}: o no hay nada conectado en A0 --la '
                       f'entrada al aire termina contra un riel-- o esta placa '
                       f'tiene el ADC de 12 bits del LGT8F328P, y entonces la '
                       f'escala de corriente sale multiplicada por 4')
            else:
                report('cero de i', None,
                       f'entrada contra el riel del ADC ({adc:.0f} de {_ADC_FULL}): '
                       f'no parece haber nada conectado en A0')
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
                   f'izero = {self.izero}, {lsb:.2f} mA por cuenta, quedan '
                   f'{rest_ma:+.1f} mA en reposo (ruido {noise / lsb:.2f} cuentas)'
                   + ('' if noise > 0.1 * lsb else
                      '  -- sin dither: la senal no llega a un escalon del ADC'))

        # 6. El actuador, y con él toda la cadena: un comando que sale, movimiento
        #    y corriente que vuelven. Hay dos evidencias posibles y cada una
        #    depende de su propio sensor, así que sólo se usa la que esté
        #    disponible: dar por bueno un motor porque la corriente se movió,
        #    cuando la entrada de corriente está al aire, es peor que no medir.
        if not motor:
            report('motor', None, 'omitido (motor=False)')
            report('sentido', None, 'omitido (motor=False)')
            report('polaridad', None, 'omitido (motor=False)')
        else:
            print(f'  accionando el motor con u = {u:+} durante 0,4 s, '
                  f'PWM a {self.pwm_hz / 1000:.1f} kHz ...')
            fwd, drawn = self.spin(u)

            evidence = ([f'{abs(fwd):.2f} vueltas'] if present else []) + \
                       ([f'{drawn:.0f} mA de pico'] if sensed else [])

            if not evidence:
                report('motor', None, 'no se puede evaluar: no hay sensor de '
                                      'angulo ni medicion de corriente')
            else:
                report('motor',
                       (present and abs(fwd) > 0.05) or
                       (sensed and drawn > abs(rest_ma) + 50),
                       ', '.join(evidence))

            # 7. El sentido, que es una verificación aparte porque falla aparte y
            #    en otro lado: IN1 e IN2 intercambiados, o uno de los dos sin
            #    conectar, dejan pasar todo lo anterior y recién se notan como un
            #    lazo cerrado que se escapa en vez de establecerse. La evidencia
            #    tiene que ser el angulo: la corriente mide lo mismo en los dos
            #    sentidos, asi que no distingue el caso.
            if not self.bidir:
                report('sentido', None, 'bidir = 0, el puente esta declarado de un '
                                        'solo cuadrante')
                report('polaridad', None, 'no se puede evaluar sin invertir')
            elif not present:
                report('sentido', None, 'no se puede evaluar sin el sensor de angulo')
                report('polaridad', None, 'no se puede evaluar sin el sensor de angulo')
            else:
                print(f'  y ahora con u = {-u:+} ...')
                rev, _ = self.spin(-u)
                reverses = fwd * rev < 0 and abs(rev) > 0.05

                # Las dos maneras de no invertir se arreglan en lugares distintos
                # y se distinguen en los datos, así que conviene no meterlas en el
                # mismo consejo. Si el eje gira para el mismo lado con las dos
                # polaridades, el puente sí acciona en los dos sentidos y lo que
                # está mal es qué entrada va a qué pin. Si con el comando negativo
                # no se mueve nada, el puente no tiene el segundo cuadrante: no hay
                # nada que arreglar en el cableado, hay que decírselo al lazo para
                # que recorte en cero y el anti-windup se entere.
                if reverses:
                    pista = ''
                elif abs(rev) <= 0.05:
                    pista = ('  -- con el comando negativo no se movio: si el puente '
                             'es de un solo cuadrante, poner dev.bidir = 0; si no, '
                             'revisar IN2 (7)')
                else:
                    pista = ('  -- giro para el mismo lado con las dos polaridades: '
                             'revisar IN1 (6) e IN2 (7)')

                report('sentido', reverses,
                       f'{fwd:+.2f} vueltas con u = {u:+}, {rev:+.2f} con u = {-u:+}'
                       + pista)

                # 8. Y la polaridad, que es distinta de que el puente invierta: un
                #    comando positivo tiene que hacer *subir* el angulo medido. Si
                #    lo hace bajar, el lazo de posicion realimenta en positivo y se
                #    escapa con una referencia de cualquier signo, asi que probando
                #    no se descubre. Depende de dos cables --los del motor en el
                #    puente, y el sentido en que el iman mira al sensor-- y `uinvert`
                #    es el que los reconcilia.
                if not reverses:
                    report('polaridad', None,
                           'no se puede evaluar mientras el puente no invierta')
                else:
                    ok = fwd > 0
                    report('polaridad', ok,
                           f'un u positivo hace {"subir" if ok else "BAJAR"} el '
                           f'angulo, con uinvert = {self.uinvert}'
                           + ('' if ok else
                              f'  -- poner uinvert = {0 if self.uinvert else 1}, '
                              f'o dar vuelta los dos cables del motor'))

        bad = results.count(False)
        print(f'\n{"todas las verificaciones pasaron" if not bad else f"FALLARON {bad} verificacion(es)"}')
        return not bad


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
