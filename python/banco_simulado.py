"""Un banco de mentira, para poder dar la clase sin la placa.

Expone lo mismo que `Bench` --los parámetros como atributos, `capture()`,
`step()`, `bringup()`-- pero las capturas salen de un modelo en lugar de un
motor. Los notebooks no se enteran: corren el mismo código en los dos casos, y lo
único que cambia es de dónde vienen las filas.

El modelo sigue al banco de verdad, no al de un libro, y eso incluye lo
incómodo. El actuador es un transistor a masa con su diodo de rueda libre:
empuja y no frena, un comando negativo empuja para el mismo lado, y con el
comando en cero el eje sigue por inercia hasta que lo para el rozamiento, que
tarda segundos. El motor no se reinicia entre capturas --sigue girando mientras
la computadora hace otra cosa, igual que el de verdad--, así que un ensayo que no
espere a que el eje pare mide la cola del anterior.

Es de mentira y lo dice. Tiene adentro un error de sensor y un motor que alguien
eligió, así que "descubrirlos" no prueba nada sobre ningún banco; lo que prueba
es que el procedimiento encuentra lo que hay que encontrar. Cuando el banco
está, se usa el banco.
"""
import math
import time
from collections import namedtuple

import numpy as np
import pandas as pd

import catalogo

CUENTAS = 4096
GRADOS_POR_CUENTA = 360.0 / CUENTAS

# El error de ángulo que este banco imaginario tiene adentro, en cuentas. El
# primero es el imán descentrado, el segundo la inclinación. Son los órdenes que
# la literatura anticipa y las amplitudes son las de un montaje mediocre pero
# creíble: 6 cuentas son 0,53 grados.
ERROR_SENSOR = {1: (6.0, 0.7), 2: (2.5, -2.0)}

# Y la trampa: una ondulación de par del motor, enganchada al ángulo igual que el
# error del sensor, con la amplitud referida a 5 vueltas por segundo. Cae como
# omega^-2, que es lo que hace la inercia, y es lo único que distingue una cosa
# de la otra. Un notebook que la calibre como si fuera el sensor está mal.
RIPPLE_MOTOR = (3, 4.0, -1.0)   # (orden, cuentas a 5 rev/s, fase)

# El motor y su actuador. Un transistor a masa con diodo de rueda libre, a 1 kHz,
# resuelto período a período con la solución exacta del circuito RL: mientras el
# transistor conduce la armadura ve Vs, cuando se abre la corriente se descarga
# por el diodo contra Vd, y en ninguno de los dos tramos puede invertirse. Con
# L/R = 0,2 ms contra un período de 1 ms la corriente se extingue antes del final
# del período casi siempre --conducción discontinua--, y eso es lo que dobla la
# curva estática, acorta la constante de tiempo con la velocidad y deja un salto
# al 100 %, donde el transistor no se abre nunca.
#
# El rozamiento es Coulomb con un arranque más alto que el deslizamiento, más un
# viscoso. Los números se eligieron para parecerse a lo medido en el banco: zona
# muerta entre 7 y 10 %, ganancia de ~17 (rad/s)/% abajo y ~3 arriba, bajadas
# unas 1,8 veces más lentas que las subidas, y una corriente de decenas de mA que
# con el eje trabado no llega a un cuarto de ampere.
MOTOR = dict(
    Vs=5.0,         # V, la fuente
    Vd=0.7,         # V, el diodo de rueda libre
    R=24.0,         # ohm
    L=4.8e-3,       # H
    Ke=0.006,       # V/(rad/s), y N·m/A
    J=7.5e-7,       # kg·m², con el disco
    Tc=3.0e-5,      # N·m, Coulomb girando
    Ts=6.25e-5,     # N·m, lo que hace falta para despegar
    B=2.75e-7,      # N·m/(rad/s)
)
PWM_T = 1e-3        # s, 1 kHz

# El instante del período de PWM en el que el ADC toma la corriente, como
# fracción del período desde que el transistor conduce. PWM y muestreo están
# enganchados en fase, así que es siempre el mismo instante: el canal no mide el
# promedio sino una muestra de la forma de onda, y la diferencia depende del duty.
FASE_ADC = 0.35

# El retardo entre el eje y lo que informa el sensor: el filtro del AS5600 en 2x
# más el muestreo.
RETARDO_S = 0.5e-3

# La medición de corriente, con las escalas del sketch: 12 bits contra 5006 mV y
# un ACS712 de 185 mV/A. El UNO cuenta de a cuatro, porque su ADC es de 10 bits
# corrido dos lugares. El reposo no cae justo en media escala, y el ruido son los
# 91 mA RMS que se midieron en el banco.
MA_POR_CUENTA = 1000.0 * (5006.0 / 4096.0) / 185.0
REPOSO_I = 2048 + 57         # cuentas
RUIDO_I_MA = 91.0

_Canal = namedtuple('_Canal', 'name scale unit')

CANALES = {
    'y_raw': _Canal('y_raw', GRADOS_POR_CUENTA, 'deg'),
    'y_uw':  _Canal('y_uw',  GRADOS_POR_CUENTA, 'deg'),
    'u':     _Canal('u',     1.0,               'pwm'),
    'i':     _Canal('i',     MA_POR_CUENTA,     'mA'),
}

# Más que esto de reloj de pared entre dos llamadas no se integra: con el comando
# quieto, en veinte segundos el eje ya llegó a donde iba a llegar.
_PONERSE_AL_DIA_S = 20.0


class BancoSimulado:
    """Todo lo que los notebooks le piden a un banco."""

    def __init__(self, ruido=0.5, semilla=0, motor=None):
        self._rng = np.random.default_rng(semilla)
        self.ruido = ruido
        self.motor = dict(MOTOR, **(motor or {}))

        # Las perillas y las lecturas de la placa, con los mismos nombres. Las
        # lecturas son las de un banco sano, y están para que la tabla que muestra
        # `dev` sea la misma con el cable enchufado y sin él.
        self.ang_cal = 0
        self.cur_zero = 2048
        self.loop_div = 10
        self.dec = 1
        self.ang_present = 1
        self.ang_status = 0x20          # imán detectado, ni débil ni fuerte
        self.ang_agc = 128              # media escala: la distancia correcta
        self.ang_mag = 1800
        self.ang_busovr = self.ang_buserr = 0
        self.loop_late = 600
        self.loop_missed = 0

        self.lut = [0] * 64
        self.ang_lutw = 0xFFFFFFFF
        self._lutw_aplicado = 0xFFFFFFFF

        # El estado del motor, que sobrevive a las capturas.
        self._uff = 0
        self._w = 0.0                   # rad/s
        self._theta = 1234.0            # cuentas, desde el arranque
        self._i = 0.0                   # A
        self._reloj = time.monotonic()

        self.info = 'CtrlLink 1 Banco (SIMULADO) chans=4 dt_us=2000'
        self.simulado = True

    @property
    def dt(self):
        return self.loop_div / 5000.0

    # ------------------------------------------------------ los parámetros

    @property
    def ctl_uff(self):
        return self._uff

    @ctl_uff.setter
    def ctl_uff(self, valor):
        # Lo que pasó desde la última vez, con el comando que había.
        self._ponerse_al_dia()
        self._uff = int(round(min(255, max(-255, float(valor)))))

    def set(self, name, value, tries=3):
        if name == 'ang_lutw':
            value = int(value) & 0xFFFFFFFF
            self.ang_lutw = value

            if value != self._lutw_aplicado:
                self._lutw_aplicado = value
                i = value >> 16
                if i < 64:
                    v = value & 0xFFFF
                    self.lut[i] = v - 65536 if v > 32767 else v
            return value

        setattr(self, name, value)
        return getattr(self, name)

    def get(self, name):
        if name == 'ang_lutsum':
            a = b = 0
            for v in self.lut:
                for byte in ((v & 0xFF), ((v >> 8) & 0xFF)):
                    a = (a + byte) & 0xFF
                    b = (b + a) & 0xFF
            return (b << 8) | a
        return getattr(self, name)

    @property
    def params(self):
        return {n: self.get(n) for n in self._nombres()}

    def channel(self, name):
        try:
            return CANALES[name]
        except KeyError:
            raise KeyError(f'no hay ningun canal llamado {name!r}') from None

    @property
    def channels(self):
        return list(CANALES.values())

    def close(self):
        """No hay nada que cerrar; está para que una celda con `dev.close()` corra."""
        self.rest()

    def rest(self):
        self.ctl_uff = 0

    def zero_current(self, seconds=0.3):
        """Lo mismo que en el banco de verdad: el reposo de ahora pasa a ser el cero."""
        self.rest()
        df = self.capture(seconds, warn=False)
        self.cur_zero = round(self.cur_zero + df['i'].mean() / MA_POR_CUENTA)
        return self.cur_zero

    # ------------------------------------------------------ contarse solo

    def _nombres(self, filtro=''):
        tiene = {n for n in catalogo.nombres_conocidos() if hasattr(self, n)}
        tiene.add('ang_lutsum')
        return sorted(n for n in tiene if filtro in n)

    def _para_describir(self, filtro=''):
        canales = ([(c.name, c.scale, c.unit) for c in CANALES.values()]
                   if not filtro else ())
        resumen = (f'filas a {1 / self.dt:.0f} Hz, {len(CANALES)} canales de '
                   f'telemetría  -- NADA DE ESTO ES REAL: es el banco simulado')
        return self.info, resumen, self._nombres(filtro), self._valor_legible, canales

    def _valor_legible(self, nombre):
        valor = self.get(nombre)
        return f'{valor:.6g}' if isinstance(valor, float) else str(valor)

    def describe(self, filtro=''):
        """Las perillas y las lecturas, agrupadas, como texto. Ver Bench.describe()."""
        return catalogo.texto(*self._para_describir(filtro))

    def __repr__(self):
        return self.describe()

    def _repr_html_(self):
        return catalogo.html(*self._para_describir())

    # ------------------------------------------------------------ el motor

    def _periodo(self, D, w, i0):
        """Un período de PWM con la velocidad congelada.

        Devuelve (corriente al final, corriente media, corriente en FASE_ADC), en A.
        """
        p = self.motor
        R, T = p['R'], PWM_T
        tau = p['L'] / R
        emf = p['Ke'] * w
        ton = D * T
        tadc = FASE_ADC * T

        def tramo(i, a, t):
            # i(t) = a + (i - a) e^{-t/tau}, que se detiene en cero si va a cruzarlo:
            # ni el transistor ni el diodo dejan pasar corriente al revés.
            if a < 0.0:
                if i <= 0.0:
                    return 0.0, 0.0
                tz = tau * math.log((i - a) / (-a))
                if tz < t:
                    return 0.0, a * tz + (i - a) * tau * (1.0 - math.exp(-tz / tau))
            e = math.exp(-t / tau)
            return a + (i - a) * e, a * t + (i - a) * tau * (1.0 - e)

        a_on = (p['Vs'] - emf) / R
        a_off = (-p['Vd'] - emf) / R

        i, area = i0, 0.0
        if ton > 0.0:
            i, area = tramo(i0, a_on, ton)
        i_on = i
        if ton < T:
            i, ar = tramo(i, a_off, T - ton)
            area += ar

        muestra = (tramo(i0, a_on, tadc)[0] if tadc < ton
                   else tramo(i_on, a_off, tadc - ton)[0])
        return i, area / T, muestra

    def _integrar(self, comandos):
        """Avanza el motor un período de PWM por comando. Devuelve (w, theta, i_adc) por período."""
        p = self.motor
        J, T = p['J'], PWM_T
        w, theta, i = self._w, self._theta, self._i

        n = len(comandos)
        ws, thetas, muestras = np.empty(n), np.empty(n), np.empty(n)
        a_cuentas = CUENTAS / (2 * math.pi)

        for k in range(n):
            # El transistor sólo ve el módulo del comando: uno negativo empuja para
            # el mismo lado.
            D = min(255, abs(int(comandos[k]))) / 255.0
            i, i_med, muestra = self._periodo(D, w, i)
            par = p['Ke'] * i_med

            if w <= 0.0 and par <= p['Ts']:
                w_nueva = 0.0
            else:
                w_nueva = w + (par - p['Tc'] - p['B'] * w) / J * T
                if w_nueva < 0.0:
                    w_nueva = 0.0

            theta += 0.5 * (w + w_nueva) * T * a_cuentas
            w = w_nueva

            ws[k], thetas[k], muestras[k] = w, theta, muestra

        self._w, self._theta, self._i = w, theta, i
        return ws, thetas, muestras

    def _ponerse_al_dia(self):
        """Integra el reloj de pared que pasó desde la última vez, con el comando de ahora."""
        ahora = time.monotonic()
        transcurrido = min(ahora - self._reloj, _PONERSE_AL_DIA_S)
        self._reloj = ahora

        n = int(transcurrido / PWM_T)
        if n > 0:
            self._integrar(np.full(n, self._uff))

    def _error_sensor(self, theta, w):
        """El error de ángulo en cuentas: el del sensor, más el del motor."""
        e = np.zeros_like(theta)

        for k, (A, phi) in ERROR_SENSOR.items():
            e += A * np.sin(2*np.pi*k*theta/CUENTAS + phi)

        k, A, phi = RIPPLE_MOTOR

        # El piso de velocidad es del modelo, no de la física: omega^-2 diverge y
        # sin un piso el eje casi detenido tendría una ondulación de cientos de
        # cuentas, que no se parece a ningún banco.
        seguro = np.maximum(np.abs(w) / (2 * np.pi), 2.0)
        e += A * (5.0/seguro)**2 * np.sin(2*np.pi*k*theta/CUENTAS + phi)

        return e

    def _lut_lookup(self, crudo):
        """Igual que AngleLut::correction() en la placa, redondeo incluido."""
        crudo = np.asarray(crudo, dtype=np.int64) & (CUENTAS - 1)
        i = (crudo >> 6) & 63
        frac = crudo & 0x3F
        tabla = np.array(self.lut, dtype=np.int64)
        octavos = (tabla[i] * (64 - frac) + tabla[(i + 1) & 63] * frac) >> 6
        return (octavos + 4) >> 3

    # ---------------------------------------------------------- la captura

    def capture(self, duration, events=(), poll=0.005, warn=True):
        self._ponerse_al_dia()

        filas = int(round(float(duration) / self.dt))
        t = np.arange(filas) * self.dt
        n = max(1, int(math.ceil(filas * self.dt / PWM_T)) + 1)

        # El comando período a período, y el que queda puesto al terminar.
        comandos = np.full(n, self._uff, dtype=np.int64)
        t_periodo = np.arange(n) * PWM_T
        for retardo, nombre, valor in sorted(events, key=lambda e: float(e[0])):
            if nombre == 'ctl_uff':
                v = int(round(min(255, max(-255, float(valor)))))
                comandos[t_periodo >= float(retardo) - 1e-12] = v
                self._uff = v
            else:
                self.set(nombre, valor)

        theta0 = self._theta
        ws, thetas, muestras = self._integrar(comandos)
        self._reloj = time.monotonic()

        # Cada fila lee lo que el sensor ve en su instante, que es el eje de hace
        # RETARDO_S.
        t_fin = t_periodo + PWM_T
        tt = np.concatenate([[0.0], t_fin])
        theta = np.interp(t - RETARDO_S, tt, np.concatenate([[theta0], thetas]))
        w = np.interp(t, t_fin, ws)
        idx = np.minimum((t / PWM_T).astype(np.int64), n - 1)

        medido = theta + self._error_sensor(theta, w)
        medido = medido + self._rng.normal(0, self.ruido, filas)
        cuentas = np.rint(medido).astype(np.int64)

        crudo = np.mod(cuentas, CUENTAS)
        corregido = cuentas - (self._lut_lookup(crudo) if self.ang_cal else 0)

        # La corriente, como la ve el ADC del UNO: una muestra de la forma de onda,
        # con ruido, en escalones de cuatro cuentas.
        adc = (REPOSO_I + muestras[idx] * 185.0 / (5006.0 / 4096.0)
               + self._rng.normal(0, RUIDO_I_MA / MA_POR_CUENTA, filas))
        adc = np.clip(np.floor(adc / 4) * 4, 0, 4092)

        df = pd.DataFrame({
            't': t,
            'y_raw': crudo * GRADOS_POR_CUENTA,
            'y_uw': corregido * GRADOS_POR_CUENTA,
            'u': comandos[idx].astype(float),
            'i': (adc - self.cur_zero) * MA_POR_CUENTA,
        })

        df.attrs.update(tick=np.arange(filas, dtype=np.int64) * self.loop_div,
                        dec=self.dec, dt_us=self.dt*1e6, marks=[], notes=[],
                        gaps=0, missed=0, maxlate=600, sovr=0, serr=0,
                        spres=1, mstat=0x20, agc=128, mag=1800,
                        wall=float(duration), rows=filas, drops=0,
                        units={'y_raw': 'deg', 'y_uw': 'deg', 'u': 'pwm', 'i': 'mA'})

        return df

    def step(self, name, value, pre=0.1, post=0.9, back=None, warn=True):
        antes = self.get(name)
        df = self.capture(pre + post, events=[(pre, name, value)])
        df['t'] -= pre
        self.set(name, antes if back is None else back)
        return df

    # ---------------------------------------------------- puesta en marcha

    def bringup(self, motor=True, u=120):
        print(f'puesta en marcha: {self.info}')
        for etiqueta, detalle in [
                ('muestreo', '500 Hz reales contra 500 nominales, 0 perdidos'),
                ('sensor', 'contesta en el bus'),
                ('iman', 'detectado, AGC 128/255, campo 1800'),
                ('bus i2c', '0 errores de transferencia, 0 desbordes'),
                ('motor', f'gira con u={u}')]:
            print(f'  [   ok]  {etiqueta:<18}  {detalle}')
        print('\n  NADA DE ESTO ES REAL: es el banco simulado.')
        return True


def conseguir_banco(forzar_simulado=False, **kw):
    """El banco de verdad si está, y si no uno simulado, avisando cuál es.

    Un notebook que se muestra en clase no puede depender de que el cable esté
    enchufado, pero tampoco puede hacer pasar un modelo por una medición. Así
    que cae solo, y lo dice fuerte.
    """
    if not forzar_simulado:
        try:
            from bench import sync_board
            return sync_board()
        except Exception as exc:
            print(f'no hay banco: {type(exc).__name__}: {exc}')

    print('=' * 68)
    print('BANCO SIMULADO. Las capturas salen de un modelo, no de un motor.')
    print('El motor y el error de sensor los puso banco_simulado.py.')
    print('=' * 68)
    return BancoSimulado(**kw)
