"""Un banco de mentira, para poder dar la clase sin la placa.

Expone lo mismo que `Bench` --los parámetros como atributos, `capture()`,
`step()`, `bringup()`-- pero las capturas salen de un modelo en lugar de un
motor. Los notebooks no se enteran: corren el mismo código en los dos casos, y lo
único que cambia es de dónde vienen las filas.

Es de mentira y lo dice. El modelo tiene adentro un error de sensor que alguien
eligió, así que "descubrirlo" no prueba nada sobre ningún AS5600; lo que prueba
es que el procedimiento encuentra lo que hay que encontrar, que es exactamente lo
que uno quiere mostrar en un pizarrón. Cuando el banco está, se usa el banco.
"""
from collections import namedtuple

import numpy as np
import pandas as pd

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

# El motor de mentira, visto desde el PWM. La velocidad de régimen es lineal con
# el comando por encima de una zona muerta, y el transitorio es de primer orden:
# rápido al acelerar, lento al soltar porque frenando sólo actúa el rozamiento.
U_MUERTO   = 60.0    # cuentas: por debajo no arranca
W_MAX      = 12.0    # vueltas por segundo a fondo
TAU_SUBE   = 0.3     # s, acelerando
TAU_BAJA   = 6.0     # s, soltando
RETARDO    = 1       # muestras entre el eje y lo que informa el sensor

# La corriente sale de la ecuación eléctrica sobre esa velocidad: lo que la
# tensión de armadura le gana a la fuerza contraelectromotriz, sobre la
# resistencia. Un puente Darlington contra 5 V le deja al motor unos 3 V. KE es
# lo bastante chica como para que la corriente de régimen crezca con la
# velocidad: así el par que el motor entrega en régimen --que es todo
# rozamiento-- crece con ella, como en un motor de verdad.
V_MOTOR = 3.0        # V a fondo
R_MOTOR = 5.0        # ohm
KE      = 0.025      # V por rad/s

# Las escalas que el dispositivo de verdad declara en su tabla de canales. Los mA
# por cuenta son los del sketch: 12 bits contra 5006 mV y un ACS712 de 185 mV/A.
# El UNO cuenta de a cuatro, porque su ADC es de 10 bits corrido dos lugares.
_Canal = namedtuple('_Canal', 'name scale unit')

MA_POR_CUENTA = 1000.0 * (5006.0 / 4096.0) / 185.0

CANALES = {
    'y_raw': _Canal('y_raw', GRADOS_POR_CUENTA, 'deg'),
    'y_uw':  _Canal('y_uw',  GRADOS_POR_CUENTA, 'deg'),
    'u':     _Canal('u',     1.0,               'pwm'),
    'i':     _Canal('i',     MA_POR_CUENTA,     'mA'),
}


class BancoSimulado:
    """Todo lo que los notebooks le piden a un banco."""

    def __init__(self, ruido=0.5, semilla=0):
        self._rng = np.random.default_rng(semilla)
        self.ruido = ruido

        self.uff = 0
        self.cal = 0
        self.izero = 2048
        self.tickdiv = 10

        self.lut = [0] * 64
        self._lutw = 0xFFFFFFFF

        self.info = 'CtrlLink 1 Banco (SIMULADO) chans=4 dt_us=2000'
        self.simulado = True

    @property
    def dt(self):
        return self.tickdiv / 5000.0

    # ------------------------------------------------------ los parámetros

    def set(self, name, value, tries=3):
        if name == 'lutw':
            value = int(value) & 0xFFFFFFFF
            if value != self._lutw:
                self._lutw = value
                i = value >> 16
                if i < 64:
                    v = value & 0xFFFF
                    self.lut[i] = v - 65536 if v > 32767 else v
            return value

        setattr(self, name, value)
        return value

    def get(self, name):
        if name == 'lutsum':
            a = b = 0
            for v in self.lut:
                for byte in ((v & 0xFF), ((v >> 8) & 0xFF)):
                    a = (a + byte) & 0xFF
                    b = (b + a) & 0xFF
            return (b << 8) | a
        return getattr(self, name)

    def channel(self, name):
        try:
            return CANALES[name]
        except KeyError:
            raise KeyError(f'no hay ningun canal llamado {name!r}') from None

    def rest(self):
        self.uff = 0

    def zero_current(self, seconds=0.3):
        """El cero de la corriente, que en el modelo ya está en cero."""
        return self.izero

    # ------------------------------------------------------------ el motor

    @staticmethod
    def _recorte(u):
        return min(255.0, max(-255.0, float(u)))

    def _velocidad_final(self, u):
        """Vueltas por segundo en régimen para un comando `u`."""
        u = self._recorte(u)
        if abs(u) <= U_MUERTO:
            return 0.0
        return np.sign(u) * (abs(u) - U_MUERTO) / (255.0 - U_MUERTO) * W_MAX

    def _comando(self, t, eventos):
        """El `uff` a lo largo de la captura: lo que sale al puente en cada fila."""
        u = np.full(len(t), self._recorte(self.uff))
        for retardo, nombre, valor in sorted(eventos, key=lambda e: float(e[0])):
            if nombre == 'uff':
                u[t >= float(retardo)] = self._recorte(valor)
        return u

    def _perfil(self, t, u):
        """La velocidad instantánea a lo largo de la captura, en vueltas por segundo.

        La captura arranca con el eje ya a régimen del comando que tiene puesto,
        no desde parado: es lo que hace `regimen()` esperando un par de segundos
        antes de medir, y `desaceleracion()` llevando el motor a velocidad antes
        de soltarlo.
        """
        dt = self.dt
        w = np.zeros_like(t)
        actual = self._velocidad_final(u[0])

        for k in range(len(t)):
            objetivo = self._velocidad_final(u[k])
            tau = TAU_SUBE if abs(objetivo) > abs(actual) else TAU_BAJA
            actual += (objetivo - actual) * dt / tau
            w[k] = actual

        return w

    def _corriente(self, u, w):
        """Miliamperes, como los informa el ADC: en escalones de cuatro cuentas."""
        v = u / 255.0 * V_MOTOR
        i = (v - KE * w * 2 * np.pi) / R_MOTOR * 1000.0
        cuentas = np.rint(i / MA_POR_CUENTA / 4 + self._rng.normal(0, 0.3, len(u))) * 4
        return cuentas * MA_POR_CUENTA

    def _error_sensor(self, theta, w):
        """El error de ángulo en cuentas: el del sensor, más el del motor."""
        e = np.zeros_like(theta)

        for k, (A, phi) in ERROR_SENSOR.items():
            e += A * np.sin(2*np.pi*k*theta/CUENTAS + phi)

        k, A, phi = RIPPLE_MOTOR

        # El piso de velocidad es del modelo, no de la física: omega^-2 diverge y
        # sin un piso el eje casi detenido tendría una ondulación de cientos de
        # cuentas, que no se parece a ningún banco.
        seguro = np.maximum(np.abs(w), 2.0)
        e += A * (5.0/seguro)**2 * np.sin(2*np.pi*k*theta/CUENTAS + phi)

        return e

    def _lut_lookup(self, crudo):
        """Igual que lut_lookup() en el sketch, redondeo incluido."""
        crudo = np.asarray(crudo, dtype=np.int64) & (CUENTAS - 1)
        i = (crudo >> 6) & 63
        frac = crudo & 0x3F
        tabla = np.array(self.lut, dtype=np.int64)
        octavos = (tabla[i] * (64 - frac) + tabla[(i + 1) & 63] * frac) >> 6
        return (octavos + 4) >> 3

    # ---------------------------------------------------------- la captura

    def capture(self, duration, events=(), poll=0.005, warn=True):
        t = np.arange(0, float(duration), self.dt)

        u = self._comando(t, events)
        w = self._perfil(t, u)
        theta = np.cumsum(w) * CUENTAS * self.dt

        # Lo que informa el sensor llega una muestra después de lo que hizo el
        # eje, que es lo que hacen el filtro del AS5600 y la cadena de muestreo.
        if RETARDO:
            theta = np.concatenate([np.full(RETARDO, theta[0]), theta[:-RETARDO]])

        medido = theta + self._error_sensor(theta, w)
        medido = medido + self._rng.normal(0, self.ruido, len(t))

        crudo = np.mod(np.rint(medido), CUENTAS).astype(np.int64)
        corregido = medido - (self._lut_lookup(crudo) if self.cal else 0)

        df = pd.DataFrame({
            't': t,
            'y_raw': crudo * GRADOS_POR_CUENTA,
            'y_uw': corregido * GRADOS_POR_CUENTA,
            'u': u,
            'i': self._corriente(u, w),
        })

        df.attrs.update(tick=np.arange(len(t), dtype=np.int64) * self.tickdiv,
                        dec=self.tickdiv, dt_us=self.dt*1e6, marks=[], notes=[],
                        gaps=0, missed=0, maxlate=600, sovr=0, serr=0,
                        spres=1, mstat=0x20,
                        wall=float(duration), rows=len(t), drops=0,
                        units={'y_raw': 'deg', 'y_uw': 'deg'})

        return df

    def step(self, name, value, pre=0.1, post=0.9, back=None, warn=True):
        antes = getattr(self, name)
        df = self.capture(pre + post, events=[(pre, name, value)])
        df['t'] -= pre
        setattr(self, name, antes if back is None else back)
        return df

    # ---------------------------------------------------- puesta en marcha

    def bringup(self, motor=True, u=120):
        print(f'puesta en marcha: {self.info}')
        for etiqueta, detalle in [
                ('muestreo', '500 Hz reales contra 500 nominales, 0 perdidos'),
                ('sensor', 'contesta en el bus'),
                ('iman', 'detectado, AGC en rango'),
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
    print('El error de sensor que se va a "descubrir" lo puso banco_simulado.py.')
    print('=' * 68)
    return BancoSimulado(**kw)
