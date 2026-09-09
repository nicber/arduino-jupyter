"""Un banco de mentira, para poder dar la clase sin la placa.

Expone lo mismo que `Bench` --los parámetros como atributos, `capture()`,
`bringup()`-- pero las capturas salen de un modelo en lugar de un motor. El
notebook de calibración no se entera: corre el mismo código en los dos casos, y
lo único que cambia es de dónde vienen las filas.

Es de mentira y lo dice. El modelo tiene adentro un error de sensor que alguien
eligió, así que "descubrirlo" no prueba nada sobre ningún AS5600; lo que prueba
es que el procedimiento encuentra lo que hay que encontrar, que es exactamente lo
que uno quiere mostrar en un pizarrón. Cuando el banco está, se usa el banco.
"""
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


class BancoSimulado:
    """Todo lo que el notebook de calibración le pide a un banco."""

    def __init__(self, ruido=0.5, semilla=0, sfilt=3):
        self._rng = np.random.default_rng(semilla)
        self.ruido = ruido

        self.mode = 0
        self.offset = 0
        self.uff = 0
        self.ref = 0
        self.cal = 0
        self.sfilt = sfilt
        self.tickdiv = 10
        self.kp = self.ki = self.kd = 0.0

        self.lut = [0] * 64
        self._lutw = 0xFFFFFFFF

        self.dt = self.tickdiv / 5000.0
        self.info = 'CtrlLink 1 ControlDemo (SIMULADO) chans=7 dt_us=2000'
        self.simulado = True

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

    def rest(self):
        self.mode = 0
        self.uff = 0

    def zero(self):
        self.offset = 0

    def gains(self, kp=0.0, ki=0.0, kd=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd

    def smooth(self, which, tau):
        pass

    # ------------------------------------------------------------ el motor

    @staticmethod
    def _velocidad_final(u):
        """Vueltas por segundo en régimen para un comando `u`.

        Con una zona muerta abajo: un puente Darlington contra 5 V no arranca con
        cualquier cosa, y un notebook que pida uff=40 tiene que ver que no gira.
        """
        u = float(u)
        muerto = 60.0
        if abs(u) <= muerto:
            return 0.0
        return np.sign(u) * (abs(u) - muerto) / 255.0 * 12.0

    def _perfil(self, t, eventos):
        """La velocidad instantánea a lo largo de la captura, en vueltas por segundo."""
        objetivo = self._velocidad_final(self.uff)

        # La captura arranca con el eje ya a régimen, no desde parado: `regimen()`
        # espera un par de segundos antes de medir y `desaceleracion()` lleva el
        # motor a velocidad antes de soltarlo. Arrancar de cero metería el
        # transitorio de arranque adentro de la ventana de ajuste, y ahí el
        # polinomio de la tendencia no le llega: el residuo se dispara y las
        # amplitudes salen cualquier cosa. Es una falla real y se ve --el ajuste
        # informa su residuo-- pero acá no corresponde tenerla.
        w = np.zeros_like(t)
        actual = objetivo
        dt = t[1] - t[0] if len(t) > 1 else 0.002

        cambios = sorted((float(d), float(v)) for d, nombre, v in eventos
                         if nombre == 'uff')
        siguiente = 0

        for i, ti in enumerate(t):
            while siguiente < len(cambios) and ti >= cambios[siguiente][0]:
                objetivo = self._velocidad_final(cambios[siguiente][1])
                siguiente += 1

            # Arrancar cuesta; soltar cuesta más, porque frenando sólo actúa el
            # rozamiento. Eso es lo que hace que una desaceleración barra la
            # velocidad despacio y dé muchas vueltas a cada una.
            tau = 0.3 if abs(objetivo) > abs(actual) else 6.0
            actual += (objetivo - actual) * dt / tau
            w[i] = actual

        return w

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

        # Sólo lazo abierto: el notebook de calibración mide con el motor a
        # comando constante y soltándolo, y un lazo cerrado simulado no agregaría
        # nada que se pueda mostrar. En el banco de verdad `mode` sigue haciendo
        # lo suyo.
        w = self._perfil(t, events)
        theta = np.cumsum(w) * CUENTAS * self.dt
        medido = theta + self._error_sensor(theta, w)
        medido = medido + self._rng.normal(0, self.ruido, len(t))
        refs = np.full(len(t), float(self.ref))

        crudo = np.mod(np.rint(medido), CUENTAS).astype(np.int64)

        corregido = medido - (self._lut_lookup(crudo) if self.cal else 0)

        df = pd.DataFrame({
            't': t,
            'y_raw': crudo * GRADOS_POR_CUENTA,
            # El firmware informa y = offset - counts, así que y_uw baja cuando el
            # ángulo sube. Se reproduce el signo para que el notebook no descubra
            # tarde que el banco de verdad no se le parece.
            'y_uw': -corregido * GRADOS_POR_CUENTA,
            'y_uwf': -corregido * GRADOS_POR_CUENTA,
            'u': np.full(len(t), float(self.uff)),
            'i': np.abs(w) * 20.0,
            'ref': refs,
        })

        df.attrs.update(tick=np.arange(len(t), dtype=np.int64) * self.tickdiv,
                        dec=self.tickdiv, dt_us=self.dt*1e6, marks=[], notes=[],
                        gaps=0, missed=0, maxlate=600, sovr=0, serr=0,
                        spres=1, mstat=0x20, agc=128, mag=1800,
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
                ('lazo de control', '500 Hz reales contra 500 nominales, 0 perdidos'),
                ('sensor', 'contesta en el bus'),
                ('iman', 'detectado, AGC 128/255, campo 1800'),
                ('bus i2c', '0 errores de transferencia, 0 desbordes'),
                ('motor', f'gira con u={u}')]:
            print(f'  [   ok]  {etiqueta:<18}  {detalle}')
        print('\n  NADA DE ESTO ES REAL: es el banco simulado.')


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
