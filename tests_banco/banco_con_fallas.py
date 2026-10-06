"""Un banco simulado con las fallas que tiene el hardware real, y un Bench encima.

`BancoSimulado` (python/banco_simulado.py) sigue al banco en lo mecánico, pero su
cableado es perfecto: una placa a 5 V, los signos ya bien y la fuente del motor
siempre prendida. Justamente eso es lo que la lógica de `bench.py` tiene que
resolver, y donde es fácil equivocarse (medir `cur_inv` con la caída de AVCC
superpuesta). Acá se agregan, a nivel de lo que ven los pines:

- `angulo_invertido`: el imán mira al revés, así que la cuenta cruda baja con +u;
- `sensor_invertido`: el ACS712 está insertado al revés;
- la placa es el clon a 3,3 V con el ACS712 en los 5 V del USB: A0 se lee contra
  AVCC, que se hunde con el ciclo de trabajo (`caida`, en partes por uno), así que
  sin divisor la lectura sube con el PWM y reposa lejos de la mitad de la escala;
- `divisor`: (arriba_ohm, abajo_ohm) si A1 lee los 5 V del sensor, None si no hay
  nada en A1, o 'suelto' si está cableado pero una resistencia no hace contacto. La
  placa hace lo mismo que Sense/SupplyRatio.h con `cur_div`;
- `fuente_prendida`: sin fuente del motor el PWM no mueve nada ni hace corriente.

`bench_sobre(banco)` arma un `bench.Bench` real cuyo enlace es este banco, así que
lo que se prueba es el código de `bench.py`, no una copia.
"""
import numpy as np

import comun  # noqa: F401  (el path del proyecto)
import banco_simulado as bs
import bench

AVCC = 3.3          # el clon del banco
V_SENSOR = 4.95     # los 5 V del USB, que nunca son 5,000
DIVISOR_DEL_BANCO = (5100, 2000)


def caida_tipica(D):
    """La corriente de base del BD139 hunde AVCC: ~0,4 % apenas conmuta y ~1,35 % a fondo."""
    D = np.asarray(D, dtype=float)
    return np.where(D > 0, 0.004 + 0.0095 * D, 0.0)


# El motor del banco medido (TP2 v3): R y L de la corriente, el resto de la velocidad.
# BancoSimulado trae R = 24 ohm, con corrientes de pocos mA que no alcanzan para medir
# un signo; el banco real arranca con cientos de mA.
MOTOR_MEDIDO = dict(Vs=5.0, Vd=0.7, R=8.1, L=5.1e-3, Ke=6.64e-3, J=1.01e-6,
                    Tc=1.41e-4, Ts=2.61e-4, B=1.66e-7)


class BancoConFallas(bs.BancoSimulado):

    def __init__(self, angulo_invertido=False, sensor_invertido=False, caida=caida_tipica,
                 divisor=DIVISOR_DEL_BANCO, fuente_prendida=True, motor=MOTOR_MEDIDO, **kw):
        self.divisor = divisor
        super().__init__(motor=motor, **kw)
        self.angulo_invertido = angulo_invertido
        self.sensor_invertido = sensor_invertido
        self.caida = caida
        self.fuente_prendida = fuente_prendida
        self.info = 'CtrlLink 1 Banco (CON FALLAS) chans=5 dt_us=2000'
        self.vueltas_con_pwm = 0.0     # cuánto giró el eje mientras había comando
        self._params = {n: None for n in self._nombres()}

    # Sin fuente, el transistor conmuta pero la armadura no ve tensión.
    def _periodo(self, D, w, i0):
        return super()._periodo(D if self.fuente_prendida else 0.0, w, i0)

    # --- los pines
    def _a0(self, ratio, duty):
        """A0 en cuentas de 12 bits contra AVCC: `ratio` es la salida sobre los 5 V del sensor."""
        return ratio * V_SENSOR / (AVCC * (1.0 - self.caida(duty / 255.0))) * 4096

    def _a1(self, duty):
        if self.divisor is None or self.divisor == 'suelto':
            return np.zeros_like(np.asarray(duty, dtype=float))
        arriba, abajo = self.divisor
        k = abajo / (arriba + abajo)
        return np.minimum(k * V_SENSOR / (AVCC * (1.0 - self.caida(duty / 255.0))) * 4096, 4095)

    @property
    def cur_a1(self):
        return int(round(float(self._a1(abs(self.ctl_uff))))) if int(self.cur_div) else 0

    @cur_a1.setter
    def cur_a1(self, _):
        pass                            # una lectura: BancoSimulado la inicializa en cero

    def get(self, name):
        return self.cur_a1 if name == 'cur_a1' else super().get(name)

    def capture(self, duration, events=(), poll=0.005, warn=True, canales=None):
        theta0 = self._theta
        df = super().capture(duration, events=events, poll=poll, warn=warn, canales=None)
        if (df['u'] != 0).any():
            self.vueltas_con_pwm += abs(self._theta - theta0) / bs.CUENTAS

        # --- el ángulo: el imán al revés invierte la cuenta antes que ang_inv
        if self.angulo_invertido:
            df['y_uw'] = -df['y_uw']
            df['y_raw'] = (360.0 - df['y_raw']) % 360.0

        # --- la corriente: de lo publicado a la salida del sensor, a los pines, y de vuelta
        lsb = bs.MA_POR_CUENTA
        s_pub = -1 if self.cur_inv else 1
        equivalentes = s_pub * df['i'].to_numpy() / lsb + self.cur_zero   # lo que "midió" el simulado
        senal = equivalentes - bs.REPOSO_I
        if self.sensor_invertido:
            senal = -senal
        ratio = (bs.REPOSO_I + senal) / 4000.0          # 4000 cuentas equivalentes son 5 V

        duty = np.abs(df['u'].to_numpy())
        filas = max(1, int(self.cur_filas))
        duty_ventana = np.convolve(duty, np.ones(filas) / filas, mode='full')[:len(duty)]
        a0 = self._a0(ratio, duty_ventana)
        a1 = self._a1(duty_ventana)

        div = int(self.cur_div)
        if not div:
            publicado = a0
        else:
            with np.errstate(divide='ignore'):
                publicado = np.where(a1 > 0, a0 / np.maximum(a1, 1e-9) * 4000 * div / 10000, 4095)
            publicado = np.minimum(publicado, 4095)
        df['i'] = s_pub * (publicado - self.cur_zero) * lsb

        if canales is not None:
            canales = [canales] if isinstance(canales, str) else list(canales)
            df = df[['t'] + [c for c in bs.CANALES if c in canales]]
        return df


def bench_sobre(banco, cableado):
    """Un `bench.Bench` real sobre `banco`, con `cableado.json` en `cableado`."""
    rig = bench.Bench.__new__(bench.Bench)
    object.__setattr__(rig, 'link', banco)
    bench.CABLEADO = cableado
    return rig
