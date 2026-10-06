"""El lazo de velocidad del TP3, cerrado sobre el banco simulado.

Es la referencia de lo que el sketch tiene que hacer, escrita del lado de la
computadora para poder sacar la clave antes de que exista el firmware:

  - la velocidad sale de la diferencia del ángulo medido sobre `vel_win` períodos;
  - el controlador es una ganancia por una cadena de bloques (s + z)/(s + p), cada
    uno discretizado por Tustin, con p = 0 para un integrador;
  - el comando se recorta a lo que el actuador puede dar, y un bloque integrador
    deja de cargarse mientras el recorte empuja en el sentido del error.

El motor es el de `banco_simulado.py`, con su transistor de un cuadrante, su
rozamiento y su error de sensor. El banco simulado no tiene puente en H, así que
todo lo que sale de acá vale para un banco B′.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'python'))

import banco_simulado as bs                     # noqa: E402
from tabla_angulo import CUENTAS                # noqa: E402

TS = 0.002                                      # s, el período del lazo: 500 Hz
RAD_POR_CUENTA = 2 * math.pi / CUENTAS


class Bloque:
    """(s + z)/(s + p) por Tustin, en forma directa II transpuesta."""

    def __init__(self, z, p, ts=TS):
        c = 2.0 / ts
        self.b0 = (c + z) / (c + p)
        self.b1 = (z - c) / (c + p)
        self.a1 = (p - c) / (c + p)
        self.integra = (p == 0)
        self.s = 0.0

    def paso(self, x, congelar=False):
        y = self.b0 * x + self.s
        if not (congelar and self.integra):
            self.s = self.b1 * x - self.a1 * y
        return y


def simular(ref, k=0.0, bloques=(), uff=lambda t: 0.0, pre=4.0, post=4.0, motor=None,
            vel_win=5, antiwindup=True, carga=lambda t: 0.0, semilla=0):
    """Corre el lazo `pre` segundos antes de t = 0 y `post` después.

    `ref(t)` en rad/s, `uff(t)` en % --la prealimentación, que con k = 0 es el lazo
    abierto--, `carga(t)` en N·m de par resistente agregado. Devuelve t, la
    velocidad del eje, la medida y el comando.
    """
    banco = bs.BancoSimulado(motor=motor, semilla=semilla)
    banco._w = banco._i = 0.0
    tc0 = banco.motor['Tc']

    cadena = [Bloque(z, p) for z, p in bloques]
    n = int(round((pre + post) / TS))
    t = np.arange(n) * TS - pre
    w, wm, u = np.empty(n), np.empty(n), np.empty(n)
    angulos = [0] * (vel_win + 1)
    saturado = False

    hechos = 0                                  # períodos de PWM ya integrados

    for i in range(n):
        # El ángulo como lo ve la placa: cuentas enteras, con el error del sensor.
        # El motor avanza de a períodos de PWM enteros y el tick cae entre dos, así
        # que el ángulo se lleva hasta el tick con la velocidad de ese momento.
        resto = i * TS - hechos * bs.PWM_T
        theta = np.array([banco._theta + banco._w * resto / RAD_POR_CUENTA])
        cuenta = int(np.rint(theta + banco._error_sensor(theta, np.array([banco._w]))
                             + banco._rng.normal(0, banco.ruido))[0])
        angulos = angulos[1:] + [cuenta]
        medida = (angulos[-1] - angulos[0]) * RAD_POR_CUENTA / (vel_win * TS)

        e = ref(t[i]) - medida
        x = k * e
        for b in cadena:
            x = b.paso(x, congelar=antiwindup and saturado)
        pedido = uff(t[i]) + x
        salida = min(100.0, max(0.0, pedido))
        saturado = (pedido > 100.0 and e > 0) or (pedido < 0.0 and e < 0)

        w[i], wm[i], u[i] = banco._w, medida, salida

        # 2 ms son dos períodos y medio de PWM: se alternan dos y tres.
        periodos = int((i + 1) * TS / bs.PWM_T + 1e-9) - hechos
        hechos += periodos
        banco.motor['Tc'] = tc0 + carga(t[i])
        banco._integrar(np.full(periodos, round(salida * 2.55)))

    return t, w, wm, u


def medir(t, w, w0, r1, banda=0.05, cola=1.0):
    """Lo que se le mide a un escalón de referencia de w0 a r1 en t = 0.

    El sobrepico y el tiempo de establecimiento se refieren al valor final que la
    respuesta efectivamente alcanza, como `step_info`; el error de régimen, a la
    referencia. Los tres en por ciento del salto pedido, salvo `ts`, en segundos.
    """
    despues = t >= 0
    td, wd = t[despues], w[despues]
    final = wd[td >= td[-1] - cola].mean()
    recorrido = final - w0
    afuera = np.abs(wd - final) > banda * abs(recorrido)
    ts = td[np.nonzero(afuera)[0][-1]] + TS if afuera.any() else 0.0
    extremo = wd.max() if recorrido > 0 else wd.min()
    return dict(Mp=100 * max((extremo - final) / recorrido, 0.0), ts=ts,
                ess=100 * (r1 - final) / (r1 - w0), final=final)
