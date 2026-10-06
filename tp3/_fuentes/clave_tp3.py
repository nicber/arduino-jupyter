"""Clave del TP3, resuelta contra el banco simulado.

    python tp3/_fuentes/clave_tp3.py

Recorre los tres ejercicios con el motor de `banco_simulado.py` y el lazo de
`lazo_simulado.py`, e imprime al lado de cada medición lo que predice el modelo
identificado. Los números son de ESE motor, con actuador de un cuadrante: en un
banco real cambian, y lo que tiene que repetirse es el orden de magnitud y el
sentido de cada comparación. El resumen está en la cabecera de `build_tp3.py`.
"""
import control as ctrl
import numpy as np

from lazo_simulado import TS, medir, seguir, simular

U0, U1 = 40.0, 60.0             # %, el escalón de identificación del TP2
VEL_WIN = 5                     # períodos de la ventana de velocidad, por omisión
VEL_WIN_LARGA = 25              # la del ejercicio 3: más retardo, menos ruido


def titulo(texto):
    print(f'\n=== {texto}')


def fila(nombre, t, w, u, w0, r1):
    m = medir(t, w, w0, r1)
    despues = u[t >= 0]
    print(f'  {nombre:32s} Mp {m["Mp"]:5.1f} %   ts(5 %) {m["ts"]:5.2f} s   '
          f'error {m["ess"]:6.1f} %   final {m["final"]:6.1f} rad/s   '
          f'u de {despues.min():5.1f} a {despues.max():5.1f} %   '
          f'std(u) {u[t > t[-1] - 1].std():5.2f} %')
    return m


def escalon(a, c):
    return lambda t: a if t < 0 else c


# ------------------------------------------------------------- el modelo (TP2)

titulo('El modelo, como sale del TP2: escalón de u0 a u1 a lazo abierto')
t, w, _, _ = simular(lambda t: 0.0, uff=escalon(U0, U1), pre=8, post=8)
W0 = w[(t > -1) & (t < 0)].mean()
W1 = w[t > 7].mean()
K = (W1 - W0) / (U1 - U0)
TAU = t[np.argmax((w - W0) / (W1 - W0) >= 0.632)]
WE = (W0 + W1) / 2              # hasta dónde va el escalón de ensayo
DR = WE - W0
print(f'  w0 = {W0:.1f} rad/s   w1 = {W1:.1f} rad/s   K = {K:.2f} (rad/s)/%   tau = {TAU:.3f} s')
print(f'  escalón de ensayo: de {W0:.1f} a {WE:.1f} rad/s (salto de {DR:.1f})')

ensayo = escalon(W0, WE)


def lazo_abierto(a, c, k_modelo=None, **kw):
    """u = u0 + (r - w0)/K: el modelo invertido."""
    k_modelo = k_modelo or K
    return simular(lambda t: 0.0, uff=lambda t: U0 + ((a if t < 0 else c) - W0) / k_modelo,
                   pre=8, post=6, **kw)


# ---------------------------------------------------------------- ejercicio 1

titulo('1.1  Lazo abierto con el modelo')
print(f'  predicho: ts(5 %) = 3 tau = {3 * TAU:.2f} s, y llega a la referencia')
t, w, _, u = lazo_abierto(W0, WE)
fila('lazo abierto', t, w, u, W0, WE)

titulo('1.2 y 1.3  Proporcional con prealimentación constante, u = u0 + Kp e')
KP_R2 = 2 / K                   # ts = 3 tau/(1 + K Kp) <= tau
for kp in (KP_R2, 3 / K):
    print(f'  Kp = {kp:.3f}: predicho ts = {3 * TAU / (1 + K * kp):.2f} s, '
          f'error = {100 / (1 + K * kp):.1f} %, u(0+) = {U0 + kp * DR:.1f} %')
    t, w, _, u = simular(ensayo, k=kp, uff=lambda t: U0)
    fila(f'P, Kp = {kp:.3f}', t, w, u, W0, WE)

titulo('1.4  El Kp que pide R1 (error <= 2 %)')
KP_R1 = 49 / K
print(f'  Kp >= 49/K = {KP_R1:.2f}: u(0+) = {U0 + KP_R1 * DR:.0f} %, satura')
t, w, _, u = simular(ensayo, k=KP_R1, uff=lambda t: U0)
fila(f'P, Kp = {KP_R1:.2f}', t, w, u, W0, WE)

titulo('1.5 y 2.5  Carga constante desde t = 0 (un par resistente, como un dedo)')
PI_KC = 0.8                     # el diseño de referencia del ejercicio 2
PI = [(1 / TAU, 0.0)]
U_WE = U0 + DR / K
for par in (2e-5, 4e-5):
    carga = lambda t, par=par: par if t >= 0 else 0.0
    for nombre, kw in (('lazo abierto', dict(uff=lambda t: U_WE, pre=8)),
                       (f'P, Kp = {KP_R2:.3f}', dict(k=KP_R2, uff=lambda t: U_WE, pre=8)),
                       ('PI de referencia', dict(k=PI_KC, bloques=PI))):
        t, w, _, u = simular(lambda t: WE, carga=carga, post=5, **kw)
        antes = (t > -1) & (t < 0)
        print(f'  {par * 1e6:3.0f} uN·m  {nombre:18s} w: {w[antes].mean():6.1f} -> mínimo '
              f'{w[t >= 0].min():6.1f} -> final {w[t > 4].mean():6.1f} rad/s   '
              f'u: {u[antes].mean():5.1f} -> {u[t > 4].mean():5.1f} %')


# ---------------------------------------------------------------- ejercicio 2

titulo('2.2 a 2.4  Diseño de referencia: PI con el cero sobre el polo de la planta')
print(f'  D = Kc (s + 1/tau)/s  ->  L = Kc K/(tau s),  T = 1/(tau_c s + 1),  tau_c = tau/(K Kc)')
print(f'  R2: Kc >= 3/K = {3 / K:.2f}    R4: Kc <= (100 - u0)/salto = {(100 - U0) / DR:.2f}')
for kc in (3 / K, PI_KC, (100 - U0) / DR):
    print(f'  Kc = {kc:.2f}: predicho ts = {3 * TAU / (K * kc):.2f} s, sin sobrepico, '
          f'u(0+) = {U0 + kc * DR:.1f} %')
    t, w, _, u = simular(ensayo, k=kc, bloques=PI)
    fila(f'PI, Kc = {kc:.2f}', t, w, u, W0, WE)

titulo('2.6  Otras arquitecturas, con la misma ganancia')
for nombre, kw in (
        ('PI, cero en 2/tau', dict(k=PI_KC, bloques=[(2 / TAU, 0.0)])),
        ('PI, cero en 4/tau', dict(k=PI_KC, bloques=[(4 / TAU, 0.0)])),
        ('atraso, beta = 20, con u0', dict(k=PI_KC, bloques=[(1 / TAU, 1 / (20 * TAU))],
                                           uff=lambda t: U0)),
        ('atraso, beta = 60, con u0', dict(k=PI_KC, bloques=[(1 / TAU, 1 / (60 * TAU))],
                                           uff=lambda t: U0))):
    t, w, _, u = simular(ensayo, **kw)
    fila(nombre, t, w, u, W0, WE)

titulo('2.6  El rizado del comando contra la ventana de velocidad (PI de referencia)')
for ventana in (1, 2, 5, 10, 25):
    t, w, _, u = simular(ensayo, k=PI_KC, bloques=PI, vel_win=ventana)
    fila(f'vel_win = {ventana} ({ventana * TS * 1e3:.0f} ms)', t, w, u, W0, WE)


# ---------------------------------------------------------------- ejercicio 3

titulo('3.1  El mismo PI y el mismo modelo en otros puntos de trabajo')
for a, c in ((150.0, 150.0 + DR), (W0, WE), (W1, W1 + DR / 2)):
    t, w, _, u = simular(escalon(a, c), k=PI_KC, bloques=PI)
    fila(f'PI, de {a:.0f} a {c:.0f}', t, w, u, a, c)
    k_local = (c - a) / (u[t > 3].mean() - u[(t > -1) & (t < 0)].mean())
    t, w, _, u = lazo_abierto(a, c)
    print(f'  {"lazo abierto":32s} llega a {w[(t > -1) & (t < 0)].mean():6.1f} y a '
          f'{w[t > 5].mean():6.1f} rad/s con u = {u[0]:.1f} y {u[-1]:.1f} %   '
          f'(K local, de los u de régimen del PI: {k_local:.1f})')

titulo('3.2  Modelo equivocado: el mismo procedimiento de diseño con otra K y otra tau')
tau_c = TAU / (K * PI_KC)
for fk, ft in ((1, 1), (0.5, 1), (2, 1), (1, 0.7), (1, 1.3)):
    k_mal, tau_mal = K * fk, TAU * ft
    kc = tau_mal / (k_mal * tau_c)
    t, w, _, u = simular(ensayo, k=kc, bloques=[(1 / tau_mal, 0.0)])
    fila(f'PI con K x{fk}, tau x{ft}', t, w, u, W0, WE)
    if ft == 1:
        t, w, _, u = lazo_abierto(W0, WE, k_modelo=k_mal)
        fila(f'lazo abierto con K x{fk}', t, w, u, W0, WE)

titulo('3.3  Márgenes con el retardo que el modelo no tiene')
for ventana in (VEL_WIN_LARGA, VEL_WIN):
    # Media retención, media ventana, y ~1 ms entre el sensor y el PWM.
    td = TS / 2 + ventana * TS / 2 + 0.9e-3
    wc = K * PI_KC / TAU                    # |L| = 1 con L = (K Kc/tau) e^{-s Td}/s
    w180 = np.pi / (2 * td)
    mf = 90 - np.degrees(wc * td)
    print(f'  vel_win = {ventana}: Td ~ {td * 1e3:.1f} ms   wc = {wc:.2f} rad/s   '
          f'MF = {mf:.1f} grados   MG = {w180 / wc:.1f} en {w180:.0f} rad/s   '
          f'margen de retardo = {np.radians(mf) / wc * 1e3:.0f} ms')

titulo('3.4  Multiplicar la ganancia hasta que oscile')
for ventana in (VEL_WIN_LARGA, VEL_WIN):
    for g in (1, 3, 5, 10, 20, 40):
        for a, c in ((150.0, 150.0 + DR), (W0, WE)):
            t, w, _, u = simular(escalon(a, c), k=PI_KC * g, bloques=PI, vel_win=ventana, post=3)
            fila(f'vel_win {ventana}, x{g}, desde {a:.0f}', t, w, u, a, c)


# ------------------------------------------- ejercicio 3, el lazo sobre el ángulo

VUELTA = 2 * np.pi
TD_POS = TS / 2 + 0.9e-3        # sin ventana de velocidad: media retención y ~1 ms


def angulo(nombre, kc, bloques, post=20.0, cierre=-6.0):
    """Un salto de una vuelta en la referencia que avanza a WE, con u_ff del modelo.

    `cierre` es cuánto antes del salto se cierra el lazo: un integrador lento
    necesita decenas de segundos para absorber lo que le falta a u_ff.
    """
    t, e, w, u = seguir(WE, VUELTA, U_WE, k=kc, bloques=bloques, pre=6 - cierre, post=post,
                        cierre=cierre)
    antes, despues, cola = (t > -1) & (t < 0), t >= 0, t > t[-1] - 4
    e0, final = e[antes].mean(), e[cola].mean()
    afuera = np.abs(e[despues] - final) > 0.05 * VUELTA
    ts = t[despues][np.nonzero(afuera)[0][-1]] + TS if afuera.any() else 0.0
    print(f'  {nombre:34s} e antes {e0:6.2f} rad   sobrepico {max(final - e[despues].min(), 0) / VUELTA * 100:6.1f} %   '
          f'ts(5 %) {ts:5.2f} s   e final {final:6.2f} rad, oscilando +-{np.ptp(e[cola]) / 2:5.2f}   '
          f'u de {u[despues].min():5.1f} a {u[despues].max():5.1f} %')


def margen_de_fase(kc, bloques):
    lazo = ctrl.tf([kc * K], [TAU, 1, 0])
    for z, p in bloques:
        lazo = lazo * ctrl.tf([1, z], [1, p])
    _, mf, _, wc = ctrl.margin(lazo)
    return mf - np.degrees(wc * TD_POS), wc


titulo('3.5  Seguir un ángulo con un proporcional: Theta/U = K/(s (tau s + 1))')
KP_POS = 1 / (K * TAU)          # zeta = 1/(2 sqrt(K Kp tau)) = 0,5
mf, wc = margen_de_fase(KP_POS, [])
print(f'  Kp = 1/(K tau) = {KP_POS:.3f} %/rad: zeta = 0,5, wn = 1/tau = {1 / TAU:.2f} rad/s, Mp = 16,3 %, '
      f'ts(5 %) ~ 3/(zeta wn) = {6 * TAU:.1f} s, MF = {mf:.1f} grados en {wc:.2f} rad/s')
print('  error de seguimiento: (lo que le falta o le sobra a u_ff)/Kp')
angulo(f'P, Kp = {KP_POS:.3f}', KP_POS, [])

titulo('3.6  El PI de velocidad (cero sobre el polo de la planta), cerrado sobre el ángulo')
mf, wc = margen_de_fase(KP_POS, PI)
print(f'  L = Kc K/(tau s^2): polos en +-j {np.sqrt(K * KP_POS / TAU):.2f} rad/s, MF = {mf:.2f} grados')
for post in (20.0, 60.0):
    angulo(f'PI, cero en 1/tau, {post:.0f} s', KP_POS, PI, post=post)
angulo(f'PI, cero en 1/tau, Kc = {PI_KC}', PI_KC, PI)

titulo('3.7  El borde: tau s^3 + s^2 + K Kc s + K Kc z = 0 es estable si y sólo si z < 1/tau')
for factor in (2, 1, 0.5, 0.2, 0.1):
    z = factor / TAU
    polos = np.roots([TAU, 1, K * KP_POS, K * KP_POS * z])
    par = polos[np.argmax(np.abs(polos.imag))]
    mf, _ = margen_de_fase(KP_POS, [(z, 0.0)])
    print(f'  z = {factor}/tau: par en {par.real:6.3f} +- j{abs(par.imag):.3f} '
          f'(zeta {-par.real / abs(par):6.3f}), MF = {mf:5.1f} grados, centro de asíntotas {(z - 1 / TAU) / 2:6.3f}')
    angulo(f'PI, cero en {factor}/tau', KP_POS, [(z, 0.0)], cierre=-6.0 if factor >= 1 else -40.0)

titulo('3.8  Con cuidado: dos diseños de referencia')
for nombre, kc, bloques in (
        ('PI lento, cero en 0,1/tau', KP_POS, [(0.1 / TAU, 0.0)]),
        ('PI + adelanto (1/tau, 10/tau)', 2.0, [(0.5 / TAU, 0.0), (1 / TAU, 10 / TAU)])):
    mf, wc = margen_de_fase(kc, bloques)
    print(f'  {nombre}, Kc = {kc:.2f}: MF = {mf:.1f} grados en {wc:.2f} rad/s, '
          f'u(0+) del salto = +{kc * VUELTA:.1f} %')
    angulo(nombre, kc, bloques, cierre=-40.0)

titulo('3.10  Escalón grande, y la bajada (actuador de un cuadrante)')
for a, c in ((200.0, 560.0), (560.0, 300.0)):
    for aw in (True, False):
        t, w, _, u = simular(escalon(a, c), k=PI_KC, bloques=PI, antiwindup=aw, post=6)
        fila(f'PI de {a:.0f} a {c:.0f}, anti-windup {"sí" if aw else "no"}', t, w, u, a, c)
        despues = t >= 0
        extremo = w[despues].max() if c > a else w[despues].min()
        saturado = np.sum((u[despues] >= 100) | (u[despues] <= 0)) * TS
        print(f'  {"":32s} extremo {extremo:.1f} rad/s, {saturado:.2f} s con el comando en el tope')

titulo('3.12  Otro motor (sólo simulado), con el PI y con el lazo abierto sin tocar')
for nombre, motor in (('nominal', {}), ('inercia x2', {'J': 1.5e-6}),
                      ('fuente de 4,2 V', {'Vs': 4.2}), ('viscoso x3', {'B': 7e-7})):
    t, w, _, u = simular(ensayo, k=PI_KC, bloques=PI, motor=motor)
    fila(f'PI, {nombre}', t, w, u, W0, WE)
    t, w, _, u = lazo_abierto(W0, WE, motor=motor)
    print(f'  {"lazo abierto, " + nombre:32s} va de {w[(t > -1) & (t < 0)].mean():6.1f} a '
          f'{w[t > 5].mean():6.1f} rad/s (pedido: de {W0:.1f} a {WE:.1f})')
