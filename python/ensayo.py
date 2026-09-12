"""Un ensayo: la velocidad a partir del ángulo, las unidades, y el archivo.

La placa entrega el ángulo desenrollado en grados, el comando en cuentas de PWM
y la corriente en miliamperes, y nada más: la velocidad no se mide, se calcula,
y se calcula acá y no en la placa a propósito. Derivar amplifica el ruido, así
que hay que filtrar, y un filtro en la placa se identifica después como si fuera
del motor. De este lado se ve lo que se hizo, se cambia y se vuelve a correr sin
tocar el banco.

    import ensayo

    df = dev.step('uff', 200, pre=0.3, post=1.2, back=0)
    t, w = ensayo.velocidad(df)                  # rad/s
    ensayo.guardar(df, 'datos/escalon_200.csv')  # t, u, theta, omega, i
    datos = ensayo.cargar('datos/escalon_200.csv')

El archivo lleva las columnas en las unidades en las que se escribe un modelo:
segundos, por ciento de PWM, radianes, radianes por segundo y amperes. Es lo que
espera cualquier herramienta de identificación, y lo que se puede leer dentro de
un año sin acordarse de nada de esto.
"""
import numpy as np
import pandas as pd

U_MAX = 255           # cuentas de PWM a fondo
VUELTA = 2 * np.pi    # radianes

COLUMNAS = ['t', 'u', 'theta', 'omega', 'i']


def _dt(t):
    """El período de muestreo, en segundos, mirando los tiempos y no los attrs.

    Así vale igual para una captura recién hecha y para un archivo leído de
    vuelta, que no trae attrs.
    """
    t = np.asarray(t, dtype=float)
    return float(np.median(np.diff(t))) if len(t) > 1 else 0.002


def signo(df):
    """+1 si un comando positivo sube el ángulo, -1 si lo baja.

    Es una propiedad del cableado --de qué lado están los cables del motor en el
    puente, y de qué lado mira el imán al sensor--, no de la planta, y se decide
    mirando para dónde fue el eje mientras el comando era positivo. Devuelve +1
    si en la captura no hubo comando o no hubo movimiento.
    """
    u = df['u'].to_numpy(dtype=float)
    theta = df['y_uw'].to_numpy(dtype=float)
    empuja = np.abs(u) > 0
    if empuja.sum() < 2:
        return 1
    avance = np.diff(theta)[empuja[1:]] * np.sign(u[1:][empuja[1:]])
    return -1 if avance.sum() < 0 else 1


def velocidad(df, ventana=0.02, canal='y_uw', signo_banco=1):
    """(t, omega): radianes por segundo, derivando el ángulo y promediando.

    La derivada es la diferencia hacia atrás dividida por el período, que es lo
    que uno escribiría en un microcontrolador. Después se promedia sobre
    `ventana` segundos, con un promedio móvil centrado: no atrasa la señal --un
    filtro causal atrasaría, y ese atraso se confundiría con un tiempo muerto
    del motor-- pero sí redondea las esquinas de un escalón, así que la ventana
    tiene que ser corta contra la constante de tiempo que se quiere ver.

    Con `ventana = 0` no se promedia, y se ve la cuantización desnuda: una cuenta
    del sensor por período, a 500 Hz, son 0,77 rad/s. Devuelve una muestra menos
    que el cuadro.
    """
    t = df['t'].to_numpy(dtype=float)
    theta = signo_banco * np.deg2rad(df[canal].to_numpy(dtype=float))
    dt = _dt(t)
    w = np.diff(theta) / dt
    n = max(1, int(round(ventana / dt)))
    if n > 1:
        # Los extremos se rellenan con el valor del borde y no con ceros: un
        # promedio contra ceros hunde las puntas de la serie, y las puntas son
        # justo de donde salen los regímenes de un escalón.
        pad = np.pad(w, (n // 2, n - 1 - n // 2), mode='edge')
        w = np.convolve(pad, np.ones(n) / n, mode='valid')
    return t[1:], w


def normalizar(df, ventana=0.02, signo_banco=None):
    """La captura en las unidades de un modelo: t [s], u [%], theta [rad], omega [rad/s], i [A].

    `signo_banco` da vuelta el ángulo si un comando positivo lo hace bajar; por
    omisión se lo deduce de la propia captura con `signo()`. La velocidad sale de
    `velocidad()` con la misma `ventana`, y la primera fila --que no tiene
    velocidad-- se descarta.
    """
    if signo_banco is None:
        signo_banco = signo(df)

    t, w = velocidad(df, ventana=ventana, signo_banco=signo_banco)
    theta = signo_banco * np.deg2rad(df['y_uw'].to_numpy(dtype=float))[1:]

    return pd.DataFrame({
        't':     t,
        'u':     df['u'].to_numpy(dtype=float)[1:] * 100.0 / U_MAX,
        'theta': theta,
        'omega': w,
        'i':     df['i'].to_numpy(dtype=float)[1:] / 1000.0,
    })


def guardar(df, ruta, ventana=0.02, signo_banco=None):
    """Escribe la captura como CSV con las columnas de `normalizar()`. Devuelve la ruta."""
    from pathlib import Path

    ruta = Path(ruta)
    ruta.parent.mkdir(parents=True, exist_ok=True)
    normalizar(df, ventana=ventana, signo_banco=signo_banco).to_csv(
        ruta, index=False, float_format='%.6g')
    return ruta


def cargar(ruta):
    """Lee un CSV escrito por `guardar()`."""
    df = pd.read_csv(ruta)
    faltan = [c for c in COLUMNAS if c not in df]
    if faltan:
        raise ValueError(f'{ruta}: faltan las columnas {faltan}')
    return df
