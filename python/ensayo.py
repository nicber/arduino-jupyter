"""Un ensayo: esperar al eje, la velocidad a partir del ángulo, las unidades y el archivo.

La placa entrega el ángulo desenrollado en grados, el comando en cuentas de PWM
y la corriente en miliamperes, y nada más: la velocidad no se mide, se calcula,
y se calcula acá y no en la placa a propósito. Derivar amplifica el ruido, así
que hay que filtrar, y un filtro en la placa se identifica después como si fuera
del motor. De este lado se ve lo que se hizo, se cambia y se vuelve a correr sin
tocar el banco.

    import ensayo

    ensayo.esperar_quieto(dev)                            # el eje, parado de verdad
    df = dev.step('ctl_uff', 200, pre=0.3, post=2.0, back=0)
    t, w = ensayo.velocidad(df)                           # rad/s
    ensayo.guardar(df, 'datos/escalon_200.csv')           # t, u, theta, omega, i
    datos = ensayo.cargar('datos/escalon_200.csv')

El archivo lleva las columnas en las unidades en las que se escribe un modelo:
segundos, por ciento de PWM, radianes, radianes por segundo y amperes. Es lo que
espera cualquier herramienta de identificación, y lo que se puede leer dentro de
un año sin acordarse de nada de esto.
"""
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

U_MAX = 255           # cuentas de PWM a fondo

COLUMNAS = ['t', 'u', 'theta', 'omega', 'i']


def _dt(t):
    """El período de muestreo, en segundos, mirando los tiempos y no los attrs.

    Así vale igual para una captura recién hecha y para un archivo leído de
    vuelta, que no trae attrs.
    """
    t = np.asarray(t, dtype=float)
    return float(np.median(np.diff(t))) if len(t) > 1 else 0.002


def esperar_quieto(dev, quieto=0.5, ventana=0.3, limite=60.0):
    """Deja el comando en cero y espera a que el eje pare. Devuelve los segundos que esperó.

    Con el comando en cero el actuador queda abierto y no frena: el eje sigue por
    inercia y sólo lo para el rozamiento, que en este banco tarda muchos segundos
    desde velocidades altas. Un ensayo que arranque antes mide la cola del
    anterior, y eso no se ve en el gráfico: se ve en un modelo que no cierra.

    Espera activamente, mirando el ángulo en ventanas de `ventana` segundos hasta
    que gire menos de `quieto` rad/s. `limite` acota la espera y, si se cumple,
    lo dice en lugar de seguir como si nada.
    """
    dev.ctl_uff = 0
    esperado = 0.0

    while True:
        df = dev.capture(ventana, warn=False)
        esperado += ventana
        giro = abs(np.deg2rad(df['y_uw'].iloc[-1] - df['y_uw'].iloc[0])) / ventana

        if giro < quieto:
            return esperado

        if esperado >= limite:
            print(f'  OJO: despues de {limite:.0f} s el eje sigue girando a '
                  f'{giro:.1f} rad/s')
            return esperado


def velocidad(df, ventana=0.02, canal='y_uw'):
    """(t, omega): radianes por segundo, derivando el ángulo y promediando.

    La derivada es la diferencia central, (theta[k+1] - theta[k-1]) dividida por
    el tiempo real entre esas dos filas, y va asignada a t[k]: no atrasa, y una
    fila perdida no se convierte en un pico. Después se promedia sobre `ventana`
    segundos, con un promedio móvil centrado de un número impar de muestras: no
    atrasa la señal --un filtro causal atrasaría, y ese atraso se confundiría con
    un tiempo muerto del motor-- pero sí redondea las esquinas de un escalón, así
    que la ventana tiene que ser corta contra la constante de tiempo que se quiere
    ver. Y no es causal: sirve para procesar una captura, no para un lazo.

    Con `ventana = 0` no se promedia, y se ve la cuantización desnuda: una cuenta
    del sensor en dos períodos, a 500 Hz, son 0,38 rad/s. Devuelve una muestra
    menos que la captura, alineada con t[1:]; en la última fila, que no tiene
    siguiente, la diferencia es hacia atrás.

    El signo ya viene resuelto de la placa: un comando positivo sube el ángulo.
    Ver `bringup()`.
    """
    t = df['t'].to_numpy(dtype=float)
    theta = np.deg2rad(df[canal].to_numpy(dtype=float))

    if len(t) < 2:
        return t[1:], np.zeros(0)

    w = np.empty(len(t) - 1)
    if len(t) > 2:
        w[:-1] = (theta[2:] - theta[:-2]) / (t[2:] - t[:-2])
    w[-1] = (theta[-1] - theta[-2]) / (t[-1] - t[-2])

    n = max(1, int(round(ventana / _dt(t))))
    n += (n + 1) % 2                                    # impar: centrado de verdad
    if n > 1:
        # En los extremos la ventana se achica a las muestras que hay. Ni ceros
        # --hunden las puntas, y de ahí salen los regímenes de un escalón-- ni
        # repetir la primera derivada, que es una sola muestra con todo su ruido y
        # con una ventana larga termina pesando en decenas de filas.
        suma = np.concatenate([[0.0], np.cumsum(w)])
        k = np.arange(len(w))
        desde = np.clip(k - n // 2, 0, len(w))
        hasta = np.clip(k + n // 2 + 1, 0, len(w))
        w = (suma[hasta] - suma[desde]) / (hasta - desde)
    return t[1:], w


def normalizar(df, ventana=0.02):
    """La captura en las unidades de un modelo: t [s], u [%], theta [rad], omega [rad/s], i [A].

    La velocidad sale de `velocidad()` con la misma `ventana`, y la primera fila
    --que no tiene velocidad-- se descarta.
    """
    t, w = velocidad(df, ventana=ventana)
    theta = np.deg2rad(df['y_uw'].to_numpy(dtype=float))[1:]

    datos = pd.DataFrame({
        't':     t,
        'u':     df['u'].to_numpy(dtype=float)[1:] * 100.0 / U_MAX,
        'theta': theta,
        'omega': w,
        'i':     df['i'].to_numpy(dtype=float)[1:] / 1000.0,
    })
    datos.attrs['config'] = dict(df.attrs.get('config', {}), ventana=ventana)
    return datos


def guardar(df, ruta, ventana=0.02):
    """Escribe la captura como CSV con las columnas de `normalizar()`. Devuelve la ruta.

    Arriba de las columnas van unas líneas que empiezan con `#` y dicen con qué se
    midió: la placa, la fecha, la `ventana` y las perillas de la captura
    (`df.attrs['config']`). Un modelo ajustado con `cur_filas = 10` y otro con
    `cur_filas = 1` no se comparan igual, y dentro de un mes eso ya no se recuerda.
    Para leerlo con otra cosa que `cargar()`: `pd.read_csv(ruta, comment='#')`.

    Una captura que ya está normalizada --tiene `omega`-- se escribe tal cual.
    """
    ruta = Path(ruta)
    ruta.parent.mkdir(parents=True, exist_ok=True)
    datos = df if 'omega' in df else normalizar(df, ventana=ventana)

    config = dict(datos.attrs.get('config', {}))
    config.setdefault('ventana', ventana)
    encabezado = [f'# guardado: {datetime.now():%Y-%m-%d %H:%M:%S}']
    encabezado += [f'# {clave}: {valor}' for clave, valor in config.items()]

    with open(ruta, 'w', encoding='utf-8', newline='') as f:
        f.write('\n'.join(encabezado) + '\n')
        datos[COLUMNAS].to_csv(f, index=False, float_format='%.6g', lineterminator='\n')
    return ruta


def _valor(texto):
    """Un valor del encabezado, como número si lo es."""
    for tipo in (int, float):
        try:
            return tipo(texto)
        except ValueError:
            pass
    return texto


def cargar(ruta):
    """Lee un CSV escrito por `guardar()`. La configuración queda en `df.attrs['config']`."""
    config = {}
    with open(ruta, encoding='utf-8') as f:
        for linea in f:
            if not linea.startswith('#'):
                break
            clave, _, valor = linea[1:].strip().partition(': ')
            config[clave] = _valor(valor)

    df = pd.read_csv(ruta, comment='#')
    faltan = [c for c in COLUMNAS if c not in df]
    if faltan:
        raise ValueError(f'{ruta}: faltan las columnas {faltan}')
    df.attrs['config'] = config
    return df
