"""Verifica contra la placa real que cada módulo hace lo que dice.

Los otros tests corren sin hardware: prueban la aritmética en la máquina de
escritorio, el protocolo contra un dispositivo simulado, y que los nombres existan.
Lo que ninguno puede probar es que el firmware que está grabado se comporte como el
que se escribió.

Así que esto necesita la placa, y lo dice si no está. Cada bloque mueve una perilla
y comprueba que la magnitud que esa perilla gobierna cambie *exactamente* lo que
tiene que cambiar, en lugar de mirar si el número resultante «parece razonable»: un
error de escala o de signo pasa una inspección visual y no pasa una resta.

    python test_hardware.py              sin mover el motor
    python test_hardware.py --motor      incluye lo que necesita accionarlo

Atención: --motor mueve el eje. Revisar que esté libre.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

fallas = []


def check(etiqueta, ok, detalle=''):
    print(f'{"PASA  " if ok else "FALLA "} {etiqueta}'
          + (f'  -- {detalle}' if detalle else ''))
    if not ok:
        fallas.append(etiqueta)


def cerca(a, b, tol):
    return abs(a - b) <= tol


def main():
    from bench import sync_board
    from ctrllink import CtrlLinkError

    try:
        dev = sync_board()
    except Exception as exc:
        print(f'no hay placa: {type(exc).__name__}: {exc}')
        print('este test la necesita; los demás corren sin ella')
        return 2

    dev.rest()
    print()

    # ------------------------------------------------------------ las tablas
    from test_tablas import leer_tablas
    from ctrllink import LINK_PARAMS

    esperados = {e['nombre'] for e in leer_tablas()['params']} | set(LINK_PARAMS)
    check('la placa declara los parámetros que el sketch define',
          set(dev.params) == esperados,
          f'sobran {sorted(set(dev.params) - esperados)}, '
          f'faltan {sorted(esperados - set(dev.params))}')

    canales = [c.name for c in dev.channels]
    check('y los canales', canales == ['y_raw', 'y_uw', 'y_rep', 'u', 'i'], str(canales))

    # ----------------------------------------------------- ida y vuelta de todo
    #
    # Cada perilla tiene que guardar lo que se le escribe, y seguir leyendo lo mismo
    # después de haber escrito todas las demás: eso descarta dos entradas de la
    # tabla apuntando a la misma dirección.
    PRUEBA = {'ctl_uff': 0, 'mot_bidir': 1, 'ang_inv': 1, 'ang_cal': 0,
              'cur_zero': 2000, 'cur_inv': 1, 'cur_filas': 7, 'loop_div': 20, 'dec': 2}

    malos = []
    for nombre, valor in PRUEBA.items():
        try:
            quedo = dev.set(nombre, valor)
        except CtrlLinkError as exc:
            malos.append(f'{nombre}: {exc}')
            continue
        if not cerca(float(quedo), float(valor), 1e-9):
            malos.append(f'{nombre}: pedí {valor}, quedó {quedo}')

    check(f'las {len(PRUEBA)} perillas guardan lo que se les escribe', not malos,
          '; '.join(malos))

    cruzados = [f'{n}: {v} -> {dev.get(n)}' for n, v in PRUEBA.items()
                if not cerca(float(dev.get(n)), float(v), 1e-9)]
    check('y ninguna se pisa con otra', not cruzados, '; '.join(cruzados))
    dev.dec = 1

    # Lo que sigue compara contra la medición cruda, así que sin los signos del
    # banco y en un cuadrante. Cada signo se prueba en su bloque.
    dev.ang_inv = dev.cur_inv = dev.mot_bidir = 0
    dev.cur_filas = 10

    # ------------------------------------------------ el promedio de la corriente
    # Con el ADC libre, más filas en la ventana tienen que bajar el ruido como la
    # raíz de las conversiones: de 1 a 10 filas, unas tres veces. Y cur_filas se
    # recorta a 1..32. Sin la media de 4 ticks ni el notch de 250 Hz, que ya bajan el
    # ruido de cada fila y dejan poco para promediar: lo que se prueba es la ventana.
    #
    # El desvío de la diferencia entre filas consecutivas, sobre raíz de dos, y no el
    # desvío a secas: el cero de este canal vagabundea por debajo de unos pocos hertz
    # --se corrió 7 cuentas en un minuto, medido-- y una ventana más larga no lo
    # promedia, así que el desvío a secas mide sobre todo esa deriva y da un número que
    # no se repite. Medido alternando las dos configuraciones: el desvío a secas con 10
    # filas da entre 1,0 y 7,1 mA de captura a captura, y así medido, entre 0,37 y
    # 0,39.
    dev.rest()
    filtros = {n: dev.get(n) for n in ('cur_ma', 'cur_nyq') if n in dev.link._params}
    for n in filtros:
        dev.set(n, 0)

    def ruido_de_banda(filas):
        dev.cur_filas = filas
        i = dev.capture(1.0, warn=False, canales=['i'])['i'].to_numpy()
        return float(np.diff(i).std() / 2 ** 0.5)

    r1 = ruido_de_banda(1)
    r10 = ruido_de_banda(10)
    for n, v in filtros.items():
        dev.set(n, v)
    check('10 filas de promedio bajan el ruido de la corriente', r10 < r1 / 2,
          f'{r1:.2f} mA con 1 fila, {r10:.2f} mA con 10')
    dev.cur_filas = 100
    check('cur_filas se recorta a 32', dev.cur_filas == 32, str(dev.cur_filas))
    dev.cur_filas = 10

    # ------------------------------------------------------ el reloj del muestreo
    for divisor in (5, 10, 25):
        dev.loop_div = divisor
        check(f'loop_div = {divisor} da el período exacto',
              cerca(dev.dt, divisor / 5000.0, 1e-9),
              f'{dev.dt * 1e6:.1f} µs contra {divisor * 200:.1f}')
    dev.loop_div = 10

    # --------------------------------------------------- la medición de corriente
    #
    # Correr el cero N cuentas tiene que correr `i` exactamente -N cuentas. La
    # tolerancia sale de cuánto se corre el canal solo entre capturas, que es una
    # deriva lenta y no se promedia.
    dev.rest()
    # En cuentas del conversor, que es la unidad de `cur_zero`: el canal puede
    # publicar fracciones de cuenta (`cur_frac`). Ver Bench._ma_por_cuenta().
    lsb = dev._ma_por_cuenta()
    base_zero = 2048
    dev.cur_zero = base_zero

    medidas = [dev.capture(0.3, warn=False)['i'].mean() / lsb for _ in range(3)]
    vaiven = max(medidas) - min(medidas)
    tol = max(3.0, 3 * vaiven)
    print(f'       el canal de corriente se corre {vaiven:.1f} cuentas entre capturas; '
          f'se tolera {tol:.1f}')

    i0 = sum(medidas) / len(medidas)
    DELTA = 100
    dev.cur_zero = base_zero + DELTA
    i1 = dev.capture(0.3, warn=False)['i'].mean() / lsb
    check('correr cur_zero corre la corriente lo mismo y al revés',
          cerca(i1 - i0, -DELTA, tol),
          f'{i0:.1f} -> {i1:.1f} cuentas, diferencia {i1 - i0:+.1f} contra {-DELTA}')

    # Con el cero corrido la corriente queda lejos de cero, que es lo que hace
    # visible el signo.
    dev.cur_inv = 1
    i2 = dev.capture(0.3, warn=False)['i'].mean() / lsb
    dev.cur_inv = 0
    check('cur_inv invierte el signo de la corriente, cero incluido', cerca(i2, -i1, tol),
          f'{i1:.1f} -> {i2:.1f} cuentas')
    dev.cur_zero = base_zero

    # ------------------------------------------------------------- el ángulo
    # Con el eje quieto: invertir el signo tiene que invertir el desenrollado
    # y no tocar la cuenta cruda.
    a0 = dev.capture(0.2, warn=False)
    dev.ang_inv = 1
    a1 = dev.capture(0.2, warn=False)
    dev.ang_inv = 0
    escala = dev.channel('y_uw').scale
    check('ang_inv invierte y_uw y deja y_raw',
          cerca(a1['y_uw'].median(), -a0['y_uw'].median(), 2 * escala)
          and cerca(a1['y_raw'].median(), a0['y_raw'].median(), 2 * escala),
          f'y_uw {a0["y_uw"].median():+.2f} -> {a1["y_uw"].median():+.2f} grados')
    check('y_rep marca pocas filas con el eje quieto', a0['y_rep'].mean() < 0.01,
          f'{a0["y_rep"].sum()} de {len(a0)}')

    # ------------------------------------------------- la tabla de calibración
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'extras' / 'calibracion_as5600'))
    import calib

    cal = calib.Calibracion.vacia()
    for k in range(len(cal.lut)):
        cal.lut[k] = (k * 37) % 201 - 100        # algo con los dos signos

    cal.aplicar(dev, verificar=False)
    check('las 64 entradas de la tabla llegan enteras',
          int(dev.ang_lutsum) == cal.checksum(),
          f'placa {int(dev.ang_lutsum):#06x} contra computadora {cal.checksum():#06x}')

    # Con el eje quieto, prender la corrección tiene que mover el ángulo desenrollado
    # exactamente lo que dice la tabla para la cuenta cruda de ahora. aplicar() la
    # deja prendida, así que se la apaga primero.
    dev.ang_cal = 0
    df0 = dev.capture(0.2, warn=False)
    dev.ang_cal = 1
    df1 = dev.capture(0.2, warn=False)
    dev.ang_cal = 0

    escala = dev.channel('y_raw').scale
    crudo = int(round(df1['y_raw'].median() / escala))
    salto = (df1['y_uw'].median() - df0['y_uw'].median()) / escala
    check('la corrección de la placa coincide con la del notebook',
          cerca(salto, -int(cal.corregir(crudo)), 2),
          f'crudo {crudo}: el ángulo saltó {salto:+.0f} cuentas, la tabla dice '
          f'{-int(cal.corregir(crudo)):+d}')

    # ------------------------------------------------------- autodescripción
    texto = dev.describe()
    check('describe() nombra cada parámetro que la placa declara',
          all(n in texto for n in dev.params), '')
    check('y la tabla de Jupyter se arma', dev._repr_html_().startswith('<div'))

    # --------------------------------------------------------------- el motor
    if '--motor' in sys.argv:
        print()
        print('--- lo que necesita mover el eje ---')

        vueltas, df = dev._tiron(120)
        check('el eje gira con un comando', abs(vueltas) > 0.05,
              f'{vueltas:+.2f} vueltas, {df["i"].abs().max():.0f} mA de pico')

        # El comando que sale es el que se pidió, recortado a -255..255, o a 0..255
        # en un solo cuadrante. Con bidir en 1 el eje se mueve: en un banco B′,
        # para el mismo lado.
        for bidir, pedido, sale in ((0, 400, 255), (0, -120, 0), (1, -400, -255)):
            dev.mot_bidir = bidir
            dev.ctl_uff = pedido
            u = dev.capture(0.1, warn=False)['u']
            dev.rest()
            check(f'con mot_bidir = {bidir}, {pedido:+d} sale como {sale:+d}',
                  (u == sale).all(), str(u.unique()))
        dev.mot_bidir = 0
    else:
        print()
        print('       lo del motor se saltea; --motor lo incluye (mueve el eje)')

    dev.rest()
    print()
    print(f'{len(fallas)} falla(s)' + (': ' + ', '.join(fallas) if fallas else ''))
    return 1 if fallas else 0


if __name__ == '__main__':
    sys.exit(main())
