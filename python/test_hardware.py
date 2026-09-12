"""Verifica contra la placa de verdad que cada módulo hace lo que dice.

Los otros tests corren sin hardware: prueban la aritmética en la máquina de
escritorio, el protocolo contra un dispositivo inventado, y que los nombres existan.
Lo que ninguno puede probar es que el firmware que está grabado se comporte como el
que se escribió.

Así que esto necesita la placa, y lo dice si no está. Cada bloque mueve una perilla
y comprueba que la magnitud que esa perilla gobierna cambie *exactamente* lo que
tiene que cambiar, en lugar de mirar si el número resultante «parece razonable»: un
error de escala o de signo pasa cualquier inspección a ojo y no pasa una resta.

    python test_hardware.py              sin mover el motor
    python test_hardware.py --motor      incluye lo que necesita accionarlo

OJO con --motor: mueve el eje. Revisar que esté libre.
"""

import sys
from pathlib import Path

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
        print('este test la necesita; los demas corren sin ella')
        return 2

    dev.rest()
    print()

    # ------------------------------------------------------------ las tablas
    from test_tablas import leer_tablas

    esperados = {e['nombre'] for e in leer_tablas()['params']} | {'dec'}
    check('la placa declara los parametros que el sketch define',
          set(dev.params) == esperados,
          f'sobran {sorted(set(dev.params) - esperados)}, '
          f'faltan {sorted(esperados - set(dev.params))}')

    canales = [c.name for c in dev.channels]
    check('y los canales', canales == ['y_raw', 'y_uw', 'u', 'i'], str(canales))

    # ----------------------------------------------------- ida y vuelta de todo
    #
    # Cada perilla tiene que guardar lo que se le escribe, y seguir leyendo lo mismo
    # después de haber escrito todas las demás: eso descarta dos entradas de la
    # tabla apuntando a la misma dirección.
    PRUEBA = {'ctl_uff': 0, 'ang_cal': 0, 'cur_zero': 2000, 'loop_div': 20, 'dec': 2}

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

    # ------------------------------------------------------ el reloj del muestreo
    for divisor in (5, 10, 25):
        dev.loop_div = divisor
        check(f'loop_div = {divisor} da el periodo exacto',
              cerca(dev.dt, divisor / 5000.0, 1e-9),
              f'{dev.dt * 1e6:.1f} us contra {divisor / 5.0:.1f}')
    dev.loop_div = 10

    # --------------------------------------------------- la medicion de corriente
    #
    # Correr el cero N cuentas tiene que correr `i` exactamente -N cuentas. La
    # tolerancia sale de cuánto se corre el canal solo entre capturas, que es una
    # deriva lenta y no se promedia.
    dev.rest()
    lsb = dev.channel('i').scale
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
    check('correr cur_zero corre la corriente lo mismo y al reves',
          cerca(i1 - i0, -DELTA, tol),
          f'{i0:.1f} -> {i1:.1f} cuentas, diferencia {i1 - i0:+.1f} contra {-DELTA}')
    dev.cur_zero = base_zero

    # ------------------------------------------------- la tabla de calibracion
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
    check('la correccion de la placa coincide con la del notebook',
          cerca(salto, -int(cal.corregir(crudo)), 2),
          f'crudo {crudo}: el angulo salto {salto:+.0f} cuentas, la tabla dice '
          f'{-int(cal.corregir(crudo)):+d}')

    # ---------------------------------------------------------- contarse solo
    texto = dev.describe()
    check('describe() nombra cada parametro que la placa declara',
          all(n in texto for n in dev.params), '')
    check('y la tabla de Jupyter se arma', dev._repr_html_().startswith('<div'))

    # --------------------------------------------------------------- el motor
    if '--motor' in sys.argv:
        print()
        print('--- lo que necesita mover el eje ---')

        vueltas, pico = dev._tiron(120)
        check('el eje gira con un comando', abs(vueltas) > 0.05,
              f'{vueltas:+.2f} vueltas, {pico:.0f} mA de pico')

        # El comando que sale es el que se pidió, recortado a -255..255.
        dev.ctl_uff = 400
        u = dev.capture(0.1, warn=False)['u']
        dev.rest()
        check('el comando se recorta en 255', (u == 255).all(), str(u.unique()))
    else:
        print()
        print('       lo del motor se saltea; --motor lo incluye (mueve el eje)')

    dev.rest()
    print()
    print(f'{len(fallas)} falla(s)' + (': ' + ', '.join(fallas) if fallas else ''))
    return 1 if fallas else 0


if __name__ == '__main__':
    sys.exit(main())
