"""Verifica contra la placa de verdad que cada módulo hace lo que dice.

Los otros tests corren sin hardware: prueban la aritmética en la máquina de
escritorio, el protocolo contra un dispositivo inventado, y que los nombres existan.
Lo que ninguno puede probar es que el firmware que está grabado se comporte como el
que se escribió, y eso es justamente lo que un reacomodo grande pone en duda.

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
    from bench import sync_board, MODE_OPEN, MODE_PID, POSITION, CURRENT
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
    #
    # Lo primero es que la placa declare lo que el golden dice que declara. Si esto
    # falla, todo lo que sigue está midiendo otra cosa.
    from test_tablas import leer_tablas
    from ctrllink import LINK_PARAMS

    esperados = {e['nombre'] for e in leer_tablas()['params']} | set(LINK_PARAMS)
    check('la placa declara los parametros que el sketch define',
          set(dev.params) == esperados,
          f'sobran {sorted(set(dev.params) - esperados)}, '
          f'faltan {sorted(esperados - set(dev.params))}')

    canales = [c.name for c in dev.channels]
    check('y los canales', canales == ['ref', 'y_raw', 'y_uw', 'y_uwf', 'e', 'u', 'i'],
          str(canales))

    # ----------------------------------------------------- ida y vuelta de todo
    #
    # Cada perilla tiene que guardar lo que se le escribe. Es la prueba que atrapa
    # una entrada de la tabla con la dirección de su vecino, que es el error que se
    # comete moviendo parámetros de lugar y que no rompe la compilación.
    from catalogo import catalogo_de

    PRUEBA = {
        'pid_kp': 0.25, 'pid_ki': 0.001, 'pid_kd': 0.5, 'pid_alpha': 0.25,
        'ctl_ref': 123, 'ctl_rate': 7, 'ctl_uff': 0,
        'mot_top': 4000, 'mot_bidir': 0, 'mot_invert': 1,
        'ang_offset': 321, 'ang_alpha': 0.5, 'ang_cal': 0, 'ang_sfilt': 2,
        'cur_zero': 2000, 'cur_invert': 1, 'cur_alpha': 0.25,
        'loop_div': 20, 'dec': 2,
    }

    malos = []
    for nombre, valor in PRUEBA.items():
        try:
            quedo = dev.set(nombre, valor)
        except CtrlLinkError as exc:
            malos.append(f'{nombre}: {exc}')
            continue

        escala = dev._params[nombre].scale if nombre in dev._params else 1.0
        if not cerca(float(quedo), float(valor), max(abs(escala), 1e-9)):
            malos.append(f'{nombre}: pedí {valor}, quedó {quedo}')

    check(f'las {len(PRUEBA)} perillas guardan lo que se les escribe', not malos,
          '; '.join(malos))

    # Y que cada una siga leyendo lo mismo después de haber escrito todas las
    # demás: eso es lo que descarta dos entradas apuntando a la misma dirección.
    cruzados = []
    for nombre, valor in PRUEBA.items():
        leido = dev.get(nombre)
        escala = dev._params[nombre].scale if nombre in dev._params else 1.0
        if not cerca(float(leido), float(valor), max(abs(escala), 1e-9)):
            cruzados.append(f'{nombre}: {valor} -> {leido}')

    check('y ninguna se pisa con otra', not cruzados, '; '.join(cruzados))

    # ------------------------------------------------------ el reloj del lazo
    #
    # loop_div divide el muestreador de 5 kHz, así que el período que informa la
    # placa tiene que ser exactamente loop_div / 5000.
    for divisor in (5, 10, 25):
        dev.loop_div = divisor
        check(f'loop_div = {divisor} da el periodo exacto',
              cerca(dev.dt, divisor / 5000.0, 1e-9),
              f'{dev.dt * 1e6:.1f} us contra {divisor / 5.0:.1f}')
    dev.loop_div = 10

    # ------------------------------------------------------------- el puente
    #
    # mot_top es el TOP del Timer1 y la frecuencia sale de f = F_CPU / (2 * top).
    for hz in (1000, 4000, 20000):
        real = dev.pwm(hz)
        check(f'pwm({hz}) queda donde el temporizador puede',
              cerca(dev.pwm_hz, real, 1.0) and abs(real - hz) / hz < 0.01,
              f'pedí {hz}, quedó {real:.0f} Hz con top = {dev.mot_top}')
    dev.pwm(1000)

    # --------------------------------------------------- la medicion de corriente
    #
    # Esto es lo que prueba que CurrentSense hace la resta que dice hacer. Con el
    # filtro apagado, correr el cero N cuentas tiene que correr `i` exactamente
    # -N cuentas, y dar vuelta el signo tiene que negarla.
    dev.rest()
    dev.cur_invert = 0
    dev.cur_alpha = 1.0            # filtro apagado: la cuenta cruda
    lsb = dev.channel('i').scale

    base_zero = 2048
    dev.cur_zero = base_zero

    # Primero cuánto se mueve el canal por su cuenta, sin tocar nada. No es el ruido
    # por muestra --eso se promedia-- sino cuánto se corre el promedio de una captura
    # a la siguiente, que es una deriva lenta y no se promedia. Medido en este banco:
    # un par de cuentas. La tolerancia sale de ahí en lugar de ser un número
    # elegido, porque una tolerancia elegida a mano se termina aflojando hasta que
    # pase.
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
          f'{i0:.1f} -> {i1:.1f} cuentas, diferencia {i1 - i0:+.1f} '
          f'contra {-DELTA} esperada')

    dev.cur_zero = base_zero
    dev.cur_invert = 1
    i2 = dev.capture(0.3, warn=False)['i'].mean() / lsb
    check('cur_invert niega la corriente', cerca(i2, -i0, tol),
          f'{i0:+.1f} -> {i2:+.1f} cuentas')

    dev.cur_invert = 0
    dev.cur_alpha = 0.1667

    # ------------------------------------------------------ el angulo
    #
    # ang_offset es la cuenta que se lee como cero, así que moverlo mueve ang_y en
    # la misma cantidad. Con el eje quieto, que es como corre este test.
    dev.ang_offset = 0
    y0 = dev.ang_y
    dev.ang_offset = 500
    y1 = dev.ang_y
    check('correr ang_offset corre el angulo lo mismo',
          cerca((y1 - y0) % 4096, 500, 4),
          f'{y0} -> {y1}, diferencia {(y1 - y0) % 4096} contra 500')
    dev.ang_offset = 0

    # Y zero() tiene que dejar el angulo en cero, sea cual sea la posicion del eje.
    dev.zero()
    check('zero() deja el angulo del eje en cero', abs(dev.ang_y) <= 4,
          f'ang_y = {dev.ang_y}')

    # ------------------------------------------------- la tabla de calibracion
    #
    # Se empujan las 64 entradas y se verifica con una sola lectura: si la suma que
    # calcula la placa coincide con la de la computadora, las 64 llegaron bien y en
    # el orden correcto.
    import calib

    cal = calib.Calibracion.vacia()
    for k in range(len(cal.lut)):
        cal.lut[k] = (k * 37) % 201 - 100        # algo con los dos signos

    cal.aplicar(dev, verificar=False)
    check('las 64 entradas de la tabla llegan enteras',
          int(dev.ang_lutsum) == cal.checksum(),
          f'placa {int(dev.ang_lutsum):#06x} contra computadora {cal.checksum():#06x}')

    # Y la correccion que hace la placa tiene que ser la misma que la del notebook,
    # cuenta por cuenta. Se comprueba prendiendo cal y comparando el angulo leido
    # contra el que predice la tabla de este lado.
    dev.ang_cal = 1
    crudo = int(dev.capture(0.2, warn=False)['y_raw'].mean() / dev.channel('y_raw').scale)
    dev.ang_offset = 0
    leido = dev.ang_y
    esperado = -((crudo - cal.corregir(crudo)) % 4096)
    check('la correccion de la placa coincide con la del notebook',
          cerca(((leido - esperado) % 4096 + 2048) % 4096 - 2048, 0, 6),
          f'crudo {crudo}, la placa lee {leido}, se esperaba {esperado}')

    dev.ang_cal = 0

    # ---------------------------------------------------------- contarse solo
    texto = dev.describe()
    check('describe() nombra cada parametro que la placa declara',
          all(n in texto for n in dev.params), '')
    check('y la tabla de Jupyter se arma', dev._repr_html_().startswith('<div'))

    # --------------------------------------------------------------- el motor
    if '--motor' in sys.argv:
        print()
        print('--- lo que necesita mover el eje ---')

        dev.rest()
        dev.mot_bidir = 1
        dev.cur_invert = 0

        giro = dev.spin(120)
        check('el eje gira con un comando de lazo abierto',
              abs(giro.vueltas) > 0.05, f'{giro.vueltas:+.2f} vueltas')

        # El defecto que el refactor arregla: cambiar la magnitud realimentada sin
        # cambiar el controlador tiene que olvidar el integrador, porque la suma quedó
        # acumulada en las unidades de la magnitud vieja.
        #
        # La referencia se deja en cero todo el tiempo y el error se fabrica corriendo
        # el cero del ángulo. Eso importa: `ctl_ref` es la misma perilla para los dos
        # objetivos y significa otra cosa en cada uno, así que cambiar el objetivo con
        # una referencia de 180 grados puesta la convierte en 2048 cuentas de corriente
        # y el integrador se vuelve a cargar al instante, por una razón que no tiene
        # nada que ver con lo que se quiere medir.
        #
        # Y se mira con step(), que deja t = 0 exactamente en el cambio: lo que hay que
        # comparar es el comando de un lado y del otro de ese instante, no el promedio
        # de una ventana entera. Después del reinicio el integrador vuelve a cargarse
        # --el eje gira y consume-- así que medio segundo más tarde ya no se vería.
        dev.rest()

        # El cero de la corriente primero. Sin esto `i` marca mil cuentas en reposo
        # --el cero por omisión del sketch es media escala y este sensor reposa en tres
        # cuartos-- así que apenas el lazo pasa a corriente se encuentra con un error
        # enorme y el integrador se vuelve a cargar en siete períodos. Se vería un
        # comando alto de los dos lados del cambio y parecería que el reinicio no
        # ocurrió, cuando lo que pasó es que volvió a pasar lo mismo por otro motivo.
        dev.zero_current()

        dev.gains(kp=0.0, ki=20.0, kd=0.0)
        dev.ang_alpha = 1.0
        dev.ctl_ref = 0
        dev.ctl_target = POSITION

        dev.zero()
        dev.ang_offset = (dev.ang_offset + 1000) % 4096   # 88 grados de error, sin mover nada
        dev.ctl_mode = MODE_PID

        df = dev.step('ctl_target', CURRENT, pre=0.4, post=0.2, warn=False)

        dev.rest()
        dev.gains()
        dev.ang_offset = 0

        # Las primeras muestras de cada lado, no el promedio de una ventana: lo que se
        # está midiendo es el escalón del comando en el instante del cambio, y unas
        # décimas más tarde el integrador ya se cargó de nuevo contra el error nuevo.
        antes   = df['u'][df['t'] < 0].abs().iloc[-10:].mean()
        despues = df['u'][df['t'] >= 0].abs().iloc[:3].mean()

        check('cambiar de objetivo olvida el integrador',
              antes > 20 and despues < antes / 4,
              f'el comando pasa de {antes:.0f} a {despues:.0f} en el instante del cambio')
    else:
        print()
        print('       lo del motor se saltea; --motor lo incluye (mueve el eje)')

    dev.rest()
    print()
    print(f'{len(fallas)} falla(s)' + (': ' + ', '.join(fallas) if fallas else ''))
    return 1 if fallas else 0


if __name__ == '__main__':
    sys.exit(main())
