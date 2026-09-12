"""Qué significa cada parámetro de la placa, y cómo mostrarlo.

Es la mitad del banco que está escrita para una persona: la placa declara su tabla
al conectarse --nombre, tipo y escala, que es lo que la aritmética necesita-- y acá
está lo que hace falta para entenderla. Unidad, si se puede mover o sólo mirar, y una
línea de qué es.

Vive en un módulo propio y sin una sola dependencia, y eso importa por dos razones.
Lo usan las dos puntas, el banco de verdad y el simulado, y el simulado tiene que
poder correr sin pyserial para que la clase se pueda dar con el cable desenchufado.

Que no se desactualice no depende de la buena voluntad: `test_catalogo.py` compara
esta tabla contra la del sketch y falla si sobra o falta una entrada. Es la misma
idea que el golden de `test_tablas.py`.
"""

# Los valores que toman los dos parámetros que eligen qué hace el lazo. Viven acá y
# no en bench.py porque son parte de lo que significa la tabla de la placa, y porque
# este módulo no depende de nada: un notebook los importa sin arrastrar el puerto
# serie, y el banco simulado los usa igual que el de verdad.
MODE_OPEN, MODE_PID, MODE_RAMP = 0, 1, 2
POSITION,  CURRENT             = 0, 1


# Cada entrada es (clase, unidad, qué es). La clase va primera porque es lo primero
# que alguien quiere saber: contesta «¿esto lo puedo mover?».
#
#   perilla   se fija; es una decisión de quien hace el experimento
#   lectura   la placa la publica; escribirla no significa nada
#   cuenta    un total acumulado; ponerla en cero empieza a contar de nuevo
_CATALOGO = {
    # La ley de control. Las ganancias son POR MUESTRA, que es lo que hace la
    # aritmética; `dev.gains()` las toma en tiempo continuo y convierte.
    'pid_kp':      ('perilla', 'por muestra', 'ganancia proporcional: cuánto comando por cuenta de error'),
    'pid_ki':      ('perilla', 'por muestra', 'ganancia integral, sobre la suma de errores'),
    'pid_kd':      ('perilla', 'por muestra', 'ganancia derivativa, sobre el error ya filtrado'),
    'pid_alpha':   ('perilla', '0 a 1',       'polo del filtro que suaviza el error antes de derivarlo'),

    # Qué se le pide al lazo.
    'ctl_ref':     ('perilla', 'del objetivo', 'a dónde se le pide que vaya'),
    'ctl_rate':    ('perilla', 'del objetivo por período', 'pendiente de la rampa; `dev.rev_per_s()` convierte'),
    'ctl_uff':     ('perilla', '-255 a 255',  'comando de lazo abierto, o prealimentación en lazo cerrado'),
    'ctl_mode':    ('perilla', '0, 1, 2',     'qué controlador gobierna: MODE_OPEN, MODE_PID, MODE_RAMP'),
    'ctl_target':  ('perilla', '0, 1',        'sobre qué magnitud cierra el lazo: POSITION o CURRENT'),

    # El puente. Los tres describen el banco y no el programa: `bringup()` los mide.
    'mot_top':     ('perilla', 'cuentas',     'TOP del Timer1: f = 16 MHz / (2 * top); `dev.pwm()` toma Hz'),
    'mot_bidir':   ('perilla', '0, 1',        '1 si el puente acciona en los dos sentidos'),
    'mot_invert':  ('perilla', '0, 1',        '1 si un comando positivo hace *bajar* el ángulo medido'),

    # El sensor de ángulo y su calibración.
    'ang_offset':  ('perilla', 'cuentas',     'la cuenta que se lee como cero; `dev.zero()` la fija'),
    'ang_alpha':   ('perilla', '0 a 1',       'polo del filtro de posición; 1 lo apaga'),
    'ang_cal':     ('perilla', '0, 1',        '1 si la corrección de la tabla está aplicada'),
    'ang_sfilt':   ('perilla', '0 a 3',       'filtro interno del AS5600: 3 es el más rápido, 0,29 ms'),
    'ang_lutw':    ('perilla', 'empaquetado', 'una entrada de la tabla: (índice << 16) | valor'),
    'ang_lutsum':  ('lectura', '',            'suma de Fletcher de la tabla: verifica las 64 con una lectura'),
    'ang_y':       ('lectura', 'cuentas',     'el ángulo de adentro de la vuelta, referido a ang_offset'),
    'ang_y_uw':    ('lectura', 'cuentas',     'el mismo ángulo desenrollado, que no salta al dar la vuelta'),
    'ang_status':  ('lectura', 'bits',        'STATUS del AS5600: imán detectado, muy débil, muy fuerte'),
    'ang_present': ('lectura', '0, 1',        '0 si el sensor no contesta en el bus I2C'),
    'ang_agc':     ('lectura', '0 a 255',     'ganancia con la que lee; contra un extremo, imán mal montado'),
    'ang_mag':     ('lectura', 'cuentas',     'módulo del vector de campo que ve el sensor'),
    'ang_busovr':  ('cuenta',  'muestras',    'muestras que el bus I2C no llegó a seguir'),
    'ang_buserr':  ('cuenta',  'transferencias', 'transferencias del sensor que fallaron'),

    # La medición de corriente.
    'cur_zero':    ('perilla', 'cuentas ADC', 'el cero del sensor; `dev.zero_current()` lo mide'),
    'cur_invert':  ('perilla', '0, 1',        '1 si la corriente sale negativa con un comando positivo'),
    'cur_alpha':   ('perilla', '0 a 1',       'polo del filtro de corriente; 1 lo apaga'),
    'cur_ma_lsb':  ('lectura', 'mA/cuenta',   'la escala real de esta placa, calculada al arrancar'),

    # Lo que la placa mide de sí misma, para no tener que adivinar cuál es.
    'board_adcfs': ('lectura', 'cuentas',     'fondo de escala del conversor: 1024 en el UNO, 4096 en el clon'),
    'board_bgadc': ('lectura', 'cuentas',     'el canal interno medido contra Vcc'),
    'board_bus':   ('lectura', 'bits',        'estado eléctrico de SDA y SCL antes de encender el TWI'),

    # El reloj del lazo.
    'loop_div':    ('perilla', 'muestras',    'muestras de 5 kHz por período: 10 son 500 Hz, 5 son 1 kHz'),
    'loop_late':   ('cuenta',  'us',          'peor retardo entre el disparo de un tick y su atención'),
    'loop_missed': ('cuenta',  'períodos',    'períodos de control que el lazo nunca atendió'),

    # Del enlace y no del sketch: éste lo administra CtrlLink.
    'dec':         ('perilla', 'períodos',    'emitir una fila cada tantos períodos, para no saturar el cable'),
}

# El título de cada grupo, en el orden en que conviene leerlos: primero lo que se
# mueve para hacer un experimento, después lo que se mira.
_GRUPOS = (
    ('ctl',   'Qué se le pide al lazo'),
    ('pid',   'La ley de control'),
    ('mot',   'El puente'),
    ('ang',   'El sensor de ángulo'),
    ('cur',   'La medición de corriente'),
    ('loop',  'El reloj del lazo'),
    ('board', 'La placa, medida por sí misma'),
    ('',      'El enlace'),
)


def catalogo_de(nombre):
    """Lo que se sabe de un parámetro: (clase, unidad, qué es).

    Devuelve una entrada vacía si no está, y no levanta excepción a propósito: un
    sketch puede declarar un parámetro que este archivo todavía no conozca, y eso
    tiene que salir como una fila sin explicación y no como una celda que no corre.
    De que la tabla esté completa se encarga `test_catalogo.py`.
    """
    return _CATALOGO.get(nombre, ('', '', ''))


def nombres_conocidos():
    """Los nombres que este catálogo explica. Lo usa el test que lo verifica."""
    return set(_CATALOGO)


def por_grupo(nombres):
    """Los nombres repartidos por grupo, en el orden de _GRUPOS.

    Un nombre que no caiga en ningún prefijo conocido va al último grupo, así que no
    desaparece de la lista por no haber sido previsto.
    """
    quedan = list(nombres)
    salida = []

    for prefijo, titulo in _GRUPOS:
        miembros = ([n for n in quedan if n.startswith(prefijo + '_')] if prefijo
                    else list(quedan))
        quedan = [n for n in quedan if n not in miembros]

        if miembros:
            salida.append((titulo, sorted(miembros)))

    return salida


# ------------------------------------------------------------------- la vista
#
# Las dos formas de mostrar lo mismo: texto para una terminal y una tabla para
# Jupyter. Las dos toman los mismos datos en lugar de un objeto, así que sirven igual
# para el banco de verdad y para el simulado, y ninguna de las dos puntas tiene que
# saber de la otra.
#
#   encabezado  qué dispositivo es, en una línea
#   resumen     la segunda línea: frecuencias, cuántos canales
#   nombres     los parámetros que este dispositivo declara
#   valor_de    nombre -> texto con el valor de ahora
#   canales     [(nombre, escala, unidad)], o vacío

_AYUDA = (
    'para convertir unidades: gains(kp,ki,kd)  deg()  ma()  rev_per_s()  '
    'as_deg()  as_ma()  smooth()  pwm()  zero()  zero_current()',
    'para verificar el equipo: rest()  spin(u)  bringup()',
    'para medir: capture(segundos)  step(parametro, valor)',
)


def texto(encabezado, resumen, nombres, valor_de, canales=()):
    """La descripción entera como texto plano."""
    lineas = [encabezado, '', resumen]

    for titulo, miembros in por_grupo(nombres):
        lineas += ['', titulo, '-' * len(titulo)]

        for nombre in miembros:
            clase, unidad, que_es = catalogo_de(nombre)
            lineas.append(f'  {nombre:<12} {valor_de(nombre):>12}  '
                          f'{unidad:<14} {clase:<8} {que_es}')

    if canales:
        lineas += ['', 'los canales que devuelve capture(), con su escala:']
        lineas += [f'  {n:<7} {escala:>12.5f} {unidad} por cuenta'
                   for n, escala, unidad in canales]

    return '\n'.join(lineas + [''] + list(_AYUDA))


def html(encabezado, resumen, nombres, valor_de, canales=()):
    """La misma descripción como tabla, que es lo que Jupyter muestra.

    Se arma a mano y no con pandas para no arrastrar el estilo de un DataFrame, que
    en una tabla de cuarenta filas de texto se lee peor que una lista. Los colores
    son los de la paleta de los notebooks, y los tres de `clase` son el punto: de un
    vistazo se ve qué se puede mover.
    """
    from html import escape

    COLOR = {'perilla': '#2a78d6', 'lectura': '#52514e', 'cuenta': '#eda100'}

    def celda(x, **estilo):
        css = ';'.join(f'{k.replace("_", "-")}:{v}' for k, v in estilo.items())
        return f'<td style="{css};padding:2px 12px 2px 0">{escape(str(x))}</td>'

    partes = [f'<div style="font-weight:600">{escape(encabezado)}</div>',
              f'<div style="color:#52514e">{escape(resumen)}</div>',
              '<table style="border-collapse:collapse;font-size:90%">']

    for titulo, miembros in por_grupo(nombres):
        partes.append('<tr><td colspan="5" style="padding:12px 0 2px;'
                      f'font-weight:600">{escape(titulo)}</td></tr>')

        for nombre in miembros:
            clase, unidad, que_es = catalogo_de(nombre)
            partes.append(
                '<tr>'
                + celda(nombre, font_family='monospace')
                + celda(valor_de(nombre), font_family='monospace', text_align='right')
                + celda(unidad, color='#52514e')
                + celda(clase, color=COLOR.get(clase, '#52514e'))
                + celda(que_es)
                + '</tr>')

    partes.append('</table>')

    if canales:
        partes.append('<div style="color:#52514e;padding-top:10px">canales de '
                      + escape(', '.join(f'{n} ({u})' for n, _e, u in canales))
                      + '</div>')

    partes += [f'<div style="color:#52514e;padding-top:4px">{escape(l)}</div>'
               for l in _AYUDA]
    return ''.join(partes)
