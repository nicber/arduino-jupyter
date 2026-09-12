"""La tabla de parámetros y la de canales son la interfaz pública del sketch.

Los notebooks las usan como atributos de Python --`dev.ctl_uff`, `df['y_uw']`-- así
que un nombre que cambia, una entrada que se reordena o un `frac` que queda con el
del vecino rompen el otro lado en silencio. No hay ningún test que mire eso: los de
ctrllink prueban el protocolo contra un dispositivo inventado, y los de calib
prueban la aritmética de la tabla de calibración.

Éste compara las dos tablas contra un golden guardado en el repositorio. Lee el
sketch en lugar de preguntarle a una placa, así corre sin hardware y sin compilar,
que es lo que hace que sirva de red durante un reacomodo grande.

    python test_tablas.py             verifica contra el golden
    python test_tablas.py --guardar   vuelve a escribir el golden

Cuando una entrada cambia a propósito, `--guardar` y el diff del golden quedan en
el commit: es la manera de que un cambio de interfaz se vea en la revisión en
lugar de descubrirse desde un notebook.
"""

import json
import re
import sys
from pathlib import Path

_AQUI   = Path(__file__).resolve().parent
SKETCH  = _AQUI.parent / 'Banco' / 'Banco.ino'
GOLDEN  = _AQUI / 'tablas_golden.json'

# Una entrada de tabla es una línea entre llaves con campos separados por comas. Se
# parte por comas de nivel cero para no cortar adentro de un `Foo<a, b>::FRAC` ni de
# una llamada con argumentos.
_ENTRADA = re.compile(r'^\s*\{(.+)\}\s*,\s*$')


def _campos(cuerpo):
    """Los campos de una entrada, partiendo sólo por las comas de nivel cero.

    Sólo los paréntesis y los corchetes cuentan como anidamiento. Los ángulos no:
    en estas tablas hay desplazamientos --`1 << REF_FRAC`-- y contar cada `<` como
    una apertura dejaría el resto de la línea en un nivel del que no se sale.
    """
    campos = []
    actual = ''
    hondo  = 0

    for c in cuerpo:
        if c in '([':
            hondo += 1
        elif c in ')]':
            hondo -= 1

        if c == ',' and hondo == 0:
            campos.append(actual.strip())
            actual = ''
        else:
            actual += c

    if actual.strip():
        campos.append(actual.strip())
    return campos


def _tabla(texto, declaracion):
    """Las entradas de la tabla que arranca en `declaracion`, como listas de campos.

    Se corta en la primera llave de cierre a nivel de línea, que es como termina un
    inicializador de arreglo en este sketch. Los comentarios se descartan antes,
    porque adentro de las tablas hay varios y algunos llevan comas.
    """
    inicio = texto.index(declaracion)
    resto  = texto[inicio:]
    fin    = resto.index('\n};')

    entradas = []
    for linea in resto[:fin].split('\n'):
        linea = re.sub(r'//.*$', '', linea)
        m = _ENTRADA.match(linea)
        if m:
            entradas.append(_campos(m.group(1)))
    return entradas


def leer_tablas(ruta=SKETCH):
    """Los parámetros y los canales del sketch, como los vería la computadora.

    De los parámetros interesa el nombre, el tipo y el `frac`, que es lo que decide
    cómo se convierte el valor. La dirección no: es interna, y moverla de una global
    a un miembro de una clase es justamente lo que se quiere poder hacer sin que
    este test se queje.
    """
    texto = ruta.read_text(encoding='utf-8')

    params = [{'nombre': n.strip('"'), 'tipo': t, 'frac': f}
              for n, t, _addr, f in _tabla(texto, 'CtrlParam PROGMEM')]

    chans = [{'nombre': n.strip('"'), 'tipo': t, 'escala': e, 'unidad': u.strip('"')}
             for n, t, _addr, e, u in _tabla(texto, 'CtrlChannel PROGMEM')]

    return {'params': params, 'chans': chans}


# ------------------------------------------------------------------ el test

def _contar(fallas, ok, etiqueta, detalle=''):
    print(f'{"PASA  " if ok else "FALLA "} {etiqueta}' + (f'  -- {detalle}' if detalle else ''))
    return fallas + (0 if ok else 1)


def main():
    tablas = leer_tablas()

    if '--guardar' in sys.argv:
        GOLDEN.write_text(json.dumps(tablas, indent=2, ensure_ascii=False) + '\n',
                          encoding='utf-8')
        print(f'golden escrito: {len(tablas["params"])} parametros, '
              f'{len(tablas["chans"])} canales')
        return 0

    if not GOLDEN.exists():
        print(f'no hay golden en {GOLDEN}; correr con --guardar')
        return 1

    golden = json.loads(GOLDEN.read_text(encoding='utf-8'))
    fallas = 0

    # El nombre y el orden a la vez: la computadora lee la tabla por orden y guarda
    # los nombres, así que las dos cosas son la interfaz.
    for clave in ('params', 'chans'):
        viejos = [e['nombre'] for e in golden[clave]]
        nuevos = [e['nombre'] for e in tablas[clave]]

        fallas = _contar(fallas, viejos == nuevos, f'los nombres de {clave} y su orden',
                         '' if viejos == nuevos else
                         f'faltan {sorted(set(viejos) - set(nuevos))}, '
                         f'sobran {sorted(set(nuevos) - set(viejos))}')

        porNombre = {e['nombre']: e for e in tablas[clave]}
        for esperado in golden[clave]:
            hay = porNombre.get(esperado['nombre'])
            if hay is None:
                continue
            fallas = _contar(fallas, hay == esperado,
                             f'{clave}: {esperado["nombre"]} conserva su formato',
                             '' if hay == esperado else f'{esperado} -> {hay}')

    # El ancho de una fila de telemetría, que es lo que decide si entra en el buffer
    # de transmisión. Lo recalcula el dispositivo, pero acá se ve sin grabar nada.
    ancho = {'CTRL_I8': 2, 'CTRL_U8': 2, 'CTRL_I16': 4, 'CTRL_U16': 4}
    fila  = 4 + 1 + sum(ancho.get(c['tipo'], 8) for c in tablas['chans'])
    fallas = _contar(fallas, fila <= 63, 'la fila entra en el buffer de transmision',
                     f'{fila} bytes de 63')

    print(f'\n{fallas} falla(s)')
    return 1 if fallas else 0


if __name__ == '__main__':
    sys.exit(main())
