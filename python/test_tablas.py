"""La tabla de parámetros y la de canales son la interfaz pública del sketch.

Los notebooks las usan como atributos de Python --`dev.ctl_uff`, `df['y_uw']`-- así
que un nombre que cambia, una entrada que se reordena o un `frac` que queda con el
del vecino rompen el otro lado sin ningún aviso. Ninguna otra prueba mira eso: las de
ctrllink prueban el protocolo contra un dispositivo simulado, y las de
extras/calibracion_as5600/test_calib.py la aritmética de la tabla de calibración.

Éste compara las dos tablas contra un golden guardado en el repositorio. Lee el
sketch en lugar de preguntarle a una placa, así corre sin hardware y sin compilar,
y sirve de control durante una reorganización del sketch.

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

    # Lo que está adentro de un `#if SENSE_DIAG` no es de la interfaz: son las perillas
    # de diagnóstico del conversor, que se compilan afuera salvo que se las pida. Ver
    # SENSE_DIAG en Sense/RowAdc.h.
    entradas = []
    diagnostico = 0
    for linea in resto[:fin].split('\n'):
        linea = re.sub(r'//.*$', '', linea)
        if re.match(r'\s*#\s*if\s+SENSE_DIAG', linea):
            diagnostico += 1
            continue
        if diagnostico and re.match(r'\s*#\s*(endif|else)\b', linea):
            diagnostico -= 1
            continue
        if diagnostico:
            continue
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


def nombres_de_parametros(ruta=SKETCH):
    """Los parámetros que se ven desde Python: los de la tabla del sketch, más los
    que administra el enlace por su cuenta (`LINK_PARAMS` en ctrllink.py)."""
    from ctrllink import LINK_PARAMS
    return {e['nombre'] for e in leer_tablas(ruta)['params']} | set(LINK_PARAMS)


# ------------------------------------------------------------------ el test

def _contar(fallas, ok, etiqueta, detalle=''):
    print(f'{"PASA  " if ok else "FALLA "} {etiqueta}' + (f'  -- {detalle}' if detalle else ''))
    return fallas + (0 if ok else 1)


def main():
    tablas = leer_tablas()

    if '--guardar' in sys.argv:
        GOLDEN.write_text(json.dumps(tablas, indent=2, ensure_ascii=False) + '\n',
                          encoding='utf-8')
        print(f'golden escrito: {len(tablas["params"])} parámetros, '
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
    # de transmisión. Lo recalcula el dispositivo, pero acá se ve sin grabar nada. El
    # buffer es el que compila placa.py (64 si no lo cambia), y es un anillo: le entra
    # un byte menos que su tamaño.
    import placa
    m = re.search(r'-DSERIAL_TX_BUFFER_SIZE=(\d+)', ' '.join(placa.BUILD_PROPERTIES))
    lugar = (int(m.group(1)) if m else 64) - 1
    ancho = {'CTRL_I8': 2, 'CTRL_U8': 2, 'CTRL_I16': 4, 'CTRL_U16': 4}
    fila  = 4 + 1 + sum(ancho.get(c['tipo'], 8) for c in tablas['chans'])
    fallas = _contar(fallas, fila <= lugar, 'la fila entra en el buffer de transmisión',
                     f'{fila} bytes de {lugar}')

    print(f'\n{fallas} falla(s)')
    return 1 if fallas else 0


if __name__ == '__main__':
    sys.exit(main())
