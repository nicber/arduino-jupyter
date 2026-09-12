"""El catálogo tiene que explicar exactamente lo que la placa declara.

Un catálogo escrito a mano al lado de una tabla que vive en el firmware envejece
solo, y de la peor manera: la fila sigue apareciendo con una explicación que ya no
corresponde, que es peor que no tener explicación. Así que se compara.

Del lado de la placa la tabla se lee del sketch --con el mismo parser que
`test_tablas.py`-- así que esto corre sin hardware y sin compilar. Del lado de la
computadora se lee `catalogo.py`. Sobrar y faltar son las dos fallas.

Se verifica también el banco simulado, porque es el que corre cuando el cable no
está: si le falta una perilla que la placa tiene, la celda que la usa se cae en
clase y no antes.

    python test_catalogo.py
"""

import sys
from pathlib import Path

_AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(_AQUI))

import catalogo
from test_tablas import leer_tablas


def main():
    fallas = 0

    def check(ok, etiqueta, detalle=''):
        nonlocal fallas
        print(f'{"PASA  " if ok else "FALLA "} {etiqueta}'
              + ('' if ok or not detalle else f'  -- {detalle}'))
        fallas += 0 if ok else 1

    tablas = leer_tablas()

    # El enlace administra `dec` por su cuenta, así que no está en la tabla del
    # sketch pero sí es un parámetro que se ve desde Python.
    de_la_placa = {e['nombre'] for e in tablas['params']} | {'dec'}
    del_catalogo = catalogo.nombres_conocidos()

    faltan = sorted(de_la_placa - del_catalogo)
    sobran = sorted(del_catalogo - de_la_placa)

    check(not faltan, 'el catalogo explica todos los parametros de la placa',
          f'sin explicacion: {faltan}')
    check(not sobran, 'y no explica ninguno que no exista',
          f'en el catalogo pero no en la placa: {sobran}')

    # Cada entrada completa. Una unidad vacía se acepta --hay parámetros que no
    # tienen unidad, como una suma de verificación-- pero la clase y el texto no.
    for nombre in sorted(del_catalogo):
        clase, _unidad, que_es = catalogo.catalogo_de(nombre)
        check(clase in ('perilla', 'lectura', 'cuenta'),
              f'{nombre}: la clase es una de las tres', repr(clase))
        check(bool(que_es), f'{nombre}: tiene una linea de que es')

    # Los grupos tienen que cubrir todo: un nombre sin prefijo conocido cae en el
    # último grupo, así que lo que se verifica es que ninguno se pierda.
    agrupados = [n for _t, miembros in catalogo.por_grupo(sorted(del_catalogo))
                 for n in miembros]
    check(sorted(agrupados) == sorted(del_catalogo),
          'ningun parametro se pierde al agrupar',
          f'{len(agrupados)} agrupados de {len(del_catalogo)}')

    # Y el banco simulado, que es el que corre sin el cable.
    from banco_simulado import BancoSimulado

    banco = BancoSimulado()
    le_faltan = sorted(de_la_placa - set(banco._nombres()))
    check(not le_faltan, 'el banco simulado tiene todos los parametros de la placa',
          f'le faltan: {le_faltan}')

    # Las dos vistas tienen que armarse sin explotar, con el simulado y con un
    # filtro. Es una prueba de humo, pero es la que atrapa un f-string roto en una
    # rama que nadie mira hasta que alguien pone `dev` en una celda.
    check('ang_offset' in banco.describe(), 'describe() nombra las perillas')
    check('ang_offset' in banco.describe('ang'), 'y el filtro deja pasar su grupo')
    check('pid_kp' not in banco.describe('ang'), 'y saca los demas')
    check(banco._repr_html_().startswith('<div'), 'la tabla de Jupyter se arma')
    check('<table' in banco._repr_html_(), 'y tiene una tabla adentro')

    print(f'\n{fallas} falla(s)')
    return 1 if fallas else 0


if __name__ == '__main__':
    sys.exit(main())
