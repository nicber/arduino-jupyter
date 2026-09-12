"""El banco simulado tiene que cubrir todo lo que los notebooks le piden al real.

`BancoSimulado` no hereda de `Bench` ni comparte contrato con él: reimplementa a mano
la superficie que los notebooks usan, y eso es a propósito --lo que se muestra en
clase tiene que correr sin la placa, y hacer que ese archivo importe el del enlace
serie lo ataría a pyserial para nada--. El precio es que el día que `Bench` gane un
método, el simulado no falla: calla, y las celdas que corren sin placa dejan de
probar lo que creen probar.

Este test cobra ese precio. En lugar de una lista escrita a mano, que envejece igual
que el simulado, saca la superficie de los propios notebooks: todo lo que aparezca
como `dev.algo` o `banco.algo` en una celda de código tiene que existir en el banco
simulado. Así el contrato se actualiza solo cuando alguien escribe una celda nueva.

    python test_simulado.py
"""

import ast
import json
import sys
from pathlib import Path

_AQUI      = Path(__file__).resolve().parent
NOTEBOOKS  = sorted((_AQUI.parent / 'notebooks').glob('*.ipynb'))

# Lo que se le pide al objeto del banco pero no es de él. `simulado` lo pone el
# propio simulado, y los notebooks lo consultan justamente para saber si hay placa.
_AJENOS = frozenset()


def _atributos_pedidos(ruta):
    """Los nombres que las celdas de código piden sobre `dev` o `banco`.

    Se parsea con ast y no con una expresión regular: `dev.deg(90)` y
    `df.attrs['dev']` se parecen lo suficiente como para confundir a un patrón, y
    lo que hace falta es exactamente un acceso a atributo sobre ese nombre.

    Una celda que no sea Python válida por sí sola --las que empiezan con un `%`
    de IPython, o un fragmento partido entre celdas-- se saltea: no aporta nada que
    este test pueda verificar y hacer fallar por eso sería un falso positivo.
    """
    with open(ruta, encoding='utf-8') as fh:
        nb = json.load(fh)

    pedidos = set()

    for celda in nb['cells']:
        if celda['cell_type'] != 'code':
            continue

        fuente = ''.join(celda['source'])

        try:
            arbol = ast.parse(fuente)
        except SyntaxError:
            continue

        for nodo in ast.walk(arbol):
            if (isinstance(nodo, ast.Attribute)
                    and isinstance(nodo.value, ast.Name)
                    and nodo.value.id in ('dev', 'banco')):
                pedidos.add(nodo.attr)

    return pedidos


def main():
    sys.path.insert(0, str(_AQUI))
    from banco_simulado import BancoSimulado

    banco = BancoSimulado()
    fallas = 0
    total = 0

    for ruta in NOTEBOOKS:
        pedidos = _atributos_pedidos(ruta) - _AJENOS
        faltan = sorted(n for n in pedidos if not hasattr(banco, n))
        total += len(pedidos)

        ok = not faltan
        fallas += 0 if ok else 1
        print(f'{"PASA  " if ok else "FALLA "} {ruta.name}: '
              f'{len(pedidos)} atributos pedidos'
              + ('' if ok else f'  -- faltan {faltan}'))

    # Y al revés no se verifica a propósito. El simulado puede tener cosas que
    # ningún notebook use todavía; lo que no puede es que falte algo que sí se use.
    print(f'\n{total} atributos verificados contra el banco simulado')
    print(f'{fallas} falla(s)')
    return 1 if fallas else 0


if __name__ == '__main__':
    sys.exit(main())
