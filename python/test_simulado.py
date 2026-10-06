"""El banco simulado tiene que cubrir todo lo que los notebooks le piden al real.

`BancoSimulado` no hereda de `Bench`: casi todo lo de `Bench` pasa por el enlace serie,
y el simulado no tiene uno, así que reimplementa a mano la superficie que los
notebooks usan y toma de `Bench` sólo lo que no toca el enlace. La contrapartida es
que si `Bench` gana un método, el simulado no falla ni avisa, y las celdas que corren
sin placa dejan de probar lo que se supone que prueban.

Este test cubre esa contrapartida. En lugar de una lista escrita a mano, que se
desactualiza igual que el simulado, saca la superficie de los propios notebooks:
todo lo que aparezca como `dev.algo` o `banco.algo` en una celda de código tiene que existir en el banco
simulado. Así el contrato se actualiza solo cuando alguien escribe una celda nueva.

    python test_simulado.py
"""

import ast
import json
import sys
from pathlib import Path

_AQUI      = Path(__file__).resolve().parent
NOTEBOOKS  = sorted((_AQUI.parent / 'notebooks').glob('*.ipynb')) + \
            sorted((_AQUI.parent / 'extras').glob('*/*.ipynb'))


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
    import numpy as np
    from banco_simulado import BancoSimulado

    banco = BancoSimulado()
    fallas = 0
    total = 0

    for ruta in NOTEBOOKS:
        pedidos = _atributos_pedidos(ruta)
        faltan = sorted(n for n in pedidos if not hasattr(banco, n))
        total += len(pedidos)

        ok = not faltan
        fallas += 0 if ok else 1
        print(f'{"PASA  " if ok else "FALLA "} {ruta.name}: '
              f'{len(pedidos)} atributos pedidos'
              + ('' if ok else f'  -- faltan {faltan}'))

    # El tiempo de las filas, como lo arma la placa: `tick` avanza una vez por
    # período de control, `dec` emite una fila cada tantos, y `t` es tick * dt_us.
    banco.dec = 4
    df = banco.capture(0.4, warn=False)
    banco.dec = 1
    paso = 4 * banco.dt
    tick = df.attrs['tick']
    ok = (len(df) == round(0.4 / paso)
          and set(np.diff(tick)) == {4}
          and np.allclose(df['t'], tick * df.attrs['dt_us'] * 1e-6))
    fallas += 0 if ok else 1
    print(f'{"PASA  " if ok else "FALLA "} con dec = 4, una fila cada 4 ticks y t = tick * dt_us'
          + ('' if ok else f'  -- {len(df)} filas, pasos de tick {sorted(set(np.diff(tick)))}'))

    # Y al revés no se verifica a propósito. El simulado puede tener cosas que
    # ningún notebook use todavía; lo que no puede es que falte algo que sí se use.
    print(f'\n{total} atributos verificados contra el banco simulado')
    print(f'{fallas} falla(s)')
    return 1 if fallas else 0


if __name__ == '__main__':
    sys.exit(main())
