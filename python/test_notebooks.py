"""Todo lo que los notebooks le piden al banco tiene que existir.

Este test nació de un error concreto. Renombrar los parámetros de la placa tocó los
accesos por atributo --`dev.pid_kp`-- pero no los que viajan como cadena:
`dev.step('uff', 200)` quedó nombrando un parámetro que ya no existe, y eso no se ve
leyendo el diff. Se ve cuando alguien corre la celda, que en este proyecto suele ser
en clase.

Así que se verifica, sin placa y sin abrir Jupyter. Cada `dev.algo` de cada celda de
código tiene que ser un parámetro de la placa o algo que el banco sepa hacer, y cada
nombre que se le pase a `step()`, `set()` o `get()` tiene que ser un parámetro. Un
nombre inventado falla acá en lugar de fallar adelante de un curso.

Del lado de la placa la tabla se lee del sketch, con el mismo parser que
`test_tablas.py`, así que esto no necesita compilar ni conectar nada.

Y además corre de verdad los notebooks que pueden correr sin placa --los que usan
`conseguir_banco()`, que cae al banco simulado-- porque es la única forma de atrapar
un error que no está en un nombre: un gráfico que se queda sin columna, una celda que
depende de otra que se movió. Tarda unos segundos, así que va por omisión;
`--sin-ejecutar` lo saltea.

    python test_notebooks.py
    python test_notebooks.py --sin-ejecutar
"""

import ast
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

_AQUI     = Path(__file__).resolve().parent
NOTEBOOKS = sorted((_AQUI.parent / 'notebooks').glob('*.ipynb'))

sys.path.insert(0, str(_AQUI))

from test_tablas import leer_tablas

# Los nombres que recibe un parámetro por cadena.
_TOMAN_NOMBRE = ('step', 'set', 'get')


def _celdas(ruta):
    """Los árboles de sintaxis de las celdas de código que se puedan parsear.

    Una celda con magias de IPython --las que empiezan con % o !-- no es Python
    válido por sí sola. Se saltea en lugar de fallar: no hay nada que este test
    pueda verificar ahí, y hacerlo fallar sería un falso positivo.
    """
    with open(ruta, encoding='utf-8') as fh:
        nb = json.load(fh)

    for numero, celda in enumerate(nb['cells']):
        if celda['cell_type'] != 'code':
            continue
        try:
            yield numero, ast.parse(''.join(celda['source']))
        except SyntaxError:
            continue


def _pedidos(arbol):
    """Lo que una celda le pide al banco: (atributos, nombres pasados como cadena)."""
    atributos, cadenas = set(), set()

    for nodo in ast.walk(arbol):
        # dev.algo / banco.algo
        if (isinstance(nodo, ast.Attribute)
                and isinstance(nodo.value, ast.Name)
                and nodo.value.id in ('dev', 'banco')):
            atributos.add(nodo.attr)

        # dev.step('nombre', ...) y sus dos hermanas.
        #
        # Tiene que ser una llamada sobre el banco y no cualquier `.get()`: en la
        # celda de arranque hay un `os.environ.get('HW_SIMULADO')` que no tiene nada
        # que ver con los parámetros de la placa.
        if (isinstance(nodo, ast.Call)
                and isinstance(nodo.func, ast.Attribute)
                and nodo.func.attr in _TOMAN_NOMBRE
                and isinstance(nodo.func.value, ast.Name)
                and nodo.func.value.id in ('dev', 'banco')
                and nodo.args
                and isinstance(nodo.args[0], ast.Constant)
                and isinstance(nodo.args[0].value, str)):
            cadenas.add(nodo.args[0].value)

        # events=[(segundos, 'nombre', valor)]
        if isinstance(nodo, ast.keyword) and nodo.arg == 'events':
            for elemento in ast.walk(nodo.value):
                if (isinstance(elemento, ast.Tuple) and len(elemento.elts) == 3
                        and isinstance(elemento.elts[1], ast.Constant)
                        and isinstance(elemento.elts[1].value, str)):
                    cadenas.add(elemento.elts[1].value)

    return atributos, cadenas


def _simulable(ruta):
    """Si el notebook puede correr sin placa.

    Se deduce de que use `conseguir_banco()`, que es la función que cae al banco
    simulado cuando no hay cable. Los que llaman a `sync_board()` en cada celda
    necesitan la placa y no se pueden ejecutar acá; para ésos alcanza con la
    verificación de nombres de arriba.
    """
    with open(ruta, encoding='utf-8') as fh:
        return 'conseguir_banco' in fh.read()


def _ejecutar(ruta):
    """Corre el notebook con el banco simulado. Devuelve la lista de errores."""
    entorno = dict(os.environ, HW_SIMULADO='1', CALIB_SIMULADO='1')

    with tempfile.TemporaryDirectory() as tmp:
        salida = Path(tmp) / 'salida.ipynb'
        corrida = subprocess.run(
            [sys.executable, '-m', 'jupyter', 'nbconvert', '--to', 'notebook',
             '--execute', '--output', str(salida), ruta.name],
            cwd=str(ruta.parent), env=entorno,
            capture_output=True, text=True, timeout=900)

        if not salida.exists():
            return [f'no se pudo ejecutar: {corrida.stderr.strip()[-300:]}']

        with open(salida, encoding='utf-8') as fh:
            nb = json.load(fh)

    return [f"celda {i}: {o['ename']}: {o.get('evalue', '')[:120]}"
            for i, celda in enumerate(nb['cells'])
            for o in celda.get('outputs', [])
            if o.get('output_type') == 'error']


def main():
    from banco_simulado import BancoSimulado
    import bench

    # Los parámetros que la placa declara, más los que administra el enlace.
    from ctrllink import LINK_PARAMS
    parametros = {e['nombre'] for e in leer_tablas()['params']} | set(LINK_PARAMS)

    # Y lo que un banco sabe hacer, de las dos clases: el simulado es el que corre
    # sin cable, y Bench el que corre con él. Un nombre que exista en una sola
    # tampoco sirve, así que se piden las dos.
    banco = BancoSimulado()
    sabe_hacer = ({n for n in dir(banco) if not n.startswith('_')}
                  & {n for n in dir(bench.Bench) if not n.startswith('_')})

    # Lo que Bench delega al enlace no aparece en dir(Bench), así que se agrega lo
    # que el enlace ofrece.
    sabe_hacer |= {n for n in dir(bench.CtrlLink) if not n.startswith('_')}

    conocidos = parametros | sabe_hacer
    fallas = 0

    for ruta in NOTEBOOKS:
        malos_attr, malos_str = set(), set()

        for numero, arbol in _celdas(ruta):
            atributos, cadenas = _pedidos(arbol)
            malos_attr |= {f'c{numero}:{a}' for a in atributos if a not in conocidos}
            malos_str  |= {f'c{numero}:{s}' for s in cadenas if s not in parametros}

        for etiqueta, malos in (('atributos', malos_attr),
                                ('nombres pasados como cadena', malos_str)):
            ok = not malos
            fallas += 0 if ok else 1
            print(f'{"PASA  " if ok else "FALLA "} {ruta.name}: {etiqueta}'
                  + ('' if ok else f'  -- no existen: {sorted(malos)}'))

    # Y correrlos de verdad, los que puedan.
    if '--sin-ejecutar' not in sys.argv:
        for ruta in NOTEBOOKS:
            if not _simulable(ruta):
                print(f'       {ruta.name}: necesita la placa, no se ejecuta')
                continue

            errores = _ejecutar(ruta)
            ok = not errores
            fallas += 0 if ok else 1
            print(f'{"PASA  " if ok else "FALLA "} {ruta.name}: corre entero '
                  f'contra el banco simulado'
                  + ('' if ok else '\n         ' + '\n         '.join(errores)))

    print(f'\n{fallas} falla(s)')
    return 1 if fallas else 0


if __name__ == '__main__':
    sys.exit(main())
