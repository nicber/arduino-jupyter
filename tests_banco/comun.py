"""Lo compartido por las pruebas de tests_banco/: dónde está el proyecto y cómo se informa.

Estas pruebas no van en el zip del TP2. Prueban el proyecto desde afuera: por omisión
este repositorio, o un zip descomprimido con ARDUINO_JUPYTER:

    set ARDUINO_JUPYTER=C:\\ruta\\a\\arduino-jupyter      (por omisión, este repositorio)
    python tests_banco/correr.py
"""
import os
import sys
from pathlib import Path

AQUI = Path(__file__).resolve().parent
RAIZ = Path(os.environ.get('ARDUINO_JUPYTER', AQUI.parent)).resolve()
PYTHON = RAIZ / 'python'

if str(PYTHON) not in sys.path:
    sys.path.insert(0, str(PYTHON))
if str(AQUI) not in sys.path:
    sys.path.insert(0, str(AQUI))

fallas = 0


def check(que, ok, detalle=''):
    """Informa una verificación. `detalle` explica una falla, así que sólo sale con ella."""
    global fallas
    if ok:
        print(f'PASA   {que}')
    else:
        fallas += 1
        print(f'FALLA  {que}' + (f'  -- {detalle}' if detalle else ''))
    return ok


def terminar():
    print(f'\n{fallas} falla(s)')
    sys.exit(1 if fallas else 0)
