"""Que el notebook corra en el entorno del curso, y decirlo claro si no.

Un kernel equivocado es el error más común de quien recién empieza, y el que peor
se explica solo: la primera celda falla con un `ModuleNotFoundError` sobre
`serial` o `control`, y eso invita a instalar paquetes sueltos en el Python que
haya quedado elegido, que suele ser justamente el que no hay que usar. Así que la
primera celda de cada notebook llama a `verificar()` antes de importar nada, y lo
que falte se informa junto con el Python en el que se está corriendo.

Sólo usa la biblioteca estándar: tiene que poder correr en cualquier Python,
incluido el equivocado.

    import entorno
    entorno.verificar()
"""

import importlib.util
import sys
from pathlib import Path

# El entorno de conda del curso. Ver "Instalación en Windows" en el README.
ENTORNO = 'dyc'

# Nombre del paquete de conda -> nombre con el que se importa, cuando difieren.
_IMPORTA_COMO = {'pyserial': 'serial'}

_REQUISITOS = Path(__file__).resolve().parent / 'requirements.txt'


class EntornoIncorrecto(ImportError):
    pass


def paquetes():
    """Los paquetes de requirements.txt, que es la lista de la que se instala."""
    lineas = _REQUISITOS.read_text(encoding='utf-8').splitlines()
    return [l.split('#')[0].strip() for l in lineas if l.split('#')[0].strip()]


def faltantes():
    return [p for p in paquetes()
            if importlib.util.find_spec(_IMPORTA_COMO.get(p, p)) is None]


def verificar():
    """Levanta EntornoIncorrecto si falta algún paquete, explicando qué hacer."""
    faltan = faltantes()
    if not faltan:
        return

    prefijo = Path(sys.prefix)
    en_dyc = prefijo.name == ENTORNO
    lista = ', '.join(faltan)

    if en_dyc:
        consejo = (f'El kernel es el entorno {ENTORNO}, pero le faltan paquetes. '
                   f'En el Miniforge Prompt:\n\n'
                   f'    conda activate {prefijo}\n'
                   f'    conda install -c conda-forge {" ".join(faltan)}\n\n'
                   f'y despues reiniciar el kernel (menu Kernel > Restart Kernel).')
    else:
        consejo = (f'Lo mas probable es que el notebook este usando un Python '
                   f'equivocado, y no el entorno {ENTORNO} del curso.\n'
                   f'Cerrar JupyterLab y volver a abrirlo desde el Miniforge Prompt, '
                   f'con el entorno activado:\n\n'
                   f'    conda activate C:\\envs\\{ENTORNO}\n'
                   f'    jupyter lab\n\n'
                   f'No instalar los paquetes que faltan en este Python.')

    raise EntornoIncorrecto(
        f'\n\nFaltan paquetes: {lista}.\n'
        f'Este notebook esta corriendo con {sys.executable}\n\n'
        f'{consejo}\n'
        f'Ver "Instalacion en Windows, paso a paso" en el README.') from None
