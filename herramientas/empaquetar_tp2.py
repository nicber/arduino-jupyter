"""Arma el zip con lo necesario para hacer el TP2.

    python herramientas\\empaquetar_tp2.py            # deja dist\\arduino-jupyter-tp2.zip
    python herramientas\\empaquetar_tp2.py otro.zip

Toma los archivos que están en git --con el submódulo nI2C adentro, que es justo lo
que el botón "Download ZIP" de GitHub no trae--, así que no se cuela nada que no
sea del proyecto: ni `build/`, ni datos medidos, ni la calibración de un banco. Se
leen del disco, así que un cambio sin commitear entra igual; el script lo avisa, y
anota en `VERSION.txt` de qué commit sale el zip.

Todo va adentro de una carpeta `arduino-jupyter/`, así que descomprimido en
`C:\\envs` queda donde lo espera el README.
"""

import subprocess
import sys
import zipfile
from datetime import datetime
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent
CARPETA = 'arduino-jupyter'

# Lo que hace falta para el TP2. Una entrada que termina en / es una carpeta entera.
INCLUIR = [
    'README.md',
    'PROTOCOL.md',
    'dyc.yml',
    'Banco/',                       # el sketch que graba el notebook
    'AS5600_Bringup/',              # verificación del sensor, si algo falla
    'Puente_Bringup/',              # verificación del actuador, si algo falla
    'libraries/',
    'python/',
    'notebooks/hardware.ipynb',
    'notebooks/calibracion.ipynb',
    'Docs/CALIBRACION_AS5600.md',
    'herramientas/verificar.py',
    '.vscode/extensions.json',      # abrir la carpeta en VS Code y que ofrezca todo
    '.vscode/settings.json',
    '.vscode/tasks.json',
    '.vscode/c_cpp_properties.json',
]

# Adentro de lo incluido, lo que igual sobra. Se busca en la ruta con una / adelante,
# así que '/.vscode/' saca la de cada sketch; la de la raíz entra por su nombre.
EXCLUIR = ('/.vscode/', '/examples/', '.gitignore', '.gitmodules')


def git(*argumentos):
    return subprocess.run(['git', *argumentos], cwd=RAIZ, capture_output=True,
                          encoding='utf-8', check=True).stdout


def archivos():
    """Los archivos de git, submódulos incluidos, que caen adentro de INCLUIR."""
    todos = git('ls-files', '--recurse-submodules').splitlines()
    elegidos = []
    for ruta in todos:
        entra = any(ruta == i or (i.endswith('/') and ruta.startswith(i)) for i in INCLUIR)
        sobra = ruta not in INCLUIR and any(e in f'/{ruta}' for e in EXCLUIR)
        if entra and not sobra:
            elegidos.append(ruta)

    faltan = [i for i in INCLUIR
              if not any(r == i or (i.endswith('/') and r.startswith(i)) for r in elegidos)]
    if faltan:
        raise SystemExit(f'no estan en git: {", ".join(faltan)}. '
                         f'Si es un archivo nuevo, hacer git add antes de empaquetar.')
    return elegidos


def main():
    destino = Path(sys.argv[1]) if len(sys.argv) > 1 else RAIZ / 'dist' / 'arduino-jupyter-tp2.zip'
    destino.parent.mkdir(parents=True, exist_ok=True)

    elegidos = archivos()

    if not (RAIZ / 'libraries' / 'nI2C' / 'nI2C.h').is_file():
        raise SystemExit('falta el submodulo nI2C: correr git submodule update --init')

    commit = git('rev-parse', '--short', 'HEAD').strip()
    sucios = [l[3:] for l in git('status', '--porcelain').splitlines()
              if any(l[3:] == i or (i.endswith('/') and l[3:].startswith(i)) for i in INCLUIR)]
    if sucios:
        print('OJO: hay cambios sin commitear que entran en el zip:')
        for ruta in sucios:
            print(f'  {ruta}')

    version = (f'arduino-jupyter, material del TP2\n'
               f'commit {commit}{" con cambios sin commitear" if sucios else ""}\n'
               f'armado el {datetime.now():%Y-%m-%d %H:%M}\n')

    with zipfile.ZipFile(destino, 'w', zipfile.ZIP_DEFLATED) as zf:
        for ruta in elegidos:
            zf.write(RAIZ / ruta, f'{CARPETA}/{ruta}')
        zf.writestr(f'{CARPETA}/VERSION.txt', version)

    print(f'{destino}: {len(elegidos)} archivos, {destino.stat().st_size / 1024:.0f} kB, '
          f'commit {commit}')


if __name__ == '__main__':
    main()
