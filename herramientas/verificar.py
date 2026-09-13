"""Verifica que la instalación anda, sin la placa: el entorno, la compilación y el Python.

    conda activate C:\\envs\\dyc
    python herramientas\\verificar.py

Hace, en orden, y sigue aunque algo falle para que la lista de fallas salga entera:

1. **El entorno**: que estén los paquetes de `python/requirements.txt`.
2. **La compilación**: que cada sketch del proyecto compile para el UNO, con el
   mismo arduino-cli y las mismas banderas que usa `sync_board()`. No graba nada.
3. **Las pruebas de Python** que no necesitan la placa: `python/test_*.py`, menos
   `test_hardware.py`.
4. **Los notebooks**, enteros, contra el banco simulado.

Los notebooks se corren en este mismo proceso y no en un kernel de Jupyter: lo que
se verifica es su código y el del proyecto, y así no depende de que Jupyter pueda
arrancar un kernel. Se corren sobre una copia en una carpeta temporal, para que lo
que escriben --datos de ensayo, una calibración simulada-- no quede mezclado con las
mediciones de verdad.

La primera compilación tarda uno o dos minutos. Devuelve 0 si todo pasó.
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import traceback
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent
PYTHON = RAIZ / 'python'
NOTEBOOKS = RAIZ / 'notebooks'

# Las que necesitan la placa enchufada.
_PRUEBAS_CON_PLACA = {'test_hardware.py'}

resultados = []


def informar(nombre, ok, detalle=''):
    resultados.append((nombre, ok))
    print(f'  [{"  ok " if ok else "FALLA"}]  {nombre}' + (f'  {detalle}' if detalle else ''),
          flush=True)


def seccion(titulo):
    print(f'\n== {titulo}', flush=True)


def ultimas_lineas(texto, n=15):
    lineas = texto.strip().splitlines()
    return '\n'.join('           ' + l for l in lineas[-n:])


# ------------------------------------------------------------------ el entorno

def verificar_entorno():
    seccion('entorno')
    sys.path.insert(0, str(PYTHON))
    import entorno

    faltan = entorno.faltantes()
    informar('paquetes de requirements.txt', not faltan,
             f'faltan: {", ".join(faltan)}' if faltan else f'Python {sys.version.split()[0]} en {sys.prefix}')
    return not faltan


# ------------------------------------------------------------------ compilar

def sketches():
    """Las carpetas de la raíz que son un sketch: tienen un .ino con su mismo nombre."""
    return sorted(p for p in RAIZ.iterdir()
                  if p.is_dir() and (p / f'{p.name}.ino').is_file())


def verificar_compilacion():
    seccion('compilacion (sin grabar)')
    import bench

    cli = bench._arduino_cli()
    if cli == 'arduino-cli':
        informar('arduino-cli', False,
                 'no se encontro: instalar el Arduino IDE 2 (ver README, paso 1)')
        return
    informar('arduino-cli', True, cli)

    banderas = []
    for prop in bench.BUILD_PROPERTIES:
        banderas += ['--build-property', prop]

    with tempfile.TemporaryDirectory(prefix='verificar-build-') as tmp:
        for sketch in sketches():
            inicio = time.monotonic()
            corrida = subprocess.run(
                [cli, 'compile', '--fqbn', bench.FQBN,
                 '--libraries', str(RAIZ / 'libraries'),
                 '--build-path', str(Path(tmp) / sketch.name)] + banderas + [str(sketch)],
                capture_output=True, encoding='utf-8', errors='replace')
            salida = corrida.stdout + corrida.stderr
            ok = corrida.returncode == 0
            if ok:
                uso = next((l.strip() for l in salida.splitlines()
                            if l.startswith('Sketch uses')), '')
                informar(f'compila {sketch.name}', True,
                         f'{time.monotonic() - inicio:.0f} s. {uso}')
            else:
                if 'platform not installed' in salida:
                    salida += ('\nFalta el soporte para placas AVR: en el Arduino IDE, '
                               'Herramientas > Placa > Gestor de placas > Arduino AVR Boards.')
                informar(f'compila {sketch.name}', False, '\n' + ultimas_lineas(salida))


# ------------------------------------------------------------ pruebas de Python

def verificar_pruebas():
    seccion('pruebas de Python (sin placa)')
    pruebas = sorted(p for p in PYTHON.glob('test_*.py') if p.name not in _PRUEBAS_CON_PLACA)
    entorno = dict(os.environ, PYTHONIOENCODING='utf-8', MPLBACKEND='Agg')

    for prueba in pruebas:
        argumentos = [sys.executable, str(prueba)]
        if prueba.name == 'test_notebooks.py':
            # Los notebooks se corren abajo, sin kernel; acá sólo los nombres.
            argumentos.append('--sin-ejecutar')

        inicio = time.monotonic()
        corrida = subprocess.run(argumentos, cwd=str(PYTHON), env=entorno,
                                 capture_output=True, encoding='utf-8', errors='replace')
        salida = corrida.stdout + corrida.stderr
        ok = corrida.returncode == 0
        detalle = f'{time.monotonic() - inicio:.0f} s'
        if not ok:
            fallas = [l for l in salida.splitlines() if l.startswith('FALLA')]
            detalle += '\n' + ultimas_lineas('\n'.join(fallas) if fallas else salida)
        informar(prueba.name, ok, detalle)


# ------------------------------------------------------------------ notebooks

def correr_notebook(ruta, carpeta):
    """Corre las celdas de código de un notebook en orden. Devuelve (ok, detalle)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    nb = json.loads(ruta.read_text(encoding='utf-8'))
    copia = carpeta / ruta.name
    shutil.copy(ruta, copia)

    anterior = Path.cwd()
    mostrar = plt.show
    plt.show = lambda *a, **k: plt.close('all')
    os.chdir(carpeta)
    espacio = {'__name__': '__main__'}

    try:
        for numero, celda in enumerate(nb['cells']):
            if celda['cell_type'] != 'code':
                continue
            codigo = ''.join(celda['source'])
            try:
                exec(compile(codigo, f'{ruta.name}, celda {numero}', 'exec'), espacio)
            except Exception:
                return False, f'celda {numero}:\n' + ultimas_lineas(traceback.format_exc(), 8)
            finally:
                plt.close('all')
        return True, ''
    finally:
        os.chdir(anterior)
        plt.show = mostrar


def verificar_notebooks():
    seccion('notebooks, contra el banco simulado')
    os.environ['HW_SIMULADO'] = '1'
    os.environ['CALIB_SIMULADO'] = '1'

    import io
    import contextlib

    with tempfile.TemporaryDirectory(prefix='verificar-nb-') as tmp:
        carpeta = Path(tmp) / 'notebooks'
        carpeta.mkdir()
        for ruta in sorted(NOTEBOOKS.glob('*.ipynb')):
            inicio = time.monotonic()
            # Lo que imprimen las celdas no es el resultado de la verificación.
            with contextlib.redirect_stdout(io.StringIO()), \
                 contextlib.redirect_stderr(io.StringIO()):
                ok, detalle = correr_notebook(ruta, carpeta)
            informar(f'corre {ruta.name}', ok,
                     f'{time.monotonic() - inicio:.0f} s' + (f'\n{detalle}' if detalle else ''))


# ------------------------------------------------------------------ todo

def main():
    print(f'verificando {RAIZ}')

    if not verificar_entorno():
        print('\nSin los paquetes no se puede verificar el resto. Lo mas probable es que '
              'el entorno dyc no este activado:\n\n    conda activate C:\\envs\\dyc\n')
        return 1

    verificar_compilacion()
    verificar_pruebas()
    verificar_notebooks()

    fallas = [nombre for nombre, ok in resultados if not ok]
    print()
    if fallas:
        print(f'FALLARON {len(fallas)} de {len(resultados)} verificaciones: {", ".join(fallas)}')
        return 1
    print(f'las {len(resultados)} verificaciones pasaron')
    return 0


if __name__ == '__main__':
    sys.exit(main())
