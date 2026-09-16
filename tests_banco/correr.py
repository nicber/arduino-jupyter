"""Corre todas las pruebas de tests_banco/ contra el proyecto de ARDUINO_JUPYTER.

    python tests_banco/correr.py

Las de Python corren siempre. Las de C++ se compilan y corren si hay un g++ o clang
de escritorio; si no, se verifica al menos que compilen con el avr-g++ del Arduino
IDE (sólo las que no usan la biblioteca estándar de C++, que avr-libc no trae). Son
cuatro: cpp/test_modulos_extra.cpp de acá, test/test_modulos.cpp del proyecto, y las
dos que viven con su biblioteca, libraries/ControlMath/test y libraries/Control/test.

En Windows no hay g++ de fábrica; con conda alcanza `conda create -p C:/envs/cxx -c
conda-forge m2w64-toolchain` y agregar `C:/envs/cxx/Library/mingw-w64/bin` al PATH.
"""
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
from comun import RAIZ  # noqa: E402

resultados = []


def correr(nombre, argv, cwd):
    corrida = subprocess.run(argv, cwd=cwd, capture_output=True, encoding='utf-8', errors='replace',
                             env=dict(os.environ, PYTHONIOENCODING='utf-8', MPLBACKEND='Agg',
                                      ARDUINO_JUPYTER=str(RAIZ)))
    salida = corrida.stdout + corrida.stderr
    fallas = [l for l in salida.splitlines() if l.startswith('FALLA')]
    ok = corrida.returncode == 0
    resultados.append((nombre, ok))
    print(f'[{" ok " if ok else "FALLA"}] {nombre}')
    for l in (fallas or ([] if ok else salida.strip().splitlines()[-8:])):
        print('        ' + l)


print(f'proyecto: {RAIZ}\n')

for prueba in ('test_bringup_y_divisor.py', 'test_consistencia.py'):
    correr(prueba, [sys.executable, str(AQUI / prueba)], AQUI)

def _inc(*libs):
    return sum((['-I', str(RAIZ / 'libraries' / l / 'src')] for l in libs), [])


incluir = {
    'extra': ['-I', str(AQUI / 'cpp')] + _inc('Actuator', 'Sampler', 'AngleSensor', 'Sense'),
    'proyecto': _inc('Calibracion', 'AngleSensor', 'Sense'),
    # Las dos suites que viven con su biblioteca. No las corría nadie: ni esto ni
    # verificar.py, así que cubrían el PID, la rampa y el filtro sólo en teoría.
    'controlmath': _inc('ControlMath'),
    'control': _inc('Control', 'ControlMath', 'AngleSensor', 'Sense'),
}
fuentes = {
    'extra': AQUI / 'cpp' / 'test_modulos_extra.cpp',
    'proyecto': RAIZ / 'test' / 'test_modulos.cpp',
    'controlmath': RAIZ / 'libraries' / 'ControlMath' / 'test' / 'test_controlmath.cpp',
    'control': RAIZ / 'libraries' / 'Control' / 'test' / 'test_control.cpp',
}

cxx = shutil.which('g++') or shutil.which('clang++')
if cxx:
    with tempfile.TemporaryDirectory() as tmp:
        for clave, fuente in fuentes.items():
            exe = Path(tmp) / f'{clave}.exe'
            # _USE_MATH_DEFINES: con -std=c++11 el mingw define __STRICT_ANSI__ y
            # esconde M_PI, que avr-g++ sí da. Es la única diferencia que apareció
            # entre compilar estas pruebas con el AVR y con un g++ de escritorio.
            compila = subprocess.run([cxx, '-std=c++11', '-O2', '-Wall', '-D_USE_MATH_DEFINES']
                                     + incluir[clave]
                                     + [str(fuente), '-o', str(exe)], capture_output=True, text=True)
            if compila.returncode:
                resultados.append((fuente.name, False))
                print(f'[FALLA] compila {fuente.name}\n' + compila.stderr[-800:])
                continue
            correr(fuente.name, [str(exe)], tmp)
else:
    avr = next(Path(os.environ.get('LOCALAPPDATA', '')).glob(
        'Arduino15/packages/arduino/tools/avr-gcc/*/bin/avr-g++.exe'), None)
    if avr is None:
        print('[ nota] no hay compilador C++: las pruebas de C++ no se verificaron')
    else:
        print('[ nota] no hay g++ ni clang de escritorio: las pruebas de C++ sólo se compilan con '
              'avr-g++, no se corren')
        corrida = subprocess.run([str(avr), '-std=gnu++11', '-fsyntax-only', '-Wall', '-mmcu=atmega328p']
                                 + incluir['extra'] + [str(fuentes['extra'])],
                                 capture_output=True, text=True)
        ok = corrida.returncode == 0
        resultados.append(('compila test_modulos_extra.cpp (avr-g++)', ok))
        print(f'[{" ok " if ok else "FALLA"}] compila test_modulos_extra.cpp (avr-g++)')
        if not ok:
            print(corrida.stderr[-800:])

malas = [n for n, ok in resultados if not ok]
print(f'\n{len(resultados) - len(malas)} de {len(resultados)} pasaron')
sys.exit(1 if malas else 0)
