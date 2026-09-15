"""La placa: compilar el sketch, grabarlo y encontrar el puerto.

Todo lo que habla con arduino-cli y con la carpeta `build/`. No sabe nada del banco
ni del protocolo: `bench.sync_board()` lo usa y después abre el enlace.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

from ctrllink import CtrlLinkError, find_port

FQBN = 'arduino:avr:uno'

# Con qué perfil *cargar*. Compilar es siempre lo mismo --el binario es el mismo
# ATmega328P a 16 MHz en todos los casos--, pero el bootloader que lo recibe no:
# un UNO escucha a 115200 y muchos clones baratos traen el bootloader antiguo del
# Nano, que escucha a 57600. Elegir mal no da un error legible sino diez líneas
# de «not in sync». Así que se prueban en orden y se recuerda cuál funcionó, por
# puerto, en `sync-state.json`.
UPLOAD_FQBNS = [
    ('arduino:avr:uno',                    'UNO'),
    ('arduino:avr:nano:cpu=atmega328old',  'clon con bootloader antiguo'),
]

_HERE     = Path(__file__).resolve().parent.parent
LIBRARIES = _HERE / 'libraries'
BUILD_DIR = _HERE / 'build'

# El sketch que se graba si no se pide otro. `sync_board(sketch=...)` acepta el
# nombre de una carpeta del repositorio --'Banco'-- o una ruta.
SKETCH    = _HERE / 'Banco'

# El core de AVR compila con `-Os` --optimizar por tamaño--, y este sketch quiere
# ciclos y no bytes: el muestreador de 5 kHz y la telemetría comparten el tiempo
# con el manejador del bus. `-O2` es `-Os` más todo lo que agrande el código, y no
# `-O3`, que en un AVR de 32 kB se paga con casi todo el espacio que queda.
#
# Las banderas van como `extra_flags` y no reemplazando `compiler.*.flags` porque
# el recipe las pega después de las propias del core, y la última `-O` de la línea
# es la que manda: así se hereda todo lo demás en lugar de copiarlo a mano. El
# enlace también lleva `-O2`, porque con `-flto` el grueso de la generación de
# código pasa ahí.
BUILD_PROPERTIES = [
    'compiler.c.extra_flags=-O2',
    'compiler.cpp.extra_flags=-O2',
    'compiler.c.elf.extra_flags=-O2',
]

# ------------------------------------------------------- compilación y carga

def _sketch_dir(sketch=None):
    """La carpeta del sketch: `SKETCH` si no se pide otro, una carpeta del
    repositorio si se da un nombre, o la ruta tal cual si se da una ruta.
    """
    if sketch is None:
        ruta = SKETCH
    elif isinstance(sketch, Path) or '/' in sketch or '\\' in sketch:
        ruta = Path(sketch).resolve()
    else:
        ruta = _HERE / sketch

    if not (ruta / f'{ruta.name}.ino').exists():
        raise ValueError(f'no hay ningún sketch en {ruta}: falta {ruta.name}.ino')
    return ruta


def _sources_hash(sketch):
    """Huella digital de todo aquello a partir de lo cual se construye el sketch.

    Por contenido y no por marca de tiempo: un checkout de git reescribe las
    mtime sin cambiar una línea, y si no dispararía una recompilación innecesaria.
    Las banderas de compilación entran en la huella junto con las fuentes: un
    `build/` que quedó de una corrida con otra optimización tiene las mismas
    fuentes y un binario que ya no es el que corresponde.
    """
    digest = hashlib.sha256()
    digest.update(repr(BUILD_PROPERTIES).encode())
    files = sorted(list(sketch.glob('*.ino')) + list(sketch.glob('*.h')) +
                   [p for p in LIBRARIES.rglob('*') if p.suffix in ('.h', '.cpp', '.c')])
    for path in files:
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _arduino_cli():
    """La ruta del Arduino CLI: el del PATH si hay uno, y si no el que trae el IDE.

    El IDE 2 compila con un arduino-cli propio, guardado adentro de su instalación
    y fuera del PATH. Buscarlo ahí es lo que permite que alcance con instalar el
    IDE, sin tocar variables de entorno. Los dos comparten los cores instalados,
    así que el core de AVR que baja el IDE sirve igual.
    """
    found = shutil.which('arduino-cli')
    if found:
        return found

    backend = Path('resources', 'app', 'lib', 'backend', 'resources')
    candidates = [Path('/Applications/Arduino IDE.app/Contents/Resources/app/lib/'
                       'backend/resources/arduino-cli')]
    for var, sub in (('LOCALAPPDATA', Path('Programs', 'arduino-ide')),
                     ('ProgramFiles', Path('Arduino IDE'))):
        if os.environ.get(var):
            candidates.append(Path(os.environ[var]) / sub / backend / 'arduino-cli.exe')

    for path in candidates:
        if path.is_file():
            return str(path)
    return 'arduino-cli'        # que _run() explique que no está


def _run(argv, what):
    """Corre una herramienta de compilación y devuelve su salida, o explica por qué no pudo.

    La codificación se fija en lugar de dejarla al locale: arduino-cli emite
    UTF-8, y una consola de Windows con cp1252 por omisión convierte un carácter
    perdido en un diagnóstico del compilador en un UnicodeDecodeError que esconde
    el error real.
    """
    try:
        done = subprocess.run(argv, capture_output=True,
                              encoding='utf-8', errors='replace')
    except FileNotFoundError:
        raise RuntimeError(
            f'no se encontró arduino-cli, así que no se puede {what} el sketch.\n'
            f'Instalar el Arduino IDE (https://www.arduino.cc/en/software) y abrirlo '
            f'una vez. Si está instalado en un lugar poco común, instalar también el '
            f'Arduino CLI y reiniciar el editor, porque el PATH se lee una sola vez '
            f'al arrancar.'
        ) from None

    if done.returncode:
        output = (done.stdout + done.stderr).strip()
        if 'platform not installed' in output:
            output += ('\n\nFalta el soporte para placas AVR. Abrir el Arduino IDE, '
                       'ir a Herramientas > Placa > Gestor de placas, buscar '
                       '"Arduino AVR Boards" e instalarlo.')
        raise RuntimeError(f'falló al {what}:\n{output}')
    return done.stdout + done.stderr


def _load_state():
    try:
        return json.loads((BUILD_DIR / 'sync-state.json').read_text())
    except (OSError, ValueError):
        return {}


def _save_state(state):
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    (BUILD_DIR / 'sync-state.json').write_text(json.dumps(state, indent=1))


def _upload(port, state, say, sketch, build):
    """Carga el binario, averiguando sola con qué bootloader habla esta placa.

    Devuelve el FQBN que funcionó, y lo deja anotado en el estado para la próxima
    vez. Si la anotación quedó desactualizada --se cambió la placa de puerto, o el puerto
    de placa-- el intento falla y se sigue con los otros perfiles, así que la
    memoria acelera pero no decide.
    """
    recordado = state.get('bootloader', {}).get(port)
    orden = ([f for f in UPLOAD_FQBNS if f[0] == recordado] +
             [f for f in UPLOAD_FQBNS if f[0] != recordado])

    fallas = []
    for fqbn, nombre in orden:
        if fallas:
            say(f'  no era {dict(UPLOAD_FQBNS)[fallas[-1][0]]}; probando '
                f'{nombre} ...')
        try:
            _run([_arduino_cli(), 'upload', '--fqbn', fqbn, '-p', port,
                  '--input-dir', str(build), str(sketch)], 'cargar')
        except RuntimeError as exc:
            fallas.append((fqbn, exc))
            continue

        state.setdefault('bootloader', {})[port] = fqbn
        _save_state(state)
        return fqbn

    detalle = '\n\n'.join(f'--- como {dict(UPLOAD_FQBNS)[f]}:\n{e}'
                          for f, e in fallas)
    raise RuntimeError(
        f'no se pudo cargar el sketch en {port} con ninguno de los bootloaders '
        f'conocidos ({", ".join(n for _, n in UPLOAD_FQBNS)}).\n'
        f'«not in sync» en todos suele ser la placa tomada por otro programa --el '
        f'monitor serie del IDE, un kernel de una sesión anterior-- o un cable de sólo '
        f'alimentación. Si la placa es de un tipo que no está en la lista, '
        f'agregarlo a UPLOAD_FQBNS en placa.py.\n\n{detalle}')


def _wait_for_port(hint=None, timeout=2.0):
    """find_port(), pero tolerante con una placa que todavía se está reenumerando.

    Por omisión la espera es corta porque una placa ausente tiene que informarse
    enseguida; la espera larga sólo vale la pena justo después de una carga.
    """
    deadline = time.monotonic() + timeout
    while True:
        try:
            return find_port(hint)
        except CtrlLinkError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.3)


def poner_al_dia(port=None, force_compile=False, force_upload=False, sketch=None,
                 say=print, antes_de_cargar=lambda: None):
    """Compila si cambió alguna fuente y graba si cambió el binario. Devuelve (puerto, notas).

    Compila sólo cuando algún archivo fuente cambió efectivamente y carga sólo cuando
    el binario resultante difiere del que este puerto recibió por última vez.
    `antes_de_cargar` se llama justo antes de grabar, para soltar el puerto: la
    carga lo necesita para sí sola.

    `sketch` elige qué se graba: el nombre de una carpeta del repositorio o una
    ruta. Por omisión `SKETCH`. Cada sketch compila en su propia carpeta de
    `build/`, así que alternar entre dos no recompila ninguno.
    """
    sketch = _sketch_dir(sketch)
    build = BUILD_DIR / sketch.name

    state = _load_state()
    hex_file = build / f'{sketch.name}.ino.hex'
    sources = _sources_hash(sketch)
    notes = []

    # `sources` guarda una huella por sketch; un estado con otro formato se
    # descarta y cuesta una compilación.
    if not isinstance(state.get('sources'), dict):
        state['sources'] = {}

    if (force_compile or not hex_file.exists()
            or state['sources'].get(sketch.name) != sources):
        build_flags = []
        for prop in BUILD_PROPERTIES:
            build_flags += ['--build-property', prop]

        _run([_arduino_cli(), 'compile', '--fqbn', FQBN,
              '--libraries', str(LIBRARIES),
              '--build-path', str(build)] +
             build_flags +
             [str(sketch)], 'compilar')
        notes.append('compilado')
        state['sources'][sketch.name] = sources
        _save_state(state)

    binary = hashlib.sha256(hex_file.read_bytes()).hexdigest()

    if port is None:
        try:
            port = _wait_for_port()
        except CtrlLinkError:
            raise CtrlLinkError(
                'el sketch está compilado, pero no hay ninguna placa alcanzable: '
                'no se encontró ningún puerto serie USB. Enchufarla y correr esto '
                'de nuevo; la compilación está en caché, así que va a pasar directamente '
                'a la carga.') from None

    uploaded = state.get('uploaded', {})

    if force_upload or uploaded.get(port) != binary:
        antes_de_cargar()
        perfil = _upload(port, state, say, sketch, build)
        notes.append('cargado' if perfil == UPLOAD_FQBNS[0][0] else
                     f'cargado como {dict(UPLOAD_FQBNS)[perfil]}')
        uploaded[port] = binary
        state['uploaded'] = uploaded
        _save_state(state)
        # algunos puentes se caen del bus mientras se resetean
        port = _wait_for_port(timeout=15.0)

    return port, notes
