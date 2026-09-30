"""Que la documentación, las tablas y las imitaciones digan lo mismo que el código.

Lo que se dice del código se desactualiza con más facilidad que el código mismo:
una frecuencia de PWM que no coincide con PWM_TOP, funciones nombradas que no
existen, archivos mencionados que no vienen en el zip, un límite de desenrollado
escrito con otra cuenta. Esto lo verifica todo sobre el texto.
"""
import json
import re

from comun import RAIZ, PYTHON, check, terminar


def texto(ruta):
    return ruta.read_text(encoding='utf-8', errors='replace')


def celdas(ruta):
    nb = json.loads(texto(ruta))
    return [''.join(c['source']) for c in nb['cells']]


README = texto(RAIZ / 'README.md')
HARDWARE = '\n'.join(celdas(RAIZ / 'notebooks' / 'hardware.ipynb'))
BANCO = texto(RAIZ / 'Banco' / 'Banco.ino')
DOCS = {'README.md': README, 'notebooks/hardware.ipynb': HARDWARE}

fuentes = {}
for patron in ('Banco/*.ino', 'libraries/*/src/*.h', 'libraries/*/src/*.cpp', 'python/*.py',
               'extras/*/*.py', 'extras/*/*.md', 'herramientas/*.py', 'test/*.cpp'):
    for ruta in RAIZ.glob(patron):
        fuentes[str(ruta.relative_to(RAIZ)).replace('\\', '/')] = texto(ruta)
for ruta in RAIZ.glob('extras/*/*.ipynb'):
    fuentes[str(ruta.relative_to(RAIZ)).replace('\\', '/')] = '\n'.join(celdas(ruta))
todo = dict(DOCS, **fuentes)

# ------------------------------------------------------------------ el PWM
top = int(re.search(r'PWM_TOP\s*=\s*(\d+)', BANCO).group(1))
f_pwm = 16e6 / (2 * top)
check('PWM_TOP da la frecuencia que dice el sketch', abs(f_pwm - 1250) < 1, f'{f_pwm:.1f} Hz')

malas = []
for nombre, t in todo.items():
    for n, l in enumerate(t.splitlines(), 1):
        if 'PWM' in l and re.search(r'(?<![\d~])1 kHz', l) and not re.search(r'1250|20 kHz|no 1 kHz|1 kHz justo', l):
            malas.append(f'{nombre}:{n}: {l.strip()[:90]}')
check('ninguna línea dice que el PWM va a 1 kHz', not malas, ' | '.join(malas[:4]))

import banco_simulado  # noqa: E402
check('el banco simulado integra con el período real del PWM',
      abs(banco_simulado.PWM_T * f_pwm - 1) < 1e-3, f'{1 / banco_simulado.PWM_T:.1f} Hz')

# ------------------------------------------------- la unidad del canal de corriente
# El golden de test_tablas.py guarda la escala como el texto `SENSE_MA_PER_LSB`, así que
# un cambio de unidad --que es el más visible de todos, porque cambia lo que significa
# cada número de `i`-- pasa sin que el golden se mueva. Acá se la calcula desde las
# constantes y se la compara contra lo que afirman el README, el notebook y el banco
# simulado.
def _entero(patron, texto, archivo):
    m = re.search(patron, texto)
    check(m is not None, f'{archivo} declara {patron}', patron)
    return int(m.group(1)) if m else 0


SUPPLY = texto(RAIZ / 'libraries' / 'Sense' / 'src' / 'SupplyRatio.h')
uv     = _entero(r'UV_PER_COUNT\s*=\s*(\d+)', SUPPLY, 'SupplyRatio.h')
frac   = _entero(r'FRAC_BITS\s*=\s*(\d+)', SUPPLY, 'SupplyRatio.h')
mv_a   = float(re.search(r'SENSE_MV_PER_A\s*=\s*([\d.]+)f', BANCO).group(1))
ma_por_unidad = uv / mv_a / (1 << frac)

check('la escala del canal i sale de las constantes',
      abs(ma_por_unidad - 0.4223) < 0.0005, f'{ma_por_unidad:.4f} mA por unidad')

for nombre, t in DOCS.items():
    check(f'{nombre} dice la escala del canal i', '0,42 mA' in t,
          'falta «0,42 mA» en el texto')

banco_simulado_txt = fuentes['python/banco_simulado.py']
check('el banco simulado publica la misma unidad que la placa',
      _entero(r'CUR_FRAC\s*=\s*(\d+)', banco_simulado_txt, 'banco_simulado.py') == frac,
      f'CUR_FRAC contra FRAC_BITS = {frac}')

NOTCH = texto(RAIZ / 'libraries' / 'Sense' / 'src' / 'MainsNotch.h')
check('el notch lleva la señal en la misma unidad que el cociente',
      _entero(r'FRAC_BITS\s*=\s*(\d+)', NOTCH, 'MainsNotch.h') == frac,
      f'MainsNotch contra SupplyRatio = {frac}')

# ----------------------------------------------------- los parámetros de la placa
import test_tablas  # noqa: E402
import catalogo  # noqa: E402

tablas = test_tablas.leer_tablas()
nombres = [p['nombre'] for p in tablas['params']]
sin_catalogo = [n for n in nombres if n not in catalogo.nombres_conocidos()]
check('cada parámetro de Banco.ino tiene su explicación en catalogo.py', not sin_catalogo, str(sin_catalogo))
sin_readme = [n for n in nombres if f'`{n}`' not in README and n not in README]
check('y cada uno aparece en el README', not sin_readme, str(sin_readme))
sobran = [n for n in catalogo.nombres_conocidos()
          if n not in nombres and n not in ('chans', 'dec')]
check('el catálogo no explica parámetros que la placa ya no tiene', not sobran, str(sobran))

# ------------------------------------------------ archivos mencionados que existan
patron_ruta = re.compile(
    r'(?<![\w/.])((?:python|libraries|notebooks|extras|herramientas|test|Banco|Docs)/'
    r'(?:[\w.]|/|-(?!-))*\w/?)')
generados = ('cableado.json', 'calibracion.json', 'calibracion-simulada.json', 'datos/')
faltan = set()
for nombre, t in todo.items():
    for m in patron_ruta.finditer(t):
        ruta = m.group(1).rstrip('.')
        if t[m.end():m.end() + 1] == '*':         # un patrón, como python/test_*.py
            continue
        if any(g in ruta for g in generados) or '<' in ruta:
            continue
        if not (RAIZ / ruta).exists():
            faltan.add(f'{nombre}: {ruta}')
check('todo archivo o carpeta del proyecto que se menciona existe', not faltan, ' | '.join(sorted(faltan)[:6]))

# «Sense/RowAdc.h», «AngleSensor/AngleTracker.h»: la forma corta, desde libraries/
cortas = set()
for nombre, t in todo.items():
    for m in re.finditer(r'(?<![\w/])([A-Z]\w+)/(\w+\.h)\b', t):
        if not (RAIZ / 'libraries' / m.group(1) / 'src' / m.group(2)).exists():
            cortas.add(f'{nombre}: {m.group(0)}')
check('y las referencias cortas a cabeceras (Sense/RowAdc.h) también', not cortas, ' | '.join(sorted(cortas)[:6]))

# -------------------------------------------- funciones que la documentación nombra
import bench  # noqa: E402
import ensayo  # noqa: E402

def publicos(obj):
    return {n for n in dir(obj)}

disponibles = {
    'dev': publicos(bench.Bench) | {'capture', 'step', 'set', 'get', 'close', 'resync'},
    'ensayo': publicos(ensayo),
    'bench': publicos(bench),
}
inexistentes = set()
for nombre, t in DOCS.items():
    for m in re.finditer(r'\b(dev|ensayo|bench)\.(\w+)\(', t):
        if m.group(2) not in disponibles[m.group(1)]:
            inexistentes.add(f'{nombre}: {m.group(1)}.{m.group(2)}()')
check('toda función que nombran el README y hardware.ipynb existe', not inexistentes,
      ' | '.join(sorted(inexistentes)))

# --------------------------------------- el protocolo: la placa contra su imitación
cpp = texto(RAIZ / 'libraries' / 'CtrlLink' / 'src' / 'CtrlLink.cpp')
de_la_placa = set(re.findall(r'strcmp\(line, "(\w+)"\)', cpp))
de_la_placa |= set(re.findall(r'strncmp\(line, "(\w+) ', cpp))
fake = texto(PYTHON / 'fakeuno.py')
del_fake = set(re.findall(r"head == '(\w+)'", fake))
check('fakeuno.py entiende los mismos comandos que CtrlLink.cpp',
      de_la_placa and de_la_placa == del_fake, f'placa {sorted(de_la_placa)}, fake {sorted(del_fake)}')

# ------------------------------------------------ el límite del desenrollado, con números
tracker = texto(RAIZ / 'libraries' / 'AngleSensor' / 'src' / 'AngleTracker.h')
m = re.search(r'a (\d+) kHz[^.]*?son (\d[\d ]*) vueltas por segundo', tracker.replace('\n    //', ''))
if m:
    fs, rev = float(m.group(1)) * 1000, float(m.group(2).replace(' ', ''))
    check('el límite de desenrollado de AngleTracker.h está bien calculado (media vuelta por muestra)',
          abs(rev - fs / 2) < 1, f'dice {rev:.0f} rev/s a {fs:.0f} Hz; son {fs / 2:.0f}')
else:
    check('AngleTracker.h dice su límite de desenrollado', False)
check('y Banco desenrolla en la ISR, no en la fila',
      re.search(r'ISR\(TIMER2_COMPA_vect\)[\s\S]{0,1000}g_turns\.update', BANCO) is not None)

# ------------------------------------------------------------- lo que no va en el zip
# Las librerías del lazo siguen en el repositorio, para ControlDemo, pero Banco no las
# usa y el zip no las lleva.
del_lazo = ('Control', 'ControlMath')
check('Banco no incluye las librerías del lazo',
      not any(f'#include <{h.name}>' in BANCO
              for d in del_lazo for h in (RAIZ / 'libraries' / d / 'src').glob('*.h')))
empaquetar = RAIZ / 'herramientas' / 'empaquetar_tp2.py'
if empaquetar.exists():
    excluir = texto(empaquetar)
    check('y empaquetar_tp2.py las deja fuera del zip',
          all(f"'/libraries/{d}/'" in excluir for d in del_lazo))
else:
    check('en un zip no vienen las librerías del lazo',
          not any((RAIZ / 'libraries' / d).exists() for d in del_lazo))

terminar()
