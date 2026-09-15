"""La lógica de bench.py contra un banco con las fallas reales.

Lo que ninguna prueba del proyecto ejercita sin placa: bringup() y los signos,
declarar_divisor(), configurar() y cableado.json, con la placa a 3,3 V, con y sin
el divisor de A1, y con la fuente del motor prendida y apagada.
"""
import contextlib
import io
import json
import tempfile
from pathlib import Path

import numpy as np

from comun import check, terminar
from banco_con_fallas import BancoConFallas, DIVISOR_DEL_BANCO, bench_sobre
import ensayo

DIV_E4 = round(DIVISOR_DEL_BANCO[1] / sum(DIVISOR_DEL_BANCO) * 10000)


def callado(funcion, *args, **kw):
    salida = io.StringIO()
    with contextlib.redirect_stdout(salida):
        resultado = funcion(*args, **kw)
    return resultado, salida.getvalue()


def linea(texto, etiqueta):
    return next((l for l in texto.splitlines() if f' {etiqueta} ' in l), '')


def leer(ruta):
    return json.loads(ruta.read_text(encoding='utf-8')) if ruta.exists() else {}


tmp = Path(tempfile.mkdtemp(prefix='tests-banco-'))

# ------------------------------------------------------------------ los signos
# Sin divisor es el caso de la v2: la caída de AVCC sube la lectura con el PWM, así
# que la media del tirón puede salir positiva con el sensor al revés; sólo comparando
# el arranque con el final se ve el signo. Con el divisor, la caída se cancela.
for div in (0, DIV_E4):
    for sensor, imán in ((True, True), (False, True), (True, False), (False, False)):
        banco = BancoConFallas(angulo_invertido=imán, sensor_invertido=sensor, semilla=1)
        cab = tmp / f'cableado_{div}{int(sensor)}{int(imán)}.json'
        rig = bench_sobre(banco, cab)
        rig.mot_bidir = 0
        rig.cur_div = div
        ok, texto = callado(rig.bringup)
        guardado = leer(cab)
        como = 'con divisor' if div else 'sin divisor, con la caída de AVCC'
        check(f'{como}: bringup mide cur_inv = {int(sensor)} y ang_inv = {int(imán)}',
              guardado.get('cur_inv') == int(sensor) and guardado.get('ang_inv') == int(imán),
              linea(texto, 'signos').strip())
        check(f'{como}: después de ponerlos, +u sube el ángulo y la corriente arranca positiva',
              '[   ok]  signos' in texto, linea(texto, 'signos').strip())

# Sin divisor, en una placa a 3,3 V el sensor no reposa en la mitad: bringup lo dice.
banco = BancoConFallas(semilla=9)
rig = bench_sobre(banco, tmp / 'cableado_ref.json')
rig.mot_bidir = 0
_, texto = callado(rig.bringup, motor=False)
check('sin divisor en una placa a 3,3 V, bringup avisa que el sensor no reposa en la mitad',
      'referencia de i' in texto and 'declarar_divisor' in texto, linea(texto, 'referencia de i').strip())

# ---------------------------------------------------------- bidir contra lo que hay
banco = BancoConFallas(semilla=2)
rig = bench_sobre(banco, tmp / 'cableado_bidir.json')
rig.mot_bidir = 1                       # declarado puente, pero el banco es B′
rig.cur_div = DIV_E4
ok, texto = callado(rig.bringup)
check('bringup marca FALLA si se declara un puente en un banco de un cuadrante',
      '[FALLA]  actuador' in linea(texto, 'actuador'), linea(texto, 'actuador').strip())
check('y bringup no pasa', not ok)

# ------------------------------------------------------- sin fuente del motor
# Sin divisor, el PWM solo corre la lectura cientos de mA: el pico no prueba que el
# motor ande. Con el sensor de ángulo, la prueba es que gire.
for div in (0, DIV_E4):
    banco = BancoConFallas(fuente_prendida=False, semilla=3)
    cab = tmp / f'cableado_sinfuente_{div}.json'
    rig = bench_sobre(banco, cab)
    rig.mot_bidir = 0
    rig.cur_div = div
    ok, texto = callado(rig.bringup)
    como = 'con divisor' if div else 'sin divisor'
    check(f'{como}: sin fuente del motor bringup marca FALLA en motor', '[FALLA]  motor' in texto,
          linea(texto, 'motor').strip())
    check(f'{como}: y no inventa signos, no escribe cableado.json', not cab.exists())

# ------------------------------------------------------------ declarar_divisor
banco = BancoConFallas(semilla=4)
cab = tmp / 'cableado_div.json'
cab.write_text(json.dumps({'ang_inv': 1, 'cur_sag': [0] * 17, 'cur_red': 4968}), encoding='utf-8')
rig = bench_sobre(banco, cab)
rig.mot_bidir = 0
rel, texto = callado(rig.declarar_divisor, *DIVISOR_DEL_BANCO)
guardado = leer(cab)
check('declarar_divisor fija la relación del divisor en la placa', rel == DIV_E4 and rig.cur_div == DIV_E4,
      f'{rel} contra {DIV_E4}')
check('mide el cero contra la alimentación del sensor, cerca de 2000 cuentas equivalentes',
      abs(rig.cur_zero / 2000 - 1) < 0.08 and 'OJO' not in texto, texto.strip().replace('\n', ' | '))
check('lo guarda en cableado.json, sin tocar los signos, y borra la calibración vieja',
      guardado.get('cur_div') == DIV_E4 and guardado.get('ang_inv') == 1
      and 'cur_sag' not in guardado and 'cur_red' not in guardado, str(guardado))

# Con el divisor, el PWM sin corriente en el motor ya no se lee como corriente.
banco = BancoConFallas(fuente_prendida=False, semilla=5)
rig = bench_sobre(banco, tmp / 'cableado_pwm.json')
rig.mot_bidir = 0
restos = {}
for div in (0, DIV_E4):
    rig.cur_div = div
    callado(rig.zero_current)
    for u in (48, 255):
        rig.ctl_uff = u
        df = rig.capture(1.0, warn=False)
        restos[(div, u)] = df['i'][df['t'] > df['t'].iloc[0] + 0.3].mean()
    rig.rest()
check('sin divisor, el PWM sin motor se lee como cientos de mA',
      abs(restos[(0, 255)]) > 100, f'{restos[(0, 255)]:+.0f} mA a fondo')
check('con divisor, menos de 10 mA',
      max(abs(restos[(DIV_E4, u)]) for u in (48, 255)) < 10,
      ', '.join(f'{restos[(DIV_E4, u)]:+.1f} mA a {u}' for u in (48, 255)))

# Un divisor declarado pero sin cablear: aborta y vuelve a medir contra AVCC.
for falla in (None, 'suelto'):
    banco = BancoConFallas(divisor=falla, semilla=6)
    cab = tmp / f'cableado_sindiv_{falla}.json'
    rig = bench_sobre(banco, cab)
    rig.mot_bidir = 0
    try:
        callado(rig.declarar_divisor, *DIVISOR_DEL_BANCO)
        abortó = False
    except RuntimeError:
        abortó = True
    check(f'declarar_divisor aborta con A1 {"sin nada" if falla is None else "suelto"}',
          abortó and rig.cur_div == 0 and not cab.exists(), f'cur_div = {rig.cur_div}')

# ------------------------------------------------------------ configurar
banco = BancoConFallas(angulo_invertido=True, sensor_invertido=True, semilla=7)
cab = tmp / 'cableado_conf.json'
cab.write_text(json.dumps({'ang_inv': 1, 'cur_inv': 1, 'cur_div': DIV_E4}), encoding='utf-8')
rig = bench_sobre(banco, cab)
dichos = []
rig.configurar(bidir=False, say=dichos.append)
check('configurar carga bidir, signos y divisor desde cableado.json',
      rig.mot_bidir == 0 and rig.ang_inv == 1 and rig.cur_inv == 1 and rig.cur_div == DIV_E4,
      ' | '.join(dichos))
check('y mide el cero de la corriente, cerca de la mitad de la alimentación del sensor',
      abs(rig.cur_zero / 2000 - 1) < 0.08, str(rig.cur_zero))
check('con A1 sano no avisa nada del divisor', not any('A1 lee' in d for d in dichos))

banco = BancoConFallas(divisor='suelto', semilla=8)
cab = tmp / 'cableado_suelto.json'
cab.write_text(json.dumps({'ang_inv': 0, 'cur_inv': 0, 'cur_div': DIV_E4}), encoding='utf-8')
rig = bench_sobre(banco, cab)
dichos = []
rig.configurar(bidir=False, say=dichos.append)
check('con el divisor declarado y suelto, configurar avisa que la corriente no es válida',
      any('A1 lee 0' in d and 'suelto' in d for d in dichos), ' | '.join(dichos))

banco = BancoConFallas(semilla=10)
rig = bench_sobre(banco, tmp / 'no_existe.json')
dichos = []
rig.configurar(bidir=None, say=dichos.append)
texto = ' '.join(dichos)
check('sin bidir declarado, configurar avisa', 'no se declaro bidir' in texto)
check('sin cableado.json, avisa que faltan el divisor y los signos',
      'no hay divisor de A1 declarado' in texto and 'sin signos medidos' in texto, texto)

# -------------------------------------------------------------- esperar_quieto
banco = BancoConFallas(semilla=11)
rig = bench_sobre(banco, tmp / 'cableado_quieto.json')
rig.mot_bidir = 0
rig.ctl_uff = 200
rig.capture(3.0, warn=False)
esperó = ensayo.esperar_quieto(rig)
df = rig.capture(0.5, warn=False)
giro = abs(np.deg2rad(df['y_uw'].iloc[-1] - df['y_uw'].iloc[0])) / 0.5
check('esperar_quieto vuelve con el eje quieto', giro < 0.5,
      f'esperó {esperó:.1f} s, queda {giro:.2f} rad/s')
check('y el soltado desde 200 tarda segundos, no décimas', esperó > 1.0, f'{esperó:.1f} s')

terminar()
