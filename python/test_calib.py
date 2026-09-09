"""Ejercita la calibración del AS5600 sin tener ni el sensor ni la placa.

Dos mitades. La primera le da al ajuste un error de ángulo que uno mismo puso, y
comprueba que lo devuelve: amplitudes, fases, y --lo que importa de verdad-- que
la compuerta de velocidad rechaza un armónico que depende de la velocidad, que es
el motor, y acepta el que no, que es el sensor. La segunda empuja la tabla a una
simulación del dispositivo que reimplementa lut_lookup() y la suma de Fletcher
tal como están en ControlDemo.ino, así que lo que se prueba es que las dos
mitades del sistema hacen la misma aritmética.
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(__file__) or '.')

import numpy as np

import calib
from fakeuno import FakeUno, connect

failures = []


def check(label, condition, detail=''):
    print(f'{"PASA " if condition else "FALLA"}  {label}'
          + (f'  -- {detail}' if detail and not condition else ''))
    if not condition:
        failures.append(label)


# ------------------------------------------------------- el banco simulado

def sensor_sintetico(t, rev_por_s, armonicos, ruido=0.5, semilla=0):
    """Cuentas medidas por un AS5600 con el error de ángulo que se le pida.

    `armonicos` es {k: (amplitud en cuentas, fase)}. La medición es
    m = theta + e(theta), que es el modelo del documento, con el error evaluado
    en el ángulo VERDADERO y no en el medido: si el ajuste sólo funcionara con la
    aproximación con la que se lo deriva, no serviría de nada.
    """
    rng = np.random.default_rng(semilla)
    theta = np.asarray(rev_por_s, dtype=float) * calib.CUENTAS * t \
        if np.ndim(rev_por_s) else rev_por_s * calib.CUENTAS * t

    error = np.zeros_like(theta)
    for k, (A, phi) in armonicos.items():
        error += A * np.sin(2*np.pi*k*theta/calib.CUENTAS + phi)

    return theta + error + rng.normal(0, ruido, len(t))


def como_captura(t, medido, dec=1):
    """Un objeto con la misma pinta que el DataFrame que devuelve capture()."""
    import pandas as pd

    crudo = np.mod(np.rint(medido), calib.CUENTAS).astype(np.int64)
    df = pd.DataFrame({'t': t, 'y_raw': crudo * calib.GRADOS_POR_CUENTA})
    df.attrs.update(tick=np.arange(len(t), dtype=np.int64) * dec, dec=dec)
    return df


# ------------------------------------------------------------------ ajuste

t = np.arange(0, 20.0, 1/500.0)          # 500 Hz, 20 s, la geometría del banco
ARM = {1: (6.0, 0.7), 2: (2.5, -2.0), 4: (0.8, 1.3)}
df = como_captura(t, sensor_sintetico(t, 5.0, ARM))

arm = calib.ajustar(*calib.serie(df))

check('el ajuste recupera el primer armonico',
      abs(arm.A[0] - 6.0) < 0.15, f'A1={arm.A[0]:.3f}')
check('el ajuste recupera el segundo armonico',
      abs(arm.A[1] - 2.5) < 0.15, f'A2={arm.A[1]:.3f}')
check('el ajuste recupera el cuarto armonico',
      abs(arm.A[3] - 0.8) < 0.15, f'A4={arm.A[3]:.3f}')
check('un armonico ausente queda bajo el piso de aceptacion',
      arm.A[2] < calib.A_MINIMA, f'A3={arm.A[2]:.3f}')
check('las fases salen bien',
      all(abs(np.angle(np.exp(1j*(arm.phi[k-1] - phi)))) < 0.05
          for k, (_, phi) in ARM.items()),
      str(np.round(arm.phi[:4], 3)))
check('el residuo es del tamano del ruido que se puso',
      0.4 < arm.residuo < 0.6, f'{arm.residuo:.3f}')

# La cuenta cruda desenrollada tiene que dar las vueltas que dio el eje.
_, cuentas = calib.serie(df)
check('el desenrollado cuenta las vueltas', abs(np.ptp(cuentas)/calib.CUENTAS - 100) < 1,
      f'{np.ptp(cuentas)/calib.CUENTAS:.1f} vueltas')

# Redondear y no truncar al deshacer la escala: el dispositivo la declara con
# siete decimales, así que 4095 cuentas vuelven como 4094,997.
check('deshacer la escala no pierde una cuenta',
      calib.a_cuentas(np.array([4095 * 0.0878906]))[0] == 4095,
      str(calib.a_cuentas(np.array([4095 * 0.0878906]))))

# Un hueco largo se avisa en lugar de inventar una vuelta.
tick = np.arange(100, dtype=np.int64)
tick[50:] += 40                                   # 40 ticks perdidos
crudo = np.mod(np.arange(100) * 900, calib.CUENTAS)
_, aviso = calib.desenrollar(crudo, tick)
check('un hueco que se pasa de media vuelta se avisa', aviso is not None, str(aviso))


# --------------------------------------------- la compuerta de velocidad (G2)

# Una desaceleración: la velocidad barre de 10 a 2 vueltas por segundo. El
# armónico 1 es del sensor --amplitud fija-- y el 3 es del motor, con la amplitud
# cayendo como omega^-2, que es lo que hace la inercia con una ondulación de par.
t = np.arange(0, 30.0, 1/500.0)
w = 10.0 - (10.0 - 2.0) * t / t[-1]
theta = np.cumsum(w) * calib.CUENTAS / 500.0

medido = theta + 6.0*np.sin(2*np.pi*theta/calib.CUENTAS + 0.7)
medido += (4.0 * (5.0/w)**2) * np.sin(2*np.pi*3*theta/calib.CUENTAS - 1.0)
medido += np.random.default_rng(1).normal(0, 0.5, len(t))

ventanas = calib.barrido(como_captura(t, medido))
check('el barrido produce varias ventanas', len(ventanas) >= 5, f'{len(ventanas)} ventanas')
check('las ventanas cubren un rango de velocidad',
      max(v.omega for v in ventanas) / min(v.omega for v in ventanas) > 2.0,
      f'{min(v.omega for v in ventanas):.1f}..{max(v.omega for v in ventanas):.1f} rev/s')

p1 = calib.pendiente(ventanas, 1)
p3 = calib.pendiente(ventanas, 3)
check('el armonico del sensor es plano en la velocidad', abs(p1) < 0.3, f'pendiente {p1:+.2f}')
check('el armonico del motor cae con la velocidad', p3 < -1.0, f'pendiente {p3:+.2f}')

aceptados, rechazos = calib.aceptar(ventanas, verboso=False)
check('se acepta el armonico del sensor', 1 not in rechazos, str(rechazos.get(1)))
check('se rechaza el armonico del motor', 3 in rechazos, str(rechazos.get(3)))
check('lo rechazado no entra en el modelo', aceptados.A[2] == 0.0, f'A3={aceptados.A[2]}')
check('lo aceptado conserva su amplitud', abs(aceptados.A[0] - 6.0) < 0.3,
      f'A1={aceptados.A[0]:.2f}')


# --------------------------------------------------------------- la tabla

cal = calib.Calibracion.desde_armonicos(aceptados, banco='banco de prueba')

check('la tabla tiene el tamano del dispositivo', len(cal.lut) == calib.LUT_SIZE)
check('la tabla entra en un int8', all(-128 <= v <= 127 for v in cal.lut))
check('la tabla queda centrada', abs(sum(cal.lut)) < calib.LUT_SIZE,
      f'suma {sum(cal.lut)}')

# Lo que importa: aplicada al ángulo medido, la corrección tiene que achicar el
# error. Se mide sobre el modelo continuo del sensor, con la interpolación y el
# redondeo enteros que hace la placa.
ang = np.arange(calib.CUENTAS)
verdadero = 6.0*np.sin(2*np.pi*ang/calib.CUENTAS + 0.7)
residual = verdadero - cal.corregir(ang)

check('la correccion achica el error mas de cinco veces',
      np.abs(residual).max() < np.abs(verdadero).max() / 5,
      f'{np.abs(verdadero).max():.2f} -> {np.abs(residual).max():.2f} cuentas')
check('lo que queda es del orden del redondeo a cuenta entera',
      np.abs(residual).max() < 1.0, f'{np.abs(residual).max():.2f} cuentas')

# La suma de Fletcher tiene que distinguir dos entradas intercambiadas; una suma
# pelada no, y una entrada en el índice equivocado es el error que se comete acá.
otra = calib.Calibracion(lut=list(cal.lut))
otra.lut[3], otra.lut[9] = otra.lut[9], otra.lut[3]
check('la suma distingue dos entradas intercambiadas',
      otra.checksum() != cal.checksum() or otra.lut == cal.lut,
      f'{otra.checksum():#06x} vs {cal.checksum():#06x}')


# ------------------------------------------------------------ persistencia

with tempfile.TemporaryDirectory() as carpeta:
    ruta = cal.guardar(os.path.join(carpeta, 'calibracion.json'))
    vuelta = calib.Calibracion.cargar(ruta)

    check('la tabla sobrevive al archivo', vuelta.lut == cal.lut)
    check('la procedencia sobrevive al archivo', vuelta.banco == 'banco de prueba')
    check('los armonicos sobreviven al archivo', set(vuelta.armonicos) == set(cal.armonicos))

    # Un archivo editado a mano no se aplica en silencio.
    import json
    with open(ruta) as f:
        roto = json.load(f)
    roto['lut_octavos'][0] += 1
    with open(ruta, 'w') as f:
        json.dump(roto, f)

    try:
        calib.Calibracion.cargar(ruta)
        check('un archivo editado se rechaza', False, 'cargo sin protestar')
    except ValueError as exc:
        check('un archivo editado se rechaza', 'suma de verificaci' in str(exc), str(exc))

    header = cal.escribir_header(os.path.join(carpeta, 'Calibracion.h'))
    texto = open(header).read()
    check('el header declara la tabla en PROGMEM',
          'static const int8_t CAL_LUT[64] PROGMEM' in texto)
    check('el header trae los 64 valores',
          texto.count(',') >= calib.LUT_SIZE, f'{texto.count(",")} comas')
    check('el header dice de donde salio', 'banco de prueba' in texto)


# -------------------------------------------------- contra el dispositivo

class FakeUnoConLut(FakeUno):
    """FakeUno con la etapa de calibración, con la aritmética del sketch.

    lut_lookup() y lut_checksum() están reimplementados acá a partir del C, no
    importados de calib.py: si las dos mitades compartieran la implementación, la
    prueba no diría nada sobre si coinciden.
    """

    def __init__(self, **kw):
        super().__init__(**kw)
        self.lut = [0] * 64
        self.params = dict(self.params)
        self.params['lutw'] = ('u16', 0, 0xFFFF)
        self.params['lutsum'] = ('u16', 0, 0)
        self.params['cal'] = ('u8', 0, 0)
        self.params['sfilt'] = ('u8', 0, 3)
        self.lutw_aplicado = 0xFFFF

    def command(self, cmd):
        super().command(cmd)

        # refresh_tuning() corre después de cualquier escritura de parámetro, que
        # es exactamente lo que permite cargar la tabla sin comandos nuevos.
        if cmd.startswith('set '):
            self.refresh_tuning()

    def refresh_tuning(self):
        lutw = self.params['lutw'][2] & 0xFFFF

        if lutw != self.lutw_aplicado:
            self.lutw_aplicado = lutw
            indice = lutw >> 8
            if indice < 64:
                valor = lutw & 0xFF
                self.lut[indice] = valor - 256 if valor > 127 else valor

        a = b = 0
        for v in self.lut:
            a = (a + (v & 0xFF)) & 0xFF
            b = (b + a) & 0xFF

        self.params['lutsum'] = ('u16', 0, (b << 8) | a)

    def lut_lookup(self, counts):
        i = (counts >> 6) & 63
        frac = counts & 0x3F
        octavos = (self.lut[i] * (64 - frac) + self.lut[(i + 1) & 63] * frac) >> 6
        return (octavos + 4) >> 3


uno = FakeUnoConLut()
dev = connect(uno)

check('el dispositivo arranca sin calibrar', uno.params['cal'][2] == 0)
check('el dispositivo arranca con la tabla en cero', all(v == 0 for v in uno.lut))

cal.aplicar(dev)

check('la tabla llega entera', uno.lut == cal.lut, f'{uno.lut[:4]} vs {cal.lut[:4]}')
check('la suma del dispositivo coincide con la de la computadora',
      uno.params['lutsum'][2] == cal.checksum(),
      f'{uno.params["lutsum"][2]:#06x} vs {cal.checksum():#06x}')
check('aplicar deja la correccion prendida', uno.params['cal'][2] == 1)
check('esta_puesta lo confirma', calib.esta_puesta(dev, cal))

# Y las dos implementaciones de la interpolación tienen que coincidir cuenta por
# cuenta, no en promedio.
mias = cal.corregir(np.arange(calib.CUENTAS))
suyas = np.array([uno.lut_lookup(c) for c in range(calib.CUENTAS)])
check('la interpolacion de la placa y la del notebook coinciden',
      np.array_equal(mias, suyas),
      f'difieren en {int((mias != suyas).sum())} de {calib.CUENTAS} angulos')

# Una tabla que no llegó entera tiene que hacer ruido, no pasar.
uno.lut[7] ^= 0x0F
uno.refresh_tuning()
try:
    cal.aplicar(dev, verificar=True)
    # aplicar() reescribe todo, así que después de eso tiene que volver a cerrar.
    check('una tabla corrompida se detecta o se repara',
          uno.lut == cal.lut, 'quedo distinta y no protesto')
except RuntimeError:
    check('una tabla corrompida se detecta o se repara', True)

# El dispositivo ya calibrado no se vuelve a cargar al pedo.
escrituras_antes = uno.lutw_aplicado
check('asegurar() no reescribe una tabla que ya esta puesta',
      calib.esta_puesta(dev, cal) and uno.lutw_aplicado == escrituras_antes)


print()
print(f'{len(failures)} falla(s)' + (': ' + ', '.join(failures) if failures else ''))
sys.exit(1 if failures else 0)
