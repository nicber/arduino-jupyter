"""Lado de la computadora del experimento AdcFase: grabar, pedir ráfagas y ubicarlas en el PWM.

    import adc_fase
    placa = adc_fase.conectar()          # compila y graba AdcFase si hace falta
    placa.u(150)
    r = placa.rafaga(384)                # r.adc, r.fase (0..1 del período), r.t

La fase es la del instante en que el ADC retuvo la entrada: la del contador del
Timer1 cuando corrió la interrupción de fin de conversión, menos `RETENCION_US`, que
se midió con el DAC interno enganchado al PWM. `fase = 0` es el fondo del contador,
el centro del pulso encendido; el pulso ocupa `|fase| < duty / 2`.

Después de usarlo, `sync_board()` vuelve a grabar Banco solo: acá se anota en el
estado de compilación qué binario quedó en el puerto.
"""

import hashlib
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import serial

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ / 'python'))

import bench                                    # noqa: E402
from ctrllink import find_port                  # noqa: E402

SKETCH = RAIZ / 'AdcFase'
TOP = 8000
PERIODO_TICKS = 2 * TOP                         # un período de PWM, en cuentas de 62,5 ns
TICK_S = 1 / 16e6

# Medido en el clon con LGT8F328P, ADC libre: cuánto antes del fin de la conversión
# queda retenida la entrada, por bits del preescalador. Una conversión libre dura
# 22 relojes del ADC.
RETENCION_US = {7: 98.0, 6: 54.0, 5: 26.4, 4: 14.8, 3: 11.7}
RELOJES_POR_CONVERSION = 22


@dataclass
class Rafaga:
    adc: np.ndarray         # cuentas, normalizadas a 12 bits
    fase: np.ndarray        # 0..1 del período de PWM en la retención, 0 = centro del pulso encendido
    ticks: np.ndarray       # el instante de retención en cuentas del Timer1, desenrollado
    u: int
    ocr1a: int
    modo: int

    @property
    def duty(self):
        return self.ocr1a / TOP


def grabar(port=None, say=print):
    """Compila y graba AdcFase. Devuelve el puerto."""
    build = bench.BUILD_DIR / SKETCH.name
    flags = []
    for prop in bench.BUILD_PROPERTIES:
        flags += ['--build-property', prop]
    bench._run([bench._arduino_cli(), 'compile', '--fqbn', bench.FQBN,
                '--libraries', str(bench.LIBRARIES), '--build-path', str(build)]
               + flags + [str(SKETCH)], 'compilar')

    port = port or find_port()
    state = bench._load_state()
    binario = hashlib.sha256((build / f'{SKETCH.name}.ino.hex').read_bytes()).hexdigest()
    if state.get('uploaded', {}).get(port) != binario:
        bench._upload(port, state, say, SKETCH, build)
        state.setdefault('uploaded', {})[port] = binario
        bench._save_state(state)
        say(f'AdcFase grabado en {port}')
    return port


class Placa:
    def __init__(self, port):
        self.ser = serial.Serial(port, 1_000_000, timeout=2)

        # Abrir el puerto resetea la placa, y el bootloader tarda lo que tarda.
        limite = time.monotonic() + 10
        while time.monotonic() < limite:
            if b'AdcFase listo' in self.ser.readline():
                break
        else:
            raise TimeoutError(f'{port}: no arranco AdcFase')
        self.info = self.cmd('i')
        campos = dict(f.split('=') for f in self.info[4:].split())
        self.full = int(campos['full'])

    def close(self):
        self.u(0)
        self.ser.close()

    def _linea(self):
        linea = self.ser.readline()
        if not linea:
            raise TimeoutError('la placa no contesta')
        return linea.decode('ascii', 'replace').strip()

    def _enviar(self, texto):
        # De a un byte, espaciados: con varias interrupciones rápidas corriendo, el
        # USART de la placa guarda dos bytes y pierde los que llegan pegados. Es lo
        # mismo que hace ctrllink con Banco.
        for b in (texto + '\n').encode():
            self.ser.write(bytes([b]))
            self.ser.flush()
            time.sleep(0.002)

    def cmd(self, texto, intentos=3):
        for intento in range(intentos):
            self.ser.reset_input_buffer()
            self._enviar(texto)
            try:
                while True:
                    linea = self._linea()
                    if linea.startswith('# ok') or linea.startswith('# i') or linea.startswith('# err'):
                        if linea.startswith('# err'):
                            raise RuntimeError(f'{texto}: {linea}')
                        return linea
            except TimeoutError:
                if intento + 1 == intentos:
                    raise

    def u(self, valor):
        self.cmd(f'u {int(valor)}')

    def rafaga(self, n=384, compacta=False):
        """Una ráfaga. `compacta` baja 2 bytes por muestra y deduce la fase del índice:
        sólo vale con el ADC libre, cuyo período es exacto."""
        for intento in range(3):
            self.ser.reset_input_buffer()
            self._enviar(f'{"B" if compacta else "b"} {n}')
            try:
                linea = self._linea()
                while not linea.startswith('# b '):
                    linea = self._linea()
                break
            except TimeoutError:
                if intento == 2:
                    raise
        campos = dict(f.split('=') for f in linea[4:].split())
        got, ancho = int(campos['n']), int(campos['wide'])
        crudo = self.ser.read(ancho * got)
        if len(crudo) != ancho * got:
            raise TimeoutError('rafaga cortada')
        while not self._linea().startswith('# end'):
            pass

        presc, modo = int(campos['presc']), int(campos['mode'])

        def posicion(t):
            t = np.asarray(t, dtype=np.int64)
            baja = (t & 0x8000) != 0
            t = t & 0x7FFF
            return np.where(baja, PERIODO_TICKS - t, t) % PERIODO_TICKS

        datos = np.frombuffer(crudo, dtype='>u2').reshape(-1, ancho // 2).astype(np.int64)
        adc = datos[:, 0] * (4096 // self.full)

        if ancho == 4:
            pos = posicion(datos[:, 1])
            salto = np.diff(pos) % PERIODO_TICKS
            ticks = np.concatenate([[pos[0]], pos[0] + np.cumsum(salto)]) if got else pos
        else:
            if modo != 0:
                raise ValueError('la rafaga compacta sólo vale con el ADC libre')
            p0 = int(posicion(int(campos['first'])))
            ticks = p0 + np.arange(got) * RELOJES_POR_CONVERSION * 2 ** presc

        # Del fin de la conversión al instante de retención.
        ticks = ticks - RETENCION_US.get(presc, 0.0) * 1e-6 / TICK_S
        return Rafaga(adc=adc, fase=(ticks % PERIODO_TICKS) / PERIODO_TICKS,
                      ticks=ticks, u=int(campos['u']), ocr1a=int(campos['ocr1a']),
                      modo=modo)


def conectar(port=None):
    return Placa(grabar(port))
