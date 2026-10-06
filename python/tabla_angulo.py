"""La tabla que corrige el error de ángulo, con la aritmética de la placa.

Reproduce AngleLut (Calibracion/AngleLut.h) cuenta por cuenta, redondeo incluido,
para las dos puntas que necesitan hacer lo mismo que la placa: la calibración
(extras/calibracion_as5600/calib.py), que arma la tabla y verifica que llegó
entera, y el banco simulado, que la aplica a sus capturas.

Una tabla son LUT_SIZE entradas en octavos de cuenta, indexadas por la cuenta
cruda del sensor.
"""
import numpy as np

# Cuentas por vuelta del AS5600: 12 bits.
CUENTAS = 4096
GRADOS_POR_CUENTA = 360.0 / CUENTAS

# Entradas de la tabla, y la unidad en la que se guardan. Tienen que coincidir con
# AngleLut en libraries/Calibracion, que es lo que corre Banco.
LUT_SIZE = 64
OCTAVOS = 8

# Techo de una entrada, en octavos de cuenta: ±511 cuentas, ±45 grados. Las
# entradas son int16 porque un error de varios grados no implica necesariamente
# un imán a la distancia equivocada: con el AGC en media escala --la distancia
# correcta-- el segundo armónico midió entre 105 y 108 cuentas, 9,2 a 9,5 grados.
# El AGC informa la distancia y no el centrado, así que un imán puede estar a la
# distancia justa y de todos modos torcido.
LUT_MAX = 4095


def checksum(lut):
    """Suma de Fletcher de 16 bits, igual que AngleLut::checksum().

    Fletcher y no una suma simple porque una suma no distingue una tabla de otra
    con dos entradas intercambiadas, y una entrada en el índice equivocado es justo
    el error que se comete al cargarla.

    Sobre palabras de 16 bits y módulo 65535, con la suma del complemento a uno
    escrita igual que en AngleLut.h para que las dos mitades den exactamente lo
    mismo y no sólo valores congruentes. Por qué no sobre bytes módulo 256, en el
    comentario de allá.
    """
    def ones_add(x, y):
        s = (x + y) & 0xFFFF
        return (s + 1) & 0xFFFF if s < x else s

    a = b = 0
    for v in lut:
        a = ones_add(a, int(v) & 0xFFFF)
        b = ones_add(b, a)
    return (a + b) & 0xFFFF


def correccion(lut, cuentas):
    """La corrección en cuentas para cada cuenta cruda, igual que AngleLut::correction().

    Interpola entre las dos entradas que rodean al ángulo y redondea igual que la
    placa, así que lo que se dibuja con esto es lo que la placa va a hacer
    efectivamente y no lo que diría el modelo continuo. El ángulo corregido es
    `crudo - correccion(lut, crudo)`.
    """
    cuentas = np.asarray(cuentas, dtype=np.int64) & (CUENTAS - 1)
    i = (cuentas >> 6) & (LUT_SIZE - 1)
    frac = cuentas & 0x3F

    tabla = np.array(lut, dtype=np.int64)
    octavos = (tabla[i] * (64 - frac) + tabla[(i + 1) & (LUT_SIZE - 1)] * frac) >> 6

    # (e + 4) >> 3 con corrimiento aritmético, que es lo que hace el AVR.
    return (octavos + 4) >> 3


def escribir(lut, empaquetado):
    """Lo que hace AngleLut::write_packed() con una escritura de `ang_lutw`.

    `empaquetado` es (índice << 16) | valor, con el valor como int16 y acotado a
    ±LUT_MAX igual que en la placa. Devuelve False, sin tocar nada, para un índice
    que no existe.
    """
    empaquetado = int(empaquetado) & 0xFFFFFFFF
    indice = empaquetado >> 16
    if indice >= len(lut):
        return False

    valor = empaquetado & 0xFFFF
    valor = valor - 65536 if valor > 32767 else valor
    lut[indice] = max(-LUT_MAX, min(LUT_MAX, valor))
    return True
