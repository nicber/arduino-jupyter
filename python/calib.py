"""Calibración del error de ángulo del AS5600, del lado de la computadora.

El sensor no mide el ángulo que uno cree: un imán descentrado corre la lectura en
una cantidad que depende del ángulo y que se repite vuelta tras vuelta. Este
módulo mide esa función sin tener un encoder de referencia, decide qué parte de lo
que midió es realmente el sensor, y arma la tabla que la corrige adentro del
Arduino. El método y por qué cada paso es como es están en
Docs/CALIBRACION_AS5600.md.

    import calib

    df  = calib.desaceleracion(dev, uff=200)      # medir
    bar = calib.barrido(df)                        # A_k contra velocidad
    cal = calib.Calibracion.desde_barrido(bar)     # decidir y armar la tabla
    cal.aplicar(dev)                               # empujarla a la placa
    cal.guardar('calibracion.json')                # y guardarla acá

Este módulo está aparte de ctrllink.py a propósito. El enlace no sabe nada de
ningún sketch en particular --descubre parámetros y canales en tiempo de
ejecución-- y esa propiedad se pierde en cuanto se le mete adentro el
conocimiento de qué significa `lutw`. Acá sí se lo sabe.
"""
import json
import time
from dataclasses import dataclass, field

import numpy as np

# Cuentas por vuelta del AS5600: 12 bits.
CUENTAS = 4096
GRADOS_POR_CUENTA = 360.0 / CUENTAS

# Entradas de la tabla en el dispositivo, y la unidad en la que se guardan.
# Tienen que coincidir con LUT_SIZE y con lut_lookup() en ControlDemo.ino.
LUT_SIZE = 64
OCTAVOS = 8

# Compuerta G2 del plan: qué armónico se acepta como error del sensor.
#
#   - Por debajo de un tercio de cuenta no vale la pena: el sensor cuantiza a una
#     cuenta y su propio ruido es de media. Contra datos sintéticos sin ese
#     armónico, el ajuste devuelve del orden de 0,05 cuentas, así que 0,3 es seis
#     veces lo que el método inventa.
#   - Y tiene que ser plano en la velocidad. El error del sensor es una función
#     del ángulo y no sabe a qué velocidad se lo recorre; una ondulación de par
#     del motor tiene que atravesar la mecánica para volverse posición, y cae
#     entre omega^-1 y omega^-2. La pendiente en log-log es lo que los separa, y
#     es la única cosa de todo este módulo que impide calibrar el motor como si
#     fuera el sensor.
A_MINIMA = 0.3          # cuentas
SIGMAS = 3.0            # A_k tiene que superar esto veces su propio error
PENDIENTE_MAXIMA = 0.3  # |d log A / d log omega| aceptable para llamarlo sensor


# ------------------------------------------------------------------ medición

def a_cuentas(grados):
    """Deshace la escala de ingeniería de un canal de ángulo.

    Se redondea y no se trunca: el dispositivo declara la escala con siete
    decimales --0,0878906 contra los 0,087890625 exactos--, así que dividir deja
    4094,997 donde el sensor midió 4095. Truncar eso corre una cuenta entera cada
    tanto, que es del mismo tamaño que el error que estamos midiendo.
    """
    return np.rint(np.asarray(grados, dtype=float) / GRADOS_POR_CUENTA).astype(np.int64)


def desenrollar(crudo, tick=None, paso_tick=1):
    """Cuentas crudas desenrolladas, por el camino más corto entre muestras.

    Devuelve (cuentas, aviso). `aviso` es None o el texto de lo que salió mal:
    desenrollar da por sentado que el eje se movió menos de media vuelta entre
    dos filas, y un hueco de telemetría --una fila que el dispositivo descartó--
    no es una muestra faltante sino un salto en el tiempo. Con el eje rápido y un
    hueco largo, media vuelta se pasa y el desenrollado inventa una vuelta que no
    existió. Por eso se mira, en lugar de confiar.
    """
    crudo = np.asarray(crudo, dtype=np.int64)

    paso = (np.diff(crudo) + CUENTAS // 2) % CUENTAS - CUENTAS // 2
    cuentas = np.concatenate([[crudo[0]], crudo[0] + np.cumsum(paso)]).astype(float)

    aviso = None

    if tick is not None and len(paso):
        dtick = np.diff(np.asarray(tick, dtype=np.int64))

        # Cuánto se mueve el eje en un tick, medido donde la serie no tiene
        # huecos. La mediana y no el promedio: un solo salto mal desenrollado no
        # tiene que poder mover la referencia con la que se lo detecta.
        limpio = dtick == paso_tick
        if limpio.any() and dtick.max() > paso_tick:
            por_tick = np.median(np.abs(paso[limpio])) / paso_tick
            peor = dtick.max() * por_tick
            if por_tick > 0 and peor > CUENTAS / 2:
                aviso = (f'hueco de {dtick.max()} ticks a {por_tick:.0f} cuentas '
                         f'por tick: {peor:.0f} cuentas es más de media vuelta, '
                         f'el desenrollado no es confiable')

    return cuentas, aviso


def serie(df, fuente='y_raw', avisar=True):
    """(t, cuentas) de una captura, listos para ajustar.

    `fuente` decide cuál de los dos ángulos se mira, y la distinción es el
    experimento entero:

      'y_raw'  la cuenta cruda, tal como salió del sensor, SIN corregir. Es la
               que se usa para MEDIR el error, y la única con la que tiene
               sentido construir la tabla, porque es la que la tabla indexa.
      'y_uw'   el ángulo que el lazo realmente usa, ya corregido si `cal` está
               prendido. Es la que se usa para VALIDAR: preguntarle a `y_raw` si
               la corrección sirvió no tendría sentido, porque `y_raw` es
               justamente lo que no cambia al prenderla.

    `y_uw` viene con el signo invertido --ControlDemo calcula y = offset - counts--
    así que se lo da vuelta acá. Eso deja las amplitudes intactas y espeja las
    fases, que para validar da igual: lo que se compara son amplitudes.
    """
    t = np.asarray(df['t'], dtype=float)

    # El tick no es una columna del DataFrame: vive en attrs, ya desenrollado de
    # su vuelta al cero de 16 bits.
    tick = df.attrs.get('tick')
    paso_tick = df.attrs.get('dec', 1) or 1

    aviso = None

    if fuente == 'y_raw' and 'y_raw' in df:
        cuentas, aviso = desenrollar(a_cuentas(df['y_raw']), tick, paso_tick)
    else:
        # y_uw ya viene desenrollado por el firmware, que lo hace sobre cada
        # muestra de 5 kHz y no sobre la fila de telemetría: no hay huecos que
        # puedan inventar una vuelta.
        cuentas = -a_cuentas(df['y_uw']).astype(float)

    if aviso and avisar:
        print('# aviso:', aviso)

    return t, cuentas


def desaceleracion(dev, uff=200, duracion=25.0, arranque=3.0, sfilt=3):
    """Lleva el motor a velocidad, lo suelta, y captura la desaceleración.

    Es el experimento central del plan. Con `u = 0` no hay corriente de armadura
    y por lo tanto no hay ondulación de par de conmutación, que es la fuente
    mecánica más grande; y como la velocidad barre continuamente de alta a baja,
    una sola captura da la amplitud de cada armónico en todo un rango de
    velocidades. Eso es lo que después separa el sensor del motor.
    """
    dev.mode = 0
    dev.offset = 0
    dev.cal = 0
    dev.sfilt = sfilt
    dev.uff = uff

    try:
        return dev.capture(duracion, events=[(arranque, 'uff', 0)])
    finally:
        dev.uff = 0


def regimen(dev, uff, duracion=20.0, sfilt=3, cal=0):
    """Captura a comando constante. La contraparte "en régimen" de desaceleracion()."""
    dev.mode = 0
    dev.offset = 0
    dev.cal = cal
    dev.sfilt = sfilt
    dev.uff = uff

    try:
        # Un par de segundos para que la velocidad se establezca antes de medir.
        time.sleep(2.0)
        return dev.capture(duracion)
    finally:
        dev.uff = 0


# ------------------------------------------------------------------- ajuste

@dataclass
class Armonicos:
    """Amplitud, fase e incertidumbre de cada armónico, en cuentas."""

    A: np.ndarray       # A[k-1], cuentas
    phi: np.ndarray     # phi[k-1], radianes; e = sum A_k sin(k*ang + phi_k)
    sigma: np.ndarray   # error estándar de A[k-1], cuentas
    omega: float        # velocidad media de la ventana, vueltas por segundo
    vueltas: float      # cuántas vueltas cubre
    residuo: float      # desvío estándar de lo que el modelo no explica, cuentas

    @property
    def K(self):
        return len(self.A)

    def evaluar(self, ang):
        """El error modelado, en cuentas, para un ángulo crudo 0..4095."""
        ang = np.asarray(ang, dtype=float)
        total = np.zeros_like(ang)
        for k in range(1, self.K + 1):
            total += self.A[k-1] * np.sin(2*np.pi*k*ang/CUENTAS + self.phi[k-1])
        return total

    def __repr__(self):
        picos = ', '.join(f'A{k+1}={a:.2f}' for k, a in enumerate(self.A) if a > A_MINIMA)
        return (f'<Armonicos {self.omega:.2f} rev/s, {self.vueltas:.0f} vueltas, '
                f'{picos or "nada por encima del piso"}>')


def ajustar(t, cuentas, K=8, grado=8):
    """Ajusta la tendencia de velocidad y los armónicos del error a la vez.

    La tentación es desenrollar, ajustar una recta y mirar el residuo. Funciona,
    pero entonces la tendencia y los armónicos compiten por la misma varianza y
    una tendencia demasiado flexible se come el primer armónico. Acá van los dos
    en un solo problema de mínimos cuadrados, y lo que los mantiene separados es
    que viven en dominios distintos: la tendencia es suave en el TIEMPO, el error
    es periódico en el ANGULO. Con 50 vueltas el armónico más lento son 50 ciclos
    contra un polinomio de grado 8, así que no compiten.

    Usar el ángulo medido en lugar del verdadero para evaluar la base de Fourier
    es un error de segundo orden en A/4096: con A de seis cuentas son 1e-4
    radianes, despreciable.
    """
    t = np.asarray(t, dtype=float)
    cuentas = np.asarray(cuentas, dtype=float)

    if len(t) < 4 * (grado + 2*K + 1):
        raise ValueError('muy pocas muestras para este ajuste')

    ang = np.mod(cuentas, CUENTAS)
    span = np.ptp(t)
    ts = (t - t.mean()) / (span / 2 if span else 1.0)

    columnas = [ts**j for j in range(grado + 1)]
    for k in range(1, K + 1):
        fase = 2*np.pi*k*ang/CUENTAS
        columnas += [np.cos(fase), np.sin(fase)]

    X = np.column_stack(columnas)
    coef, *_ = np.linalg.lstsq(X, cuentas, rcond=None)

    residuo = cuentas - X @ coef
    gl = max(len(t) - X.shape[1], 1)
    var = float(residuo @ residuo) / gl

    # Diagonal de var * (X^T X)^-1: el error estándar de cada coeficiente.
    cov = var * np.linalg.pinv(X.T @ X)
    err = np.sqrt(np.abs(np.diag(cov)))

    a = coef[grado+1::2]    # cosenos
    b = coef[grado+2::2]    # senos
    ea = err[grado+1::2]
    eb = err[grado+2::2]

    A = np.hypot(a, b)
    # Propagación a la amplitud. Donde A es chica la fórmula se indetermina, y
    # ahí el promedio de los dos errores es la respuesta honesta.
    with np.errstate(divide='ignore', invalid='ignore'):
        sigma = np.where(A > 0,
                         np.sqrt((a*ea)**2 + (b*eb)**2) / np.where(A > 0, A, 1),
                         (ea + eb) / 2)

    vueltas = np.ptp(cuentas) / CUENTAS
    omega = vueltas / span if span else 0.0

    return Armonicos(A=A, phi=np.arctan2(a, b), sigma=sigma,
                     omega=float(omega), vueltas=float(vueltas),
                     residuo=float(np.sqrt(var)))


def residuo(t, cuentas, grado=8):
    """(ángulo crudo, residuo) después de sacarle al eje su marcha suave.

    Es el paso que hace visible el problema antes de modelarlo: el ángulo medido
    menos la rampa que tendría si el eje girara parejo. Lo que queda, graficado
    contra el ángulo dentro de la vuelta en lugar de contra el tiempo, es el
    error del sensor con todas las vueltas apiladas encima.
    """
    t = np.asarray(t, dtype=float)
    cuentas = np.asarray(cuentas, dtype=float)

    span = np.ptp(t)
    ts = (t - t.mean()) / (span / 2 if span else 1.0)
    coef = np.polyfit(ts, cuentas, grado)

    return np.mod(cuentas, CUENTAS), cuentas - np.polyval(coef, ts)


def promediar(ang, valores, cajas=LUT_SIZE):
    """Promedio por caja de ángulo. Devuelve (centros, promedio, n).

    Apilar cien vueltas y promediarlas es lo que baja el ruido del sensor por
    debajo de lo que se quiere medir: media cuenta sobre cien vueltas son cinco
    centésimas.
    """
    ang = np.asarray(ang, dtype=float)
    valores = np.asarray(valores, dtype=float)

    borde = np.linspace(0, CUENTAS, cajas + 1)
    caja = np.clip(np.digitize(ang, borde) - 1, 0, cajas - 1)

    n = np.bincount(caja, minlength=cajas)
    suma = np.bincount(caja, weights=valores, minlength=cajas)

    with np.errstate(invalid='ignore'):
        promedio = np.where(n > 0, suma / np.maximum(n, 1), np.nan)

    return (borde[:-1] + borde[1:]) / 2, promedio, n


def barrido(df, vueltas_por_ventana=10, K=8, grado=3, minimo=6, fuente='y_raw'):
    """Ajusta por ventanas de velocidad casi constante y devuelve la lista.

    Una desaceleración barre la velocidad sola, así que partirla en ventanas da
    A_k(omega) de una sola captura. Adentro de una ventana la velocidad casi no
    cambia, así que la tendencia puede ser un polinomio de grado bajo.

    `minimo` es cuántas vueltas tiene que tener una ventana para que su ajuste
    signifique algo. Por debajo de eso la tendencia y el primer armónico dejan de
    estar separados y el ajuste empieza a mentir sin avisar.
    """
    t, cuentas = serie(df, fuente=fuente)

    # Sólo la parte que se mueve: con el eje quieto no hay ángulo que barrer y el
    # ajuste no tiene de dónde sacar nada.
    if len(t) > 1:
        paso = np.abs(np.gradient(cuentas, t))
        movil = paso > 0.02 * paso.max()
        t, cuentas = t[movil], cuentas[movil]

    salida = []
    borde = cuentas[0]
    inicio = 0

    for i in range(1, len(cuentas)):
        if abs(cuentas[i] - borde) < vueltas_por_ventana * CUENTAS:
            continue

        trozo = slice(inicio, i)
        if abs(cuentas[i-1] - cuentas[inicio]) >= minimo * CUENTAS:
            try:
                salida.append(ajustar(t[trozo], cuentas[trozo], K=K, grado=grado))
            except (ValueError, np.linalg.LinAlgError):
                pass

        borde = cuentas[i]
        inicio = i

    return salida


def pendiente(ventanas, k):
    """d log A_k / d log omega sobre las ventanas de un barrido.

    Cerca de cero quiere decir que la amplitud no depende de la velocidad, que es
    la firma del sensor. Entre -1 y -2 es la mecánica. Devuelve nan si no hay
    ventanas suficientes o si el rango de velocidad es demasiado angosto para
    que la pendiente signifique algo.
    """
    w = np.array([v.omega for v in ventanas])
    A = np.array([v.A[k-1] for v in ventanas])

    bueno = (w > 0) & (A > 0)
    if bueno.sum() < 3:
        return float('nan')

    w, A = w[bueno], A[bueno]
    if w.max() / w.min() < 1.5:
        return float('nan')

    return float(np.polyfit(np.log(w), np.log(A), 1)[0])


def aceptar(ventanas, K=None, verboso=True):
    """Aplica la compuerta G2 y devuelve los armónicos que son del sensor.

    Devuelve (Armonicos promedio, dict k -> motivo del rechazo).
    """
    if not ventanas:
        raise ValueError('no hay ventanas que juzgar')

    K = K or ventanas[0].K

    # Promedio sobre las ventanas en el plano complejo, para que la fase promedie
    # bien: promediar amplitudes y fases por separado hace que dos fases opuestas
    # den una amplitud grande con una fase inventada.
    A = np.zeros(K)
    phi = np.zeros(K)
    sigma = np.zeros(K)
    rechazos = {}

    for k in range(1, K + 1):
        z = np.mean([v.A[k-1] * np.exp(1j*v.phi[k-1]) for v in ventanas])
        A[k-1] = abs(z)
        phi[k-1] = np.angle(z)
        sigma[k-1] = np.mean([v.sigma[k-1] for v in ventanas]) / np.sqrt(len(ventanas))

        p = pendiente(ventanas, k)

        if A[k-1] < A_MINIMA:
            rechazos[k] = f'A={A[k-1]:.2f} por debajo del piso de {A_MINIMA} cuentas'
        elif A[k-1] < SIGMAS * sigma[k-1]:
            rechazos[k] = f'A={A[k-1]:.2f} no supera {SIGMAS}·sigma={SIGMAS*sigma[k-1]:.2f}'
        elif np.isfinite(p) and abs(p) > PENDIENTE_MAXIMA:
            rechazos[k] = f'pendiente {p:+.2f} en log-log: depende de la velocidad, es el motor'

        if k in rechazos:
            A[k-1] = 0.0

    if verboso:
        for k in range(1, K + 1):
            if k in rechazos:
                print(f'  k={k}: rechazado, {rechazos[k]}')
            else:
                print(f'  k={k}: aceptado, A={A[k-1]:.2f} cuentas '
                      f'({A[k-1]*GRADOS_POR_CUENTA:.3f} grados), pendiente {pendiente(ventanas, k):+.2f}')

    promedio = Armonicos(A=A, phi=phi, sigma=sigma,
                         omega=float(np.mean([v.omega for v in ventanas])),
                         vueltas=float(sum(v.vueltas for v in ventanas)),
                         residuo=float(np.mean([v.residuo for v in ventanas])))

    return promedio, rechazos


# -------------------------------------------------------------------- tabla

@dataclass
class Calibracion:
    """La tabla que corrige el sensor, más de dónde salió.

    `lut` son LUT_SIZE enteros en octavos de cuenta, que es exactamente lo que
    guarda el dispositivo. La procedencia viaja con ella porque una tabla sin
    procedencia es un número mágico: dentro de un mes, la diferencia entre "esto
    lo medimos con el imán viejo" y "esto lo medimos ayer" es la única cosa que
    importa.
    """

    lut: list
    armonicos: dict = field(default_factory=dict)   # k -> (A en cuentas, phi)
    banco: str = ''
    creada: str = ''
    notas: str = ''

    @classmethod
    def desde_armonicos(cls, arm, **meta):
        """Muestrea el error modelado en los LUT_SIZE ángulos de la tabla."""
        ang = np.arange(LUT_SIZE) * (CUENTAS / LUT_SIZE)
        error = arm.evaluar(ang)

        # El promedio no se corrige: un corrimiento constante es el cero del
        # sensor, que es asunto de `offset` y no de esta tabla. Sacarlo acá haría
        # que calibrar moviera el cero, que es lo último que uno quiere que pase
        # sin haberlo pedido.
        error = error - error.mean()

        octavos = np.clip(np.round(error * OCTAVOS), -127, 127).astype(int)

        arms = {k: (float(arm.A[k-1]), float(arm.phi[k-1]))
                for k in range(1, arm.K + 1) if arm.A[k-1] > 0}

        meta.setdefault('creada', time.strftime('%Y-%m-%dT%H:%M:%S'))
        return cls(lut=[int(v) for v in octavos], armonicos=arms, **meta)

    @classmethod
    def vacia(cls):
        return cls(lut=[0] * LUT_SIZE, notas='sin corrección')

    # ------------------------------------------------------------- lo mismo
    # que hace el dispositivo, para poder verificarlo sin creerle.

    def checksum(self):
        """Suma de Fletcher de 16 bits, igual que lut_checksum() en el sketch.

        Fletcher y no una suma pelada porque una suma no distingue una tabla de
        otra con dos entradas intercambiadas, y una entrada en el índice
        equivocado es justo el error que se comete acá.
        """
        a = b = 0
        for v in self.lut:
            a = (a + (v & 0xFF)) & 0xFF
            b = (b + a) & 0xFF
        return (b << 8) | a

    def corregir(self, cuentas):
        """La corrección que aplicaría el dispositivo, en cuentas.

        Reproduce lut_lookup() incluida la interpolación y el redondeo, así que
        el notebook puede dibujar lo que la placa va a hacer de verdad y no lo que
        el modelo continuo diría.
        """
        cuentas = np.asarray(cuentas, dtype=np.int64) & (CUENTAS - 1)
        i = (cuentas >> 6) & (LUT_SIZE - 1)
        frac = cuentas & 0x3F

        tabla = np.array(self.lut, dtype=np.int64)
        octavos = (tabla[i] * (64 - frac) + tabla[(i + 1) & (LUT_SIZE - 1)] * frac) >> 6

        # (e + 4) >> 3 con corrimiento aritmético, que es lo que hace el AVR.
        return (octavos + 4) >> 3

    # ------------------------------------------------------------ persistencia

    def guardar(self, ruta):
        with open(ruta, 'w') as f:
            json.dump({'version': 1,
                       'creada': self.creada,
                       'banco': self.banco,
                       'notas': self.notas,
                       'checksum': self.checksum(),
                       'lut_octavos': self.lut,
                       'armonicos': {str(k): v for k, v in self.armonicos.items()}},
                      f, indent=2)
        return ruta

    @classmethod
    def cargar(cls, ruta):
        with open(ruta) as f:
            d = json.load(f)

        cal = cls(lut=[int(v) for v in d['lut_octavos']],
                  armonicos={int(k): tuple(v) for k, v in d.get('armonicos', {}).items()},
                  banco=d.get('banco', ''), creada=d.get('creada', ''),
                  notas=d.get('notas', ''))

        if 'checksum' in d and d['checksum'] != cal.checksum():
            raise ValueError(f'{ruta}: la tabla no coincide con su propia suma de '
                             f'verificación; el archivo está editado o corrupto')

        return cal

    def escribir_header(self, ruta='ControlDemo/Calibracion.h'):
        """Genera el header que deja la calibración compilada adentro del sketch.

        Es el camino para un tablero que se enciende solo y nadie conecta a un
        notebook. El sketch lo toma con __has_include, así que basta con que el
        archivo exista.
        """
        filas = []
        for i in range(0, LUT_SIZE, 8):
            filas.append('    ' + ', '.join(f'{v:4d}' for v in self.lut[i:i+8]) + ',')

        with open(ruta, 'w') as f:
            f.write('// Generado por python/calib.py. No editar a mano.\n')
            f.write(f'//\n// Banco:  {self.banco or "sin identificar"}\n')
            f.write(f'// Medida: {self.creada}\n')
            if self.armonicos:
                picos = ', '.join(f'A{k}={A:.2f} cuentas' for k, (A, _) in
                                  sorted(self.armonicos.items()))
                f.write(f'// Ajuste: {picos}\n')
            f.write('//\n// Entradas en octavos de cuenta, indexadas por el ángulo crudo.\n\n')
            f.write('static const int8_t CAL_LUT[64] PROGMEM =\n{\n')
            f.write('\n'.join(filas))
            f.write('\n};\n')

        return ruta

    # ---------------------------------------------------------------- placa

    def aplicar(self, dev, verificar=True):
        """Empuja la tabla al dispositivo y comprueba que llegó entera.

        Son LUT_SIZE escrituras del parámetro `lutw`, cada una con el índice
        adentro del valor. No hace falta ningún comando nuevo en el protocolo:
        cada escritura es un `set` común, con la misma respuesta verificada que
        cualquier otra.

        La verificación es una sola lectura: el dispositivo mantiene la suma de
        Fletcher de lo que tiene, y acá se la compara contra la de lo que se
        quiso mandar. El protocolo ya distingue entre un comando deformado, que
        se rechaza a los gritos, y un valor deformado, que se aceptaría en
        silencio; una tabla es toda valores.
        """
        for i, v in enumerate(self.lut):
            dev.set('lutw', (i << 8) | (int(v) & 0xFF))

        if verificar:
            leido = int(dev.get('lutsum'))
            propio = self.checksum()
            if leido != propio:
                raise RuntimeError(f'la tabla no llegó entera: el dispositivo dice '
                                   f'{leido:#06x} y esta tabla es {propio:#06x}')

        dev.cal = 1
        return self

    @classmethod
    def leer_de(cls, dev):
        """La suma que declara el dispositivo, para saber qué tiene puesto.

        No devuelve la tabla: el dispositivo no la sabe leer de vuelta a propósito.
        Sirve para preguntar "¿es ésta la que está?" contra una que ya se tiene.
        """
        return int(dev.get('lutsum'))


def esta_puesta(dev, cal):
    """Si el dispositivo tiene puesta esta tabla exacta y la corrección prendida."""
    return int(dev.get('lutsum')) == cal.checksum() and int(dev.get('cal')) == 1


def asegurar(dev, ruta='calibracion.json'):
    """Carga la tabla del archivo y la aplica si no está ya puesta.

    Es lo que va al principio de un notebook: el dispositivo arranca siempre sin
    calibrar --a propósito, para que no pueda mentir-- así que alguien tiene que
    ponerle la tabla, y ese alguien es la computadora, que es la que la tiene.
    """
    cal = Calibracion.cargar(ruta)

    if esta_puesta(dev, cal):
        return cal

    return cal.aplicar(dev)
