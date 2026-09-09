# Calibración del AS5600: medir la distorsión y corregirla en la placa

El AS5600 de este banco no mide el ángulo que uno cree. La hoja de datos promete
del orden de ±0,5° con el imán bien puesto, y ese "bien puesto" tiene una
tolerancia de ±0,25 mm entre el eje de giro del imán y el centro del integrado.
Lo que sobra de esa tolerancia no aparece como ruido: aparece como una función
fija del ángulo, que se repite vuelta tras vuelta y que a un lazo de control se
le presenta como una ondulación de velocidad y un error de posición que ninguna
ganancia arregla.

Este documento es el plan para (1) medir esa función con el banco que ya
tenemos, sin sensor de referencia, (2) decidir con datos cuánto de lo que se mide
es realmente el sensor, y (3) meter la corrección adentro del Arduino como una
tabla.

## 1. Qué dice la literatura

**La distorsión es armónica y de orden bajo.** El error de un encoder magnético
mal centrado se describe bien con los primeros armónicos del ángulo mecánico.
ODrive modela exactamente el primero y el segundo —sus coeficientes de
compensación son `cosx_coef`, `sinx_coef`, `cos2x_coef`, `sin2x_coef`— y los
estima haciendo girar el motor descargado unos segundos
([ODrive, *Designing for Magnetic Encoders*](https://docs.odriverobotics.com/v/latest/articles/magnetic-encoders.html)).
Los trabajos de compensación por análisis armónico llegan a lo mismo por FFT: el
error de medición es una suma de pocas cosenoides de baja frecuencia, y se
compensa con los parámetros estimados de cada componente
([Sensors 20(6):1715](https://www.mdpi.com/1424-8220/20/6/1715)).

**Cada orden tiene una causa distinta, y eso importa para saber qué esperar.**
El AS5600 interpola el arcotangente de dos canales en cuadratura, así que hereda
la taxonomía clásica de errores de una figura de Lissajous
([Heydemann; ver *What do Lissajous figures tell us about encoder output?*](https://www.motioncontroltips.com/what-do-lissajous-figures-tell-us-about-encoder-output/)):

| Defecto físico | Efecto en los canales | Orden del error de ángulo |
|---|---|---|
| Imán descentrado / eje corrido | corrimiento de cero | 1 por vuelta |
| Imán inclinado respecto del chip | ganancias desiguales, cuadratura imperfecta | 2 por vuelta |
| No linealidad del interpolador, imán no uniforme | distorsión armónica de los canales | 3, 4 y superiores, chicos |

**Y desalinear el imán es exactamente lo que introduce armónicos.** Cualquier
desalineación más allá de ±0,25 mm produce distorsión armónica adicional en las
señales de los canales Hall; ODrive pide ±0,5 mm de desalineación total entre
encoder, imán y eje, y aclara que su compensación armónica "no sustituye al
diseño cuidadoso".

**La receta práctica ya existe y es la que vamos a usar.** SimpleFOC tiene un
`CalibratedSensor` basado en el método de Ben Katz: girar en lazo abierto una
vuelta mecánica en un sentido, otra en el sentido contrario, promediar las dos
pasadas y guardar el resultado en una tabla con interpolación lineal
([SimpleFOC Community, *SimpleFoc Sensor Eccentricity Calibration*](https://community.simplefoc.com/t/simplefoc-sensor-eccentricity-calibration/2212)).
Los números que reportan ahí son alentadores y también son una advertencia: con
0,5 mm de desplazamiento deliberado midieron ~0,12 rad (7°) de error mecánico
pico, y corregirlo bajó el desvío estándar de la velocidad de 6,71 a 0,62 rad/s.
La advertencia es que su propio hilo dice que lo que queda después de corregir es
*cogging*, que no es el sensor.

**La autocalibración por velocidad constante es un método reconocido.** Se
asume muestreo equiespaciado y velocidad angular aproximadamente constante, y la
calibración consiste en estimar el conjunto óptimo de coeficientes de Fourier y
sus fases. Es justamente lo que propone el enunciado de este trabajo.

## 2. Lo que este banco puede y no puede medir

No hay encoder de referencia. La única regla contra la que se puede comparar el
AS5600 es el tiempo, y el puente entre el tiempo y el ángulo es la hipótesis de
velocidad constante. Todo el diseño experimental sale de ahí.

Lo que tenemos, y que es bastante:

- Muestreo del sensor a 5 kHz con período rígido (Timer2, CTC, divisor exacto).
- Telemetría a 500 Hz por omisión (`tickdiv = 10`), con número de tick en cada
  fila, así que un hueco se ve y no se confunde con una muestra.
- `y_uw`, el ángulo ya desenrollado en cuentas, en el flujo.
- Lazo abierto con `mode = 0` y `uff` como comando, y `capture(duración,
  events=[...])` para cambiar un parámetro en un tick conocido en mitad de la
  corrida.
- 4096 cuentas por vuelta: una cuenta son 0,0879°.

### El modelo

Con `θ` el ángulo verdadero del eje en cuentas y `m` la lectura:

```
m(θ) = θ + e(θ) + n,      e(θ) = Σ_{k=1..K} A_k · sin(2π k θ / 4096 + φ_k)
```

`e` es periódica en una vuelta *mecánica* porque el imán está pegado al eje: no
hay pares de polos que multipliquen nada. `K = 8` alcanza y sobra; lo que la
literatura anticipa es que `A_1` y `A_2` se lleven casi todo.

### El primer sospechoso no es el imán: es el filtro del sensor

El AS5600 arranca con el filtro lento en 16x, y eso son **2,2 ms de retardo de
respuesta al escalón** (ruido de salida 0,015° RMS). Con el filtro en 2x el
retardo baja a **0,286 ms** y el ruido sube a 0,043°. Los bits están en el
registro CONF: `SF` en 9:8 (`00`=16x, `11`=2x) y `FTH` en 12:10.

A velocidad constante un retardo `τ` no distorsiona nada: corre la lectura un
ángulo `ω·τ`. Pero ese corrimiento es enorme comparado con lo que estamos
persiguiendo. A 5 vueltas por segundo:

| Filtro | τ | Corrimiento a 5 rev/s |
|---|---|---|
| 16x (por omisión) | 2,2 ms | 3,96° = 45 cuentas |
| 2x | 0,286 ms | 0,51° = 5,9 cuentas |

Cuarenta y cinco cuentas de corrimiento contra un error que esperamos de unas
seis. Y el corrimiento es una *fase* que rota con la velocidad, así que una tabla
calibrada a una velocidad queda desfasada a otra. **Antes de calibrar nada hay
que poner el filtro en 2x**, y de paso el lazo de control gana 1,9 ms de retardo
que hoy está pagando sin saberlo. Esto es, muy probablemente, la mejora más
grande y más barata de todo el trabajo.

## 3. El problema difícil: qué es el sensor y qué es el motor

Un motor de continua con escobillas a `uff` constante no gira a velocidad
constante. Tiene ondulación de par por conmutación (una vez por cada segmento del
colector), *cogging*, y rozamiento que depende del ángulo. Y todo eso está
**enganchado al ángulo**, igual que el error del sensor. En una sola corrida a una
sola velocidad las dos cosas son literalmente indistinguibles: las dos producen
un residuo periódico en el ángulo.

Si se calibra sin separarlas, la tabla "corrige" la dinámica del motor metiéndola
en el sensor, y el resultado es peor que no hacer nada.

Hay dos discriminadores, y conviene saber cuál sirve para qué.

**El que funciona: barrer la velocidad.** El error del sensor es una función del
ángulo y no sabe a qué velocidad se lo recorre: `A_k` es constante en `ω`. Una
ondulación de par en cambio tiene que atravesar la mecánica para volverse
posición. Con `J δ̈ + b δ̇ = T_k cos(kθ)` y `θ = ωt`, la amplitud del residuo de
posición es

```
|δ_k| = T_k / sqrt( (J k² ω²)² + (b k ω)² )
```

o sea que cae como `ω⁻²` cuando manda la inercia y como `ω⁻¹` cuando manda el
rozamiento. **En un gráfico log-log de `A_k` contra `ω`, el sensor es una recta
horizontal y la mecánica tiene pendiente entre −1 y −2.** Ése es el experimento
central de todo este plan.

**El que no funciona para esto: invertir el sentido.** Es tentador y es lo que
hace SimpleFOC, pero conviene entender qué cancela y qué no. En el régimen
dominado por inercia el término `J ω² δ''` no cambia de signo al invertir `ω`,
así que la ondulación mecánica también es invariante al sentido: dar vuelta el
motor **no** separa la mecánica del sensor. Lo que sí cancela es el retardo: la
fase `ω τ` cambia de signo, así que promediar una corrida en cada sentido a la
misma velocidad devuelve la tabla referida a retardo cero, y la *diferencia* de
las dos mide `τ` y cualquier histéresis. Las dos corridas hay que hacerlas; sólo
hay que saber qué se le está pidiendo a cada una.

**El regalo: la desaceleración libre.** Si el motor se lleva a velocidad y
después se lo deja en `u = 0`, durante la desaceleración no hay corriente de
armadura y por lo tanto no hay ondulación de par de conmutación, que es la fuente
mecánica más grande. Queda el *cogging* y el rozamiento, que además se hacen
chicos rápido con la velocidad. Y como la velocidad barre continuamente de alta a
baja, **una sola captura de desaceleración da `A_k(ω)` en todo un rango**: se la
parte en ventanas de unas diez vueltas y se ajusta en cada ventana. Es el
experimento con mejor relación entre lo que cuesta y lo que dice.

## 4. Lo que hay que agregarle al firmware *antes* de medir

Nada de esto es la etapa de calibración todavía. Es lo mínimo para que los datos
signifiquen algo.

1. **Parámetro `sfilt` (u8, 0..3)** que escribe los bits `SF` del CONF del AS5600.
   Sin esto se mide con 2,2 ms de retardo y la fase de la tabla depende de la
   velocidad. Es un registro volátil, no hay que quemar nada.
2. **Canal `y_raw` (u16)**: la cuenta cruda del sensor, sin `offset`, sin signo
   invertido y sin corregir. Hoy `sensor_measurement()` devuelve `offset − counts`,
   así que desde el notebook la cuenta cruda se recupera como `(-y_uw) % 4096`
   con `offset = 0`; funciona, pero indexar la tabla es exactamente el lugar donde
   un signo equivocado se paga caro y no se nota. Cuesta dos bytes por fila.
3. **Diagnóstico de montaje**: leer una vez `AGC` (0x1A) y `MAGNITUDE` (0x1B/1C)
   además de `STATUS`, y exponerlos como parámetros de sólo lectura. `AGC` cerca
   del medio de su rango es la única evidencia barata de que el imán está a la
   distancia correcta. Es el mismo mecanismo perezoso que ya usa `read_status()`.

## 5. Los experimentos

Cada uno dice qué se corre, qué se calcula y qué decisión habilita. Las compuertas
(**G**) son puntos donde el plan se puede terminar temprano, que es la mitad de
para qué sirve un plan.

### E0 — Higiene del banco (30 min, sin datos que analizar)

Con el motor parado: `STATUS` con `MD` en 1 y `ML`/`MH` en 0, `AGC` a media
escala, `MAGNITUDE` estable. Medir con calibre lo que se pueda del montaje y
sacarle una foto al conjunto imán/sensor.

> **G0.** Si `ML` o `MH` están activos, o `AGC` está contra un extremo, el imán
> está a la distancia equivocada. **Se arregla el montaje y se vuelve a empezar.**
> Ninguna tabla compensa un AGC saturado.

### E1 — Piso de ruido (5 min)

Eje quieto y sujeto, `sfilt` en 2x, capturar 10 s. Calcular el desvío estándar de
`y_raw` en cuentas y su espectro.

Esto fija el umbral de detección de todo lo demás: nada por debajo de unas pocas
veces este número es una medición. Referencia de la hoja de datos: 0,043° RMS con
el filtro en 2x, que son 0,49 cuentas.

### E2 — Desaceleración libre (el experimento central)

```python
dev.mode, dev.offset, dev.cal = 0, 0, 0
dev.sfilt = 3                      # filtro 2x
dev.uff = 200                      # llevarlo a velocidad
df = dev.capture(25, events=[(3.0, 'uff', 0)])   # y soltarlo
```

Cinco repeticiones en cada sentido (`uff` positivo y negativo). De cada captura:
partir la parte de desaceleración en ventanas de ~10 vueltas y ajustar en cada
ventana los armónicos `k = 1..8` (§6). Salida: `A_k(ω)` y `φ_k(ω)`.

### E3 — Velocidad sostenida a varios comandos

Cuatro valores de `uff` que cubran un factor tres o cuatro de velocidad, 20 s
cada uno, en los dos sentidos. Es la contraparte "en régimen" de E2: confirma que
lo que se ve en la desaceleración también está cuando el motor tira, donde la
ondulación de conmutación sí existe.

Restricción de muestreo: a 500 Hz de telemetría hacen falta al menos 40 muestras
por vuelta para el octavo armónico con margen, o sea `ω ≤ 12,5 rev/s`. Si el
motor no baja de ahí, poner `tickdiv = 5` (1 kHz) y sacar canales de la tabla
para que la fila entre en el enlace.

> **G1 — ¿hay algo que corregir?** Si la suma de los armónicos aceptados da menos
> de 2 cuentas pico a pico (0,18°), el error del sensor está por debajo de su
> propia cuantización y del ruido. **Se documenta el resultado y no se hace la
> tabla.** Es un resultado, no un fracaso.
>
> **G2 — ¿qué armónicos son del sensor?** Se acepta el armónico `k` si
> (a) `A_k > 3σ_k`, (b) `A_k ≥ 0,3` cuentas, y (c) la pendiente de `log A_k`
> contra `log ω` está entre −0,3 y +0,3. Todo lo que tenga pendiente cerca de −1
> o −2 es el motor y **no entra en la tabla**.

### E4 — Retardo y sentido

Repetir un `uff` de E3 con `sfilt` en 16x y en 2x, en los dos sentidos. La fase
ajustada `φ_k` tiene que correrse en `k·ω·τ` y cambiar de signo con el sentido.

Predicción falsable: entre 16x y 2x, `φ_1` se corre 39 cuentas a 5 rev/s. Si el
corrimiento no aparece, el modelo de retardo está mal y hay que entender por qué
antes de seguir.

De acá sale también el `τ` medido, y la tabla se refiere a retardo cero
promediando los dos sentidos.

### E5 — Repetibilidad

Repetir E3 a un `uff` (a) después de apagar y encender la placa, (b) después de
frenar el eje con la mano y soltarlo, (c) al día siguiente.

> **G3.** Si las tablas de dos corridas separadas difieren en más de un tercio de
> su propia amplitud, hay algo suelto —el imán en el eje, el sensor en su
> soporte— y **hay que arreglar la mecánica antes que el software**. Una
> calibración de algo que se mueve es peor que ninguna.

### E6 — Control positivo (opcional, pero convence)

Correr el sensor deliberadamente ~0,5 mm del centro con una lámina y repetir E2.
`A_1` tiene que crecer de manera clara y proporcional. Es la prueba de que el
tubo de medición mide lo que decimos que mide, y no un artefacto del ajuste.

La variante fuerte, si el imán se puede desmontar: girarlo 180° respecto del eje
y volver a medir. Lo que rote con el imán es del imán; lo que se quede quieto es
del motor.

### E7 — Ajuste mecánico antes de la tabla

Si de E2/E3 sale un primer armónico grande, vale la pena centrar mejor el imán y
volver a medir. Corregir por tabla lo que se puede corregir con un tornillo es
mal negocio: la tabla es un modelo de primer orden de algo que en el fondo no es
lineal, y cuanto más chico sea lo que tiene que corregir, mejor se porta.

> **G4.** Se pasa a implementar la tabla cuando el error residual, ya mejorado
> mecánicamente todo lo razonable, siga por encima del umbral de G1.

### E8 — Validación de la corrección

Con la tabla ya en la placa, repetir E2 y E3 con `cal = 0` y `cal = 1` en la misma
sesión, sin tocar nada más.

> **G5 — criterio de éxito.** `A_1` y `A_2` medidos con `cal = 1` tienen que caer
> a menos de un quinto de lo que valían con `cal = 0`, y el desvío estándar de la
> velocidad instantánea tiene que bajar de manera visible. Si `A_1` no baja, la
> tabla está mal indexada o mal signada; si baja pero la velocidad no mejora, lo
> que quedaba era mecánico.

### E9 — El "y entonces qué"

Escalón de posición y seguimiento de rampa con `cal` en 0 y en 1, con las mismas
ganancias. Es la única medición que le importa a alguien que no esté mirando el
sensor: si el lazo no mejora, la tabla es un adorno.

## 6. El análisis

La tentación es desenrollar, ajustar una recta y mirar el residuo. Funciona, pero
el ajuste de la tendencia y el de los armónicos compiten por la misma varianza, y
una tendencia demasiado flexible se come el primer armónico. Conviene ajustar
las dos cosas **de una sola vez**, en un único problema de mínimos cuadrados:

```python
import numpy as np

def ajustar(t, cuentas, K=8, grado=8):
    """A_k y phi_k en cuentas, con `cuentas` ya desenrollado y en el dominio crudo."""
    ang = cuentas % 4096                            # ángulo medido, 0..4095
    ts  = (t - t.mean()) / (np.ptp(t) / 2)          # tiempo normalizado, condiciona el ajuste

    # Tendencia suave en el TIEMPO: la velocidad y su deriva.
    X = [ts**j for j in range(grado + 1)]

    # Error periódico en el ANGULO. Usar el ángulo medido en lugar del verdadero
    # es un error de segundo orden en A/4096: despreciable acá.
    for k in range(1, K + 1):
        X += [np.cos(2*np.pi*k*ang/4096), np.sin(2*np.pi*k*ang/4096)]

    X = np.column_stack(X)
    coef, *_ = np.linalg.lstsq(X, cuentas, rcond=None)

    a = coef[grado+1::2]    # cosenos
    b = coef[grado+2::2]    # senos
    return np.hypot(a, b), np.arctan2(a, b)
```

Notas que hacen la diferencia entre un ajuste y una medición:

- **`cuentas` sale de `y_raw`**, desenrollado en Python, no de `y_uw`: así el
  signo y el `offset` no entran en juego. Con `offset = 0` y sin el canal nuevo,
  es `-df.y_uw`.
- **El grado de la tendencia** se elige por separación espectral, no a ojo: los
  armónicos están a 1 ciclo por vuelta o más, o sea ≥50 ciclos en una corrida de
  50 vueltas, contra un polinomio de grado 8. No compiten. Con menos de ~20
  vueltas por ventana esto deja de ser cierto y el ajuste empieza a mentir.
- **Los huecos se manejan con `tick`**, no con el índice de fila. Una fila
  descartada por el protocolo no es una muestra faltante en el tiempo: es un
  salto, y desenrollar sobre un salto inventa una vuelta.
- **Las barras de error** salen de la covarianza del ajuste, `σ² (XᵀX)⁻¹`. Sin
  ellas la compuerta G2 no se puede aplicar.
- **Una iteración de refinamiento**: rehacer el ajuste con la base de Fourier
  evaluada en `ang − e(ang)` de la primera pasada. Si cambia algo apreciable, el
  error es grande y el modelo de primer orden está al límite.

Contra datos sintéticos con la geometría exacta de este banco —500 Hz, 20 s,
5 rev/s (85 vueltas, 100 muestras por vuelta), ruido de 0,5 cuentas RMS y una
deriva de velocidad del 30 %— el ajuste devuelve `A_1 = 5,98` contra 6,00 puestas
y las fases con menos de 0,002 rad de error. Un armónico ausente vuelve en
0,05 cuentas, que es de dónde sale el piso de 0,3 cuentas de la compuerta G2: seis
veces lo que el propio ajuste inventa.

## 7. Diseño de la etapa de calibración en el Arduino

### Dónde se aplica

En `sensor_measurement()`, sobre la cuenta cruda y **antes** de desenrollar:

```c
static int16_t sensor_measurement(void)
{
    int16_t counts = (int16_t)Sensor::counts();
    if (g_cal) counts = (int16_t)((counts - lut_lookup(counts)) & 0x0FFF);
    return wrapped_error(g_offset, counts);
}
```

Antes de desenrollar porque la tabla se indexa con el ángulo dentro de la vuelta,
y después de desenrollar ese ángulo ya no está. Y en el dominio de la cuenta
cruda, no en el de `y`, porque `y` lleva el signo invertido y el `offset`: dos
oportunidades de equivocarse a cambio de nada.

### La tabla

64 entradas `int8`, en unidades de 1/8 de cuenta, con interpolación lineal:

| | |
|---|---|
| Tamaño | 64 entradas × 1 byte = 64 B de SRAM |
| Índice | `counts >> 6`, fracción `counts & 63` |
| Unidad | 1/8 de cuenta = 0,011° |
| Rango | ±15,9 cuentas = ±1,4° |
| Separación angular entre entradas | 5,6° |

64 entradas representan sin problema hasta el octavo armónico (ocho puntos por
ciclo) y la interpolación lineal se hace cargo del resto. La unidad de 1/8 de
cuenta existe porque el error es de unas pocas cuentas: en cuentas enteras la
tabla tendría tres o cuatro valores distintos y sería un escalón, no una
corrección.

```c
// Corrección en cuentas, redondeada. La tabla está en octavos de cuenta.
static int8_t lut_lookup(int16_t counts)
{
    uint8_t i    = (uint8_t)(counts >> 6) & 0x3F;
    uint8_t frac = (uint8_t)counts & 0x3F;
    int16_t a = (int16_t)g_lut[i];
    int16_t b = (int16_t)g_lut[(i + 1) & 0x3F];
    int16_t eighths = (int16_t)((a * (64 - frac) + b * frac) >> 6);

    // Redondeo al medio hacia arriba. Con corrimiento aritmético `(e + 4) >> 3`
    // sirve para los dos signos; el `e < 0 ? -4 : 4` que uno escribe de reflejo
    // redondea mal los negativos chicos (-3/8 daría -1 en lugar de 0).
    return (int8_t)((eighths + 4) >> 3);
}
```

El redondeo a cuenta entera tira hasta media cuenta (0,044°), que está por debajo
del ruido del sensor con el filtro en 2x. Es una decisión, no un descuido: llevar
todo el camino de posición a octavos de cuenta se puede hacer después si la
validación muestra que hace falta, y toca la aritmética del lazo entero.

Costo: dos lecturas de tabla, dos multiplicaciones de 8×8 y un par de
corrimientos, del orden de 40 ciclos. Se paga una vez por período de control
(500 Hz), o sea 0,13 % del período. Nada.

### Persistencia y protocolo

La tabla vive en EEPROM (el ATmega328P tiene 1 KB, hoy sin usar) con encabezado
mágico y CRC16, y se carga a SRAM en `setup()`. Una tabla con CRC malo se ignora
y `cal` arranca en 0: una EEPROM virgen o corrupta tiene que dar un sensor sin
corregir, nunca uno corregido con basura.

Del lado del enlace, tres agregados que respetan lo que el protocolo ya hace:

| Comando | Respuesta |
|---|---|
| `lut` | ocho líneas `# l <i> <16 hex>` con la tabla entera |
| `lut <i> <16 hex>` | `# v lut <i> <16 hex>` — el eco, para verificar y no confiar |
| `lut save` / `lut clear` | `# ok` |

Ocho entradas por línea son 16 caracteres hexadecimales, en línea con el resto
del protocolo. Escribir la tabla entera son ocho comandos, unos 100 ms con el
espaciado de bytes que ya hace la computadora. El eco no es cortesía: el protocolo
ya distingue entre un *comando* deformado, que se rechaza a los gritos, y un
*valor* deformado, que se aceptaría en silencio; una tabla es toda valores.

Y dos parámetros:

- **`cal` (u8)**: 0 o 1. Existe para que E8 sea posible. Una corrección que no se
  puede apagar no se puede medir.
- **`sfilt` (u8)**: los bits `SF` del CONF, de §4.

## 8. Riesgos, y qué los detecta

| Riesgo | Cómo se manifiesta | Qué lo agarra |
|---|---|---|
| Se calibra la mecánica del motor como si fuera el sensor | la tabla mejora una velocidad y empeora otra | G2, la pendiente de `A_k(ω)` |
| El retardo del filtro se mete en la fase | la tabla anda a la velocidad de calibración y no a otras | E4, y poner `sfilt` en 2x desde el principio |
| El imán está flojo en el eje | la tabla no se repite entre encendidos | G3 |
| Aliasing: pocas muestras por vuelta | armónicos altos aparecen donde no están | ≥40 muestras/vuelta, verificado en cada captura |
| Huecos de telemetría desenrollados como saltos | vueltas fantasma en el desenrollado | reconstruir con `tick`, no con el índice |
| La tendencia se come el primer armónico | `A_1` chico y con barra de error grande | ≥20 vueltas por ventana de ajuste |
| Signo invertido en la corrección | `A_1` se duplica en vez de anularse | E8 con `cal` en 0 y en 1: es el chequeo, y es barato |

## 9. Orden de trabajo

1. Firmware de medición: `sfilt`, `y_raw`, diagnóstico de AGC/MAGNITUDE (§4).
2. E0, E1 — higiene y piso de ruido. Compuerta G0.
3. E2, E3 — desaceleración y régimen. Compuertas G1 y G2.
4. E4, E5 — retardo, sentido, repetibilidad. Compuerta G3.
5. E7 — ajuste mecánico y remedición. Compuerta G4.
6. Firmware de corrección: tabla, EEPROM, comandos `lut`, parámetro `cal` (§7).
7. E8, E9 — validación y efecto sobre el lazo. Compuerta G5.

Los pasos 1 a 5 no escriben una línea de la etapa de calibración. Es a propósito:
la mitad de las veces que este plan se ejecuta, la respuesta correcta aparece en
el paso 2 o en el 5, y es un tornillo.

## Fuentes

- [ODrive — Designing for Magnetic Encoders](https://docs.odriverobotics.com/v/latest/articles/magnetic-encoders.html)
- [SimpleFOC Community — SimpleFoc Sensor Eccentricity Calibration](https://community.simplefoc.com/t/simplefoc-sensor-eccentricity-calibration/2212)
- [Arduino-FOC-drivers (`CalibratedSensor`)](https://github.com/simplefoc/Arduino-FOC-drivers)
- [An Angle Error Compensation Method Based on Harmonic Analysis for Integrated Joint Modules — Sensors 20(6):1715](https://www.mdpi.com/1424-8220/20/6/1715)
- [Auto-calibration and noise reduction for the sinusoidal signals of magnetic encoders](https://www.researchgate.net/publication/321983477_Auto-calibration_and_noise_reduction_for_the_sinusoidal_signals_of_magnetic_encoders)
- [What do Lissajous figures tell us about encoder output?](https://www.motioncontroltips.com/what-do-lissajous-figures-tell-us-about-encoder-output/)
- [AS5600 Datasheet, ams v1-06, 2018-Jun-20](https://files.seeedstudio.com/wiki/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/res/Magnetic%20Rotary%20Position%20Sensor%20AS5600%20Datasheet.pdf) — registro CONF, filtro lento, retardo de respuesta
- [Synapticon — Encoder accuracy and calibration](https://doc.synapticon.com/circulo/hw/encoder_calibration/encoder_accuracy.html)
