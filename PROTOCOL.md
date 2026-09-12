# CtrlLink

Un protocolo serie para gobernar desde un notebook de Jupyter un lazo de control
que corre en un Arduino: fijar las ganancias, aplicar un escalón en la
referencia y traerse de vuelta la serie temporal que resulta.

```python
from ctrllink import CtrlLink

dev = CtrlLink()                 # o CtrlLink('COM3'), o '/dev/ttyACM0'
dev.pid_kp, dev.pid_ki, dev.ctl_mode = 2.5, 0.1, 1
df = dev.step('ref', 1024, pre=0.1, post=0.9)   # un DataFrame, t = 0 en el escalón
df.plot(x='t', y=['ref', 'y'])
```

El dispositivo describe por sí mismo sus parámetros y sus canales de telemetría,
así que la computadora no sabe nada de ningún sketch en particular. Agregar una
ganancia al firmware la hace aparecer en el notebook sin tocar una línea del lado
de Python.

## Diseño

**Un puerto, dos direcciones, sin capa de encuadre.** Las líneas que empiezan
con `#` son texto: respuestas a comandos, eventos, diagnósticos. Cualquier otra
línea es una fila de telemetría. Esa única convención hace que el flujo sea a la
vez interpretable por la máquina y legible en el Monitor Serie del IDE de
Arduino, y permite que una depuración con `Serial.print` conviva con una captura
en vivo en lugar de corromperla.

**Hexadecimal de ancho fijo, en lugar de decimal o binario.** La razón es el
*ancho fijo*, no el hexadecimal. Una fila mide siempre lo mismo, así que el
presupuesto de ancho de banda es exacto en vez de ser el del peor caso, `emit()`
puede pedir exactamente el lugar que necesita, y una captura se decodifica en una
sola llamada vectorizada en lugar de recorrerse línea por línea:

```python
raw = bytes.fromhex(b''.join(rows).decode())
arr = np.frombuffer(raw, dtype=structured_dtype)
```

El hexadecimal además es algo más barato de generar que el decimal —una consulta
a tabla y un `swap` por byte, contra las divisiones sucesivas de `itoa`—, pero el
`itoa` de avr-libc está escrito a mano en assembler, así que la diferencia es
como un 20 % del costo de codificar, no un orden de magnitud. Codificar una fila
de 4 × int16 cuesta del orden de 350 ciclos, que la escritura serie de Arduino
aproximadamente duplica; digamos 70 µs, o el 7 % de un período de control de
1 ms. Las dos cosas son secundarias frente a las ventajas de encuadre de arriba.

Un encuadre binario (COBS + CRC) ahorraría otro ~40 % de los bytes, pero a
1 Mbaud el ancho de banda no es la restricción activa, y costaría la posibilidad
de leer el flujo con los ojos. Subir la velocidad es la palanca más barata:
115200 → 1 Mbaud es 8,7×, mientras que hexadecimal → binario es 1,4×.

**1 Mbaud, no 115200.** En un AVR de 16 MHz, 1000000 es un divisor exacto
(UBRR=1); 115200 cae en UBRR=16, que da 117647 baudios reales, un 2,1 % de error.
La velocidad más alta es además la más exacta. 250000 y 500000 también son
exactas, por si el puente USB-serie de alguna placa no llega más arriba.

**Las dos direcciones no son igual de robustas.** La telemetría a 1 Mbaud es
sólida: medida sobre cinco segundos, cero filas perdidas y cero huecos. Los
comandos no. Un byte entrante llega cada 10 µs, el USART del AVR guarda dos, y
entre el muestreador de 5 kHz y el manejador de TWI de nI2C mantienen las
interrupciones deshabilitadas durante más que eso, así que se pierde alrededor
del 4 % de los bytes de un comando enviado de corrido. Marcar el muestreador
como `ISR_NOBLOCK` no lo arregla, porque el manejador de TWI también bloquea y
hacer reentrante una máquina de estados de I2C no vale el riesgo.

**El enlace se limpia solo.** Una celda de notebook se interrumpe en cualquier
parte: el botón de parar en mitad de una captura, un traceback a mitad de un
`set`. Ahí el dispositivo queda emitiendo filas que nadie va a leer y media línea
de comando en su buffer de entrada, y la celda siguiente hereda el desastre: los
datos de una captura aparecen como respuesta a un `get`. Así que toda operación
toma el enlace y, si sale por una excepción, lo deja limpio antes de dejarla
pasar: una línea vacía cierra el comando a medio escribir, un `stop` calla al
dispositivo y lo que quede en el camino de vuelta se descarta. La prueba de que
el flujo paró es la respuesta al `stop`, que el dispositivo da esté emitiendo o
no, y no el silencio: con `dec` alto una fila tarda más que cualquier ventana de
silencio razonable, y un `stop` se puede perder de ida como cualquier otro
comando, así que se repite hasta que llega la confirmación. Si ni eso se logra,
el enlace queda marcado y la operación siguiente lo intenta de nuevo antes de
mandar nada. Nunca hace falta reiniciar el kernel para recuperar el control de
la placa.

Por eso la computadora separa los bytes de un comando medio milisegundo, lo que
elimina la pérdida por completo. Los comandos son raros y cortos —un `set` tarda
unos 6 ms en enviarse—, así que no cuesta nada, y el tick que informa `# mark`
hace que incluso un escalón enviado en medio de una captura quede ubicado en la
muestra exacta. Encima de eso, la computadora reintenta todo comando que el
dispositivo declare no haber entendido, y verifica el valor que devuelve un `set`
en lugar de confiar en él: un *comando* deformado se rechaza a los gritos, pero
un *valor* deformado se aceptaría en silencio.

**Flujo continuo, no captura en buffer.** Un ATmega328P tiene 2 KB de SRAM, así
que una captura en buffer entra unas 175 muestras: 350 ms a 500 Hz, mucho menos
que un transitorio hasta el establecimiento. El flujo continuo no tiene límite de
duración; lo que queda acotado es la frecuencia de muestreo, y a 1 Mbaud ese
tope está muy por encima de cualquier lazo que pueda correr un UNO.

**Nunca bloquear el lazo de control.** `Serial.write` bloquea en cuanto se llena
el buffer de transmisión de 64 bytes, lo que frenaría el lazo y distorsionaría
justamente la dinámica que se está midiendo. `emit()` consulta primero
`availableForWrite()` y descarta la fila si no hay lugar, contando el descarte.
Una fila descartada deja un hueco visible en la secuencia de ticks; una escritura
bloqueante dejaría un error de temporización invisible.

## Ancho de banda

El puerto serie usa 10 bits por byte, así que el enlace transporta `baud/10`
bytes por segundo. Una fila cuesta `4 + Σ(anchos de canal) + 1` bytes, donde un
canal ocupa 4 caracteres hexadecimales si es int16 y 8 si es int32 o float.

| | fila | @115200 | @250k | @500k | @1M |
|---|---|---|---|---|---|
| 4 × int16 | 21 B | 548 Hz | 1,2 kHz | 2,4 kHz | 4,8 kHz |
| 4 × float | 37 B | 311 Hz | 676 Hz | 1,4 kHz | 2,7 kHz |

Eso es al 100 % de utilización. Conviene quedarse por debajo de la mitad: el
sketch `ControlDemo` corre seis canales a 500 Hz —tres int32 y tres int16, 41
bytes por fila—, que son 20,5 kB/s, o el 21 % de un enlace de 1 Mbaud.

## Protocolo de línea

Los comandos son ASCII, uno por línea, terminados en `\n`. Toda respuesta
termina en `# ok`, `# err <motivo>` o `# data`, así que la computadora puede
esperar un terminador definido en lugar de adivinar con una espera fija.

| Comando | Respuesta |
|---|---|
| `id` | `# id CtrlLink 1 <sketch> chans=<n> row=<bytes> dt_us=<n>` |
| `params` | un `# p <nombre> <tipo> <frac> <valor>` por parámetro |
| `chans` | un `# c <i> <nombre> <tipo> <escala> <unidad>` por canal |
| `get <nombre>` | `# v <nombre> <valor>` |
| `set <nombre> <valor>` | `# v <nombre> <valor>` |
| `start` | el encabezado del flujo, terminado en `# data`, y después las filas |
| `stop` | `# end rows=<n> drops=<n>` |

Los tipos son `i8 u8 i16 u16 i32 u32 f32`. Una línea vacía se ignora, así que
enviar una es una forma segura de resincronizar.

El `<valor>` que va por el cable es siempre el almacenamiento crudo del
dispositivo. `<frac>` dice cuántos bits fraccionarios lleva ese almacenamiento,
así que la computadora lee `raw / 2**frac` y escribe `round(valor * 2**frac)`.
Un dispositivo puede entonces guardar una ganancia en Q22 y el polo de un filtro
en Q16 —lo que su aritmética prefiera— mientras la computadora los sigue fijando
como `0.5` y `0.02`, y la conversión ocurre del lado que tiene unidad de punto
flotante y no tiene plazo que cumplir. `frac = 0` es un entero común.

Potencias de dos en lugar de la `escala` flotante arbitraria de un canal, porque
eso es lo que es un formato de punto fijo, y porque cruza el cable de forma
exacta: ningún número fijo de decimales imprime de manera útil tanto una escala
Q22 (2,4e-7) como una Q30 (9,3e-10). Todo lo que necesite una escala que no sea
potencia de dos —grados por cuenta, miliamperes por LSB— es un canal, y un canal
tiene una.

### Flujo

```
# begin
# rate dt_us=1000 dec=1
# col tick u16 1 tick
# col ref i16 0.0878906 deg
# col y i16 0.0878906 deg
# col u i16 1.0000000 pwm
# data
0412CDB90C800076
0413CDB90C830074
```

La columna cero es siempre un contador de ticks de 16 bits, que se incrementa una
vez por período de control se emita o no una fila. Da la vuelta cada 65536
períodos y la computadora la desenrolla. Multiplicar por `dt_us` para obtener
segundos; multiplicar un canal por su `escala` para obtener unidades de
ingeniería. Todos los valores son hexadecimal sin signo del almacenamiento crudo,
con el nibble más significativo primero: un float son sus cuatro bytes IEEE-754,
no una conversión a decimal, lo que lo hace gratis de emitir en un dispositivo
sin unidad de punto flotante.

El dispositivo sólo emite valores; la conversión de unidades ocurre en la
computadora.

### Eventos durante una captura

`set` funciona mientras hay flujo, y el dispositivo informa el tick en el que el
valor entró en vigencia:

```
0412CDB90C800076
# mark 1043 ref 1024
0413CDB90C830074
```

Esto es lo que hace exacta una respuesta al escalón. La computadora programa el
escalón con una precisión del orden del milisegundo, pero la *medición* usa el
tick que informa el dispositivo, así que la fluctuación de la computadora nunca
entra en los datos.

## API del lado de la computadora

`CtrlLink(port, baud=1_000_000)` abre el puerto, resetea la placa y descubre el
dispositivo. El reset se fuerza con un flanco de bajada de DTR: abrir el puerto
no alcanza, porque la línea puede venir activada de la conexión anterior —en
macOS con un puente CH340 se queda así—, y entonces no hay flanco y la placa
sigue corriendo con el estado que le dejó la corrida pasada. `reset_wait=0` se
engancha a un sketch que ya está corriendo, sin resetear nada.

- Los parámetros son atributos, siempre en unidades reales: `dev.pid_kp = 2.5`,
  `print(dev.pid_kp)`. `dev.params` los lee todos de vuelta.
- `dev.capture(duration, events=[(retardo, nombre, valor), ...])` → DataFrame.
- `dev.step(nombre, valor, pre=0.1, post=0.9, back=None)` → DataFrame con `t = 0`
  en el escalón. Si se interrumpe, `back` se restituye igual: del otro lado del
  cable puede haber un motor empujando contra un tope.
- `dev.resync()` deja el enlace en un estado conocido. Se llama sola cuando hace
  falta; está expuesta para forzarla a mano después de algo que el módulo no vio
  pasar.

El DataFrame trae `t` en segundos más una columna por canal en unidades de
ingeniería. `df.attrs` guarda `dt_us`, `dec`, `units`, `marks`, `notes`, los
contadores `rows`/`drops` del propio dispositivo, y `gaps`, la cantidad de cortes
en la secuencia de ticks, que vale 0 en una captura limpia.

## Firmware

Se declaran los parámetros ajustables y la telemetría como tablas en PROGMEM y
el resto sale solo:

```cpp
static const CtrlParam PROGMEM g_params[] = {
    { "pid_kp", CTRL_I32, &g_pid.kp, 22 },  // Q22: la PC fija 0.5, el dispositivo guarda 2097152
    { "ref", CTRL_I16, &g_ref,  0 },   // un entero común
};

static const CtrlChannel PROGMEM g_channels[] = {
    { "y", CTRL_I16, &g_y, 360.0f / 4096, "deg" },
    { "u", CTRL_I16, &g_u, 1.0f,          "pwm" },
};

void setup() {
    CtrlLink::set_id(F("MySketch"));
    CtrlLink::begin(1000000, g_params, 2, g_channels, 2, /* dt_us */ 1000);
}

void loop() {
    if (tick_due()) {
        control_step();      // escribe g_y, g_u
        CtrlLink::emit();    // una fila, nunca bloquea
    }
    CtrlLink::poll();        // a lo sumo un comando por llamada
}
```

`emit()` lee los canales a través de sus direcciones, así que hay que llamarlo
desde el mismo contexto que los escribe: `loop()`, no una ISR. Los nombres tienen
12 caracteres como máximo, que es lo que permite ponerles un prefijo de módulo: una
tabla de tres docenas de parámetros planos no dice quién es dueño de cuál, y
`pid_kp` contra `ang_offset` contra `mot_top` lo dice sin ir a leer el sketch. Una tabla de canales que produzca una fila más larga
que el buffer de transmisión es rechazada por `begin()`, en lugar de descartar
todas las muestras en silencio.

## Medido

Sobre un clon de UNO con puente CH340G, a 1 Mbaud, con cuatro canales int16 a
1 kHz:

| | |
|---|---|
| frecuencia de muestreo | 1000,2 Hz durante 5 s |
| filas | 4983 enviadas, 4983 recibidas |
| filas descartadas | 0 |
| huecos de tick | 0 |
| uso del enlace | 21 kB/s, 21 % de 1 Mbaud |
| peor retardo de atención del lazo | 344 µs sobre un período de 1000 µs |

El CH340 merece un párrafo aparte: es el puente que traen la mayoría de los
clones de UNO y el que suele darse por limitado a velocidades bajas, y aguantó
1 Mbaud sin perder una sola fila. El retardo de atención es el costo honesto de
correr la ley de control dentro de `loop()` al lado del manejo del puerto serie;
el muestreo en sí es rígido, porque lo gobierna el Timer2, así que un cálculo
tardío aparece como fluctuación en `u`, no en `y`. `lop_late` lo informa, y se
puede escribir, así que conviene ponerlo en cero antes de una corrida para medir
esa corrida.

## Instalación

```
python3 -m venv .venv
./.venv/bin/pip install -r python/requirements.txt jupyterlab matplotlib ipykernel
./.venv/bin/python -m ipykernel install --user --name arduino-control \
    --display-name "Arduino Control (.venv)"
./.venv/bin/jupyter lab notebooks/control_demo.ipynb
```

## Organización

- `libraries/CtrlLink/` — el protocolo, lado placa
- `python/ctrllink.py` — el protocolo, lado computadora
- `python/test_ctrllink.py` — pruebas del lado computadora contra una simulación
  del dispositivo fiel byte a byte
- `ControlDemo/` — lazo de posición con AS5600 a 500 Hz, muestreado a 5 kHz
- `notebooks/control_demo.ipynb` — demostración completa: salud del enlace,
  escalones en lazo abierto y cerrado, un barrido de ganancia, cambios en la
  frecuencia del lazo

```
arduino-cli compile --fqbn arduino:avr:uno --libraries ./libraries \
  --build-property compiler.c.extra_flags=-O2 \
  --build-property compiler.cpp.extra_flags=-O2 \
  --build-property compiler.c.elf.extra_flags=-O2 \
  ControlDemo
arduino-cli upload  --fqbn arduino:avr:uno --libraries ./libraries -p <puerto> ControlDemo
```

`ControlDemo` se apropia del Timer2 para el muestreador de 5 kHz, así que
`analogWrite` deja de funcionar en los pines 3 y 11; los pines 9 y 10 (Timer1) y
5 y 6 (Timer0) no se ven afectados.
