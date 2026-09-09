# Un lazo de control con las perillas afuera

Un Arduino UNO cierra un lazo de control sobre un motor con un sensor magnético
de ángulo, y un notebook de Jupyter le mueve las ganancias, le aplica escalones y
se trae de vuelta la serie temporal. La idea es poder pasar de una pregunta a una
medición sin recompilar nada: se cambia un número en una celda, se corre, y sale
el gráfico.

Está pensado como banco de trabajo para **Control Clásico y por Variables de
Estado** (Ingeniería Electrónica, UNRN): lo que en las prácticas se calcula a mano
y se verifica con `python-control`, acá se mide sobre un motor de verdad, con su
saturación, su ruido y su cuantización.

```python
dev = sync_board()
dev.gains(kp=0.002, ki=0.05)          # ganancias en tiempo continuo
dev.mode = MODE_PID

df = dev.step('ref', dev.deg(90))     # escalón de 90 grados
df.plot(x='t', y=['ref', 'y_uw'])     # t = 0 en el escalón, con precisión de una muestra
```

Lo que se puede hacer, y es lo que recorre el notebook:

- identificar la planta en **lazo abierto**, aplicando un escalón al PWM;
- cerrar el lazo sobre la **posición** y mirar el error de régimen permanente;
- **barrer una ganancia** y superponer las respuestas;
- seguir una **rampa**, y ver por qué un proporcional solo se queda atrás;
- cerrar el mismo PID sobre la **corriente**, que es una planta mucho más rápida;
- cambiar la **frecuencia del lazo** y ver qué les pasa a las mismas ganancias.

El lazo corre a 500 Hz con aritmética entera de punta a punta —ni una instrucción
de punto flotante—, muestrea el sensor a 5 kHz con período rígido, y emite
telemetría a 1 Mbaud sin perder filas. Los detalles de cómo y por qué están en
[`PROTOCOL.md`](PROTOCOL.md).

---

## Qué hace falta

### Hardware

| Señal | Pin del UNO | |
|---|---|---|
| AS5600 SDA | A4 | |
| AS5600 SCL | A5 | |
| AS5600 VDD / GND | 5V / GND | |
| Salida del ACS712 | A0 | opcional |
| L298N `ENA` (PWM, 1 kHz) | 9 | opcional |
| L298N `IN1` | 6 | opcional |
| L298N `IN2` | 7 | opcional |

**La medición de corriente lee A0 contra la referencia interna de 1,1 V**, no
contra los 5 V: un LSB pasa de 4,9 mV a 1,07 mV, y para un motor chico ésa es la
diferencia entre medir y no medir. El precio es el techo —la entrada no puede pasar
de 1,1 V—, así que sirve para un sensor cuyo reposo caiga por debajo de eso y no
para un ACS712 alimentado a 5 V, que reposa en 2,5 V. `SENSE_REF_INTERNAL = false`
vuelve a AVcc.

Dos números que conviene verificar una vez por banco, los dos en el sketch:

- `ADC_REF_MV`. El bandgap interno está especificado entre 1,0 y 1,2 V, o sea
  ±10 % de error de ganancia entre chips. Se mide sin instrumental leyendo el canal
  14 del multiplexor del ADC contra AVcc.
- `SENSE_MV_PER_A`. La sensibilidad del sensor, que es lo único que convierte
  cuentas en amperes. `bringup()` no la puede verificar: calibra el cero, que
  tiene una condición conocida —el puente abierto—, pero para la ganancia haría
  falta una corriente conocida. Un tester en serie con el motor, una vez, alcanza.

El AS5600 necesita un imán **magnetizado diametralmente** girando sobre el chip,
a un par de milímetros. Las plaquetas de AS5600 traen su propio regulador y los
pull-ups del bus; un chip pelado en modo 3,3 V necesita adaptación de niveles.

El motor va al puente en H, y el puente a su propia fuente: el UNO le da la
lógica, nunca la potencia. El sketch modula `ENA` y usa `IN1`/`IN2` para el
sentido, así que el accionamiento es **bidireccional**, de -255 a 255. Un puente
cableado para un solo cuadrante se declara con `dev.bidir = 0`, y ahí el comando
se recorta en cero y el anti-windup se entera; es un parámetro y no un `#define`,
así que se contesta desde el notebook y sin recompilar.

**Se puede empezar sin motor y sin medición de corriente.** Sin nada conectado a
los pines 9, 6 y 7 el lazo corre igual y la telemetría se comporta de manera
idéntica: alcanza con girar el imán a mano para ver al sensor y al filtro
trabajar; `bringup(motor=False)` saltea la parte que lo haría girar. Sin ACS712,
`bringup()` detecta que la entrada quedó contra el riel del ADC y lo dice; todo lo
demás sigue en pie.

Incluso sin el AS5600 el lazo mantiene su período: al no obtener respuesta, el
muestreo pasa a sondear el bus dos veces por segundo en lugar de cinco mil, y
`bringup()` informa `sensor: no contesta`. Sirve para probar la cadena completa
—compilar, grabar, capturar, graficar— antes de tener el sensor sobre la mesa,
teniendo en cuenta que el ángulo queda congelado en cero.

> ⚠️ **Con el motor conectado, el motor se mueve.** Conviene revisar que el eje
> esté libre antes de correr cualquier celda.

**La zona muerta manda sobre el ajuste.** Un L298N alimentado con 5 V se come unos
2 V, así que por debajo de `u ≈ 60` el motor no arranca, y sin carga da 72
vueltas/s a fondo. Entre esas dos paredes la ventana de ganancias es angosta: `kp`
chico deja el eje parado antes de la referencia, `kp` grande lo pasa de largo y
oscila, y `ki` se carga mientras el eje está parado y después lo manda varias
vueltas de largo. Los valores del notebook —`kp = 0.1`, `kd = 0.002`, `ki = 0`— son
los que se midieron en este banco. Con un puente MOSFET la zona muerta casi
desaparece y el ajuste vuelve a parecerse al del libro.

### Software

- [Arduino CLI](https://arduino.github.io/arduino-cli/) en el `PATH`, con el core
  de AVR: `arduino-cli core install arduino:avr`
- Python 3.9 o posterior

No hace falta el IDE de Arduino: el notebook compila y graba solo.

---

## Puesta en marcha

**1. Clonar, con los submódulos.** La biblioteca I2C `nI2C` es un submódulo, y sin
ella no compila nada:

```
git clone --recurse-submodules git@github.com:nicber/arduino-jupyter.git
cd arduino-jupyter
```

Si el repositorio ya estaba clonado sin submódulos:
`git submodule update --init`.

**2. Armar el entorno de Python.**

```
python3 -m venv .venv
./.venv/bin/pip install -r python/requirements.txt jupyterlab ipykernel
./.venv/bin/python -m ipykernel install --user --name arduino-control \
    --display-name "Arduino Control (.venv)"
```

En Windows, las mismas líneas con `.venv\Scripts\pip` y `.venv\Scripts\python`.

**3. Probar sin placa.** Las pruebas del lado computadora corren contra una
simulación del dispositivo fiel byte a byte, así que no hace falta hardware para
verificar que la instalación quedó bien:

```
./.venv/bin/python python/test_ctrllink.py
```

Tienen que pasar las 70 verificaciones.

**4. Verificar el sensor**, antes de meter el lazo en el medio. Se graba el sketch
de puesta en marcha y se abre el monitor serie a **115200**:

```
arduino-cli compile -b arduino:avr:uno --libraries ./libraries AS5600_Bringup
arduino-cli upload  -b arduino:avr:uno --libraries ./libraries -p <puerto> AS5600_Bringup
```

Vuelca la configuración del AS5600 y después el ángulo a 5 Hz. Lo que hay que
mirar es `STATUS=[MD -- --]`, que es imán detectado y AGC en rango, y que el
ángulo siga al imán al girarlo. `AS5600_Loop5k` es el paso siguiente y opcional:
muestrea a 5 kHz e informa la frecuencia efectiva, para confirmar que el bus
aguanta el ritmo.

**5. Abrir el notebook.**

```
./.venv/bin/jupyter lab notebooks/control_demo.ipynb
```

Elegir el kernel *Arduino Control (.venv)* y correr las celdas en orden. La
primera compila y graba `ControlDemo` sola; la placa se encuentra sin nombrar
ningún puerto. La sección 1 es `dev.bringup()`, que verifica el equipo subsistema
por subsistema y es lo que conviene correr ante cualquier duda.

Cada celda arranca con `sync_board()`, que recompila si se editó el sketch, graba
si cambió el binario y reabre el enlace, lo que resetea la placa. Por eso las
celdas se pueden correr en cualquier orden: ninguna depende de lo que dejó la
anterior.

---

## Qué se puede tocar desde el notebook

Los parámetros son atributos, siempre en unidades reales:

| Parámetro | Qué es |
|---|---|
| `kp`, `ki`, `kd` | ganancias del PID, **por muestra** |
| `mode` | `MODE_OPEN`, `MODE_PID`, `MODE_RAMP` |
| `target` | `POSITION` o `CURRENT`: sobre qué magnitud cierra el lazo |
| `ref`, `refrate` | referencia y pendiente de rampa, en unidades del `target` |
| `uff` | comando de lazo abierto / prealimentación |
| `tickdiv` | divisor del muestreador de 5 kHz: 10 → 500 Hz (por omisión), 5 → 1 kHz, 50 → 100 Hz |
| `bidir` | 1 si el puente acciona en los dos sentidos; 0 lo recorta en cero |
| `uinvert` | 1 si un comando positivo hace *bajar* el ángulo medido |
| `pwmtop` | TOP del Timer1: la frecuencia del PWM, `f = 16 MHz / (2·pwmtop)` |
| `alpha_y`, `alpha_i`, `alpha_e` | polos de los filtros de posición, corriente y error |
| `offset` | cuenta del sensor de ángulo que se lee como cero |
| `izero` | LSB del ADC que se lee como corriente cero |
| `maxlate`, `missed`, `sovr`, `serr` | contadores de salud del lazo |
| `spres`, `mstat` | estado del sensor: si contesta en el bus, y qué dice del imán |

`bench.py` agrega encima las conversiones de este equipo, que son las que conviene
usar: `dev.gains(kp, ki, kd)` toma las ganancias **en tiempo continuo** y las
convierte a por-muestra, `dev.deg()` y `dev.ma()` escriben referencias en grados y
miliamperes, `dev.smooth('y', tau)` fija un filtro por constante de tiempo,
`dev.pwm(20000)` fija la frecuencia del PWM en Hz y devuelve la que realmente
quedó, `dev.zero()` toma la posición actual como cero y `dev.zero_current()` hace
lo propio con el sensor de corriente, que es la calibración de offset que `bringup()`
ya corre sola.

Para cambiar la *ley* de control —y no sus parámetros— el archivo es
`ControlDemo/ControlDemo.ino`. Agregar un controlador es agregar una función junto
a `controller_pid()` y un caso al `switch` de `control_step()`; agregar una
realimentación nueva es una rama en `target_error()`. Un parámetro nuevo es una
línea en la tabla `g_params[]`, y aparece solo en el notebook: del lado de Python
no hay nada que cambiar.

---

## Cuando algo no anda

| Síntoma | Dónde mirar |
|---|---|
| `sync_board()` no encuentra `arduino-cli` | está en el `PATH`? En Windows hay que reabrir la terminal después de instalarlo: el `PATH` se lee una sola vez al arrancar |
| «no se pudo abrir el puerto» | algo más lo tiene tomado: el monitor serie del IDE, o un kernel de una sesión anterior. Un puerto serie es exclusivo |
| «no se encontro ningun puerto serie USB» | placa desenchufada, o cable de sólo alimentación |
| `bringup` marca falla en `sensor` | el AS5600 no contesta en el bus: SDA (A4), SCL (A5), alimentación, pull-ups |
| `bringup` marca falla en `iman` | el sensor contesta pero el imán está ausente, muy lejos o muy cerca; el mensaje dice cuál |
| `bringup` marca falla en `bus i2c` | errores intermitentes con el sensor presente: cableado o pull-ups |
| `bringup` marca falla en `cero de i` | el sensor de corriente no reposa en media escala: sin alimentar, mal cableado, o no es un ACS712 de 5 V |
| `bringup` marca falla en `polaridad` | el comando y el sensor tienen signos opuestos: el lazo de posición realimenta en positivo y se escapa. Dar vuelta `uinvert`, o los dos cables del motor |
| `bringup` marca falla en `sentido` | el motor gira para el mismo lado con las dos polaridades: `IN1` (6) e `IN2` (7) intercambiados, o uno sin conectar |
| `bringup` dice «no se pudo evaluar» | falta el sensor del que esa verificación depende; arreglar primero el que sí falla |
| «el dispositivo declara sus parametros en un formato anterior» | la placa tiene grabado un sketch viejo: `sync_board(force_upload=True)` |
| se interrumpió una celda en medio de una captura | nada: la operación siguiente resincroniza el enlace sola. `dev.resync()` lo fuerza a mano |
| se pierden períodos de control | subir `tickdiv`, o sacarle trabajo al paso de control |
| se descartan filas de telemetría | subir `dec`, o emitir menos canales |

Toda captura verifica su propia salud y avisa por `stderr` si el lazo perdió
períodos o si se perdieron filas: una serie temporal a la que le faltan muestras
se ve igual que una sana hasta que uno va a fijarse.

**Periféricos que se apropia `ControlDemo`:** el Timer2, así que `analogWrite()`
en los pines 3 y 11 y `tone()` dejan de funcionar; el Timer1, que modula el puente
con su propio TOP, así que `analogWrite()` en los pines 9 y 10 y `Servo` dejan de
servir; y el ADC, que se maneja directamente, así que no hay que llamar a
`analogRead()`. El Timer0 queda intacto: `millis()` y el PWM de los pines 5 y 6
andan como siempre.

**El PWM del puente va a 1 kHz**, y es un parámetro: `dev.pwm(hz)`, entre 122 Hz y
31,4 kHz. El Timer1 corre phase-correct con su propio TOP y preescalador 1, así que
`f = 16 MHz / (2·pwmtop)` y 1 kHz es un TOP de 8000, exacto.

En abstracto conviene modular más rápido: fuera del rango audible, y con la
ondulación de corriente —inversamente proporcional a la frecuencia— lejos de la
banda del lazo. Pero un L298N alimentado con 5 V no lo tolera, y el motivo vale la
pena. Es un puente de Darlington bipolares: cae unos 2 V entre sus dos lados, así
que al motor le llegan ~2,5 V, y tarda unos 2 µs en conmutar. A 20 kHz —períodos de
50 µs— lo que se pierde en cada transición, más la recuperación de los diodos del
módulo, se lleva una fracción grande de un tiempo de encendido que ya venía escaso.
Medido en este banco: **a 20 kHz el motor no arranca y a 1 kHz anda**. Con un
puente MOSFET —un TB6612FNG, un DRV8833, que caen 0,3 V en lugar de 2— nada de esto
haría falta y `dev.pwm(20000)` sería lo correcto.

El piso de 255 en el TOP es porque más abajo el ciclo de trabajo tendría menos
escalones que el comando. El pin es forzado: el Timer0 (5 y 6) lleva `millis()` y
el Timer2 (3 y 11) es el muestreador, así que el único libre es el Timer1, o sea
los pines 9 y 10.

---

## Organización

```
ControlDemo/             el lazo de control: ley, parámetros y telemetría
AS5600_Bringup/          verificación del sensor, con volcado de configuración
AS5600_Loop5k/           prueba de muestreo a 5 kHz
libraries/CtrlLink/      el protocolo, lado placa
libraries/ControlMath/   punto fijo y filtros enteros
libraries/AS5600Async/   lectura asincrónica del AS5600
libraries/nI2C/          bus I2C por interrupciones (submódulo, de terceros)
python/ctrllink.py       el protocolo, lado computadora
python/bench.py          compilación, conexión y unidades de este equipo
python/test_ctrllink.py  pruebas, no necesitan hardware
notebooks/               el notebook de demostración
PROTOCOL.md              el protocolo: diseño, formato de línea y mediciones
```

Compilar y grabar a mano, si hiciera falta:

```
arduino-cli compile -b arduino:avr:uno --libraries ./libraries \
  --build-property compiler.c.extra_flags=-O2 \
  --build-property compiler.cpp.extra_flags=-O2 \
  --build-property compiler.c.elf.extra_flags=-O2 \
  ControlDemo
arduino-cli upload  -b arduino:avr:uno --libraries ./libraries -p <puerto> ControlDemo
```

Las tres `--build-property` son las que compilan con optimización plena. El core
de AVR trae `-Os` —optimizar por tamaño—, y este sketch quiere ciclos y no bytes;
`sync_board()` las pasa solas, así que sólo hacen falta al compilar a mano. El
porqué de `-O2` y no `-O3` está comentado arriba de `BUILD_PROPERTIES`, en
`python/bench.py`.
