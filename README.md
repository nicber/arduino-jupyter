# Un banco para medir un motor desde un notebook

Un Arduino UNO pone un PWM sobre un motor de continua, lee un sensor magnético
de ángulo a 5 kHz y una corriente, y manda las filas a 1 Mbaud. Un notebook de
Jupyter le aplica escalones y se trae de vuelta la serie temporal: tiempo,
comando, ángulo, velocidad y corriente, que es lo que hace falta para escribir
un modelo del motor y después comprobarlo. La idea es poder pasar de una
pregunta a una medición sin recompilar nada: se cambia un número en una celda,
se corre, y sale el gráfico.

Está pensado como banco de trabajo para **Control Clásico y por Variables de
Estado** (Ingeniería Electrónica, UNRN): lo que en las prácticas se calcula a
mano y se verifica con `python-control`, acá se mide sobre un motor de verdad,
con su zona muerta, su saturación, su ruido y su cuantización.

```python
dev = sync_board()
dev.uff = 100                                        # comando sobre el puente
df = dev.step('uff', 220, pre=0.3, post=1.2, back=0) # escalón, t = 0 en el escalón
t, w = ensayo.velocidad(df)                          # rad/s, derivada de este lado
ensayo.guardar(df, 'datos/escalon_100_220.csv')      # t, u, theta, omega, i
```

Lo que se puede hacer, y es lo que recorre el notebook:

- verificar el equipo parte por parte, y saber cuál falla si algo falla;
- medir la **curva estática** del actuador: la ganancia y la zona muerta;
- aplicar **escalones** de PWM y medir la constante de tiempo;
- mirar la **corriente** en el arranque;
- **guardar cada ensayo** en un archivo con las unidades de un modelo.

En la placa no hay ley de control ni filtros: el comando va derecho al puente y
el ángulo vuelve tal como lo entregó el sensor. Todo lo que se hace con la
medición --derivar, filtrar, ajustar-- pasa del lado de la computadora, donde se
ve y se puede cambiar. Un filtro en la placa se confunde con la planta que se
está midiendo, y por eso no hay ninguno. Los detalles de cómo y por qué están en
[`PROTOCOL.md`](PROTOCOL.md).

---

## Qué hace falta

### Hardware

El recorrido completo del montaje --las configuraciones por las que crece el
banco, los diagramas de cableado y qué se puede medir con cada una-- está en
[`notebooks/hardware.ipynb`](notebooks/hardware.ipynb), que es además el que se
muestra en clase. Su sección 2.2 es el caso de **un solo cuadrante** --un puente
cableado para un lado, o un transistor y un diodo--, que es el de muchos bancos:
alcanza con no mandar comandos negativos.

| Señal | Pin del UNO | |
|---|---|---|
| AS5600 SDA | A4 | |
| AS5600 SCL | A5 | |
| AS5600 VDD / GND | 5V / GND | |
| Salida del ACS712 | A0 | opcional |
| L298N `ENA` (PWM, 1 kHz) | 9 | opcional |
| L298N `IN1` | 6 | opcional |
| L298N `IN2` | 7 | opcional |

**La medición de corriente lee A0 contra Vcc.** Un ACS712 es bipolar y
ratiométrico: reposa en la mitad de su alimentación --2,5 V con 5 V-- para poder
bajar cuando la corriente cambia de sentido, así que contra la referencia interna
de 1,1 V satura en reposo y no mide nada. Contra Vcc reposa en media escala por
construcción. Lo que se paga es resolución: un LSB son 4,9 mV, y con 185 mV/A eso
son 26 mA por cuenta. Un motor chico consume decenas de mA en régimen, o sea una
cuenta o dos; lo que sí se ve es el arranque, que son cientos.

Dos números que conviene verificar una vez por banco, los dos en el sketch:

- `SENSE_MV_PER_A`. La sensibilidad del sensor, que es lo único que convierte
  cuentas en amperes: 185 para el ACS712 de 5 A, 100 para el de 20 A, 66 para el
  de 30 A. `bringup()` no la puede verificar: calibra el cero, que tiene una
  condición conocida --el puente abierto--, pero para la ganancia haría falta una
  corriente conocida. Un tester en serie con el motor, una vez, alcanza.
- `ADC_REF_MV`. Vcc, que se mide una vez con un tester.

El AS5600 necesita un imán **magnetizado diametralmente** girando sobre el chip,
a un par de milímetros. Las plaquetas de AS5600 traen su propio regulador y los
pull-ups del bus; un chip pelado en modo 3,3 V necesita adaptación de niveles.

El motor va al puente en H, y el puente a su propia fuente: el UNO le da la
lógica, nunca la potencia. El sketch modula `ENA` y usa `IN1`/`IN2` para el
sentido, así que el comando va de -255 a 255. Si un comando positivo hace bajar
el ángulo medido, eso es de qué lado están los cables del motor y de qué lado
mira el imán: `ensayo.signo()` lo detecta y `ensayo.normalizar()` lo aplica, o
se dan vuelta los dos cables.

**Se puede empezar sin motor y sin medición de corriente.** Sin nada conectado a
los pines 9, 6 y 7 el muestreo corre igual y la telemetría se comporta de manera
idéntica: alcanza con girar el imán a mano para ver al sensor trabajar;
`bringup(motor=False)` saltea la parte que lo haría girar. Sin ACS712,
`bringup()` detecta que la entrada quedó contra el riel del ADC y lo dice.

Incluso sin el AS5600 el muestreo mantiene su período: al no obtener respuesta,
pasa a sondear el bus dos veces por segundo en lugar de cinco mil, y `bringup()`
informa `sensor: no contesta`. Sirve para probar la cadena completa --compilar,
grabar, capturar, graficar-- antes de tener el sensor sobre la mesa.

> ⚠️ **Con el motor conectado, el motor se mueve.** Conviene revisar que el eje
> esté libre antes de correr cualquier celda.

**La zona muerta manda.** Un L298N alimentado con 5 V se come unos 2 V, así que
por debajo de `u ≈ 60` el motor no arranca, y sin carga da 72 vueltas/s a fondo.
Entre esas dos paredes vale un modelo lineal, y afuera no; medirlo es la mitad
de la práctica. Con un puente MOSFET la zona muerta casi desaparece.

### Software

- [Arduino CLI](https://arduino.github.io/arduino-cli/) en el `PATH`, con el core
  de AVR: `arduino-cli core install arduino:avr`
- Python 3.9 o posterior

No hace falta el IDE de Arduino: el notebook compila y graba solo.

### Clones del UNO

Sirve cualquier placa con un ATmega328P a 16 MHz, y también los clones que no
son exactamente eso. Nada de lo que sigue hay que configurar: el notebook se da
cuenta solo. Está acá porque cuando algo falla conviene saber qué se estaba
compensando.

**El bootloader.** Un UNO escucha a 115200 y muchos clones baratos traen el
bootloader viejo del Nano, que escucha a 57600. Elegir mal no da un error legible
sino diez líneas de «not in sync». `sync_board()` prueba los dos, se queda con el
que anduvo y lo recuerda por puerto; el primer intento cuesta unos segundos una
sola vez. Un tipo de placa que no esté en la lista se agrega en `UPLOAD_FQBNS`,
en `bench.py`.

**El reloj.** Hay clones armados sobre un LGT8F328P, que no lleva cristal: usa un
RC interno de 32 MHz y arranca dividido por 8, o sea a 4 MHz. Todo este proyecto
está calculado para 16 MHz --el puerto serie a 1 Mbaud, el muestreador de 5 kHz,
el PWM del puente--, así que a 4 MHz no anda nada, y el síntoma es el peor
posible: el puerto serie también emite cuatro veces lento, con lo que la placa no
puede ni avisar lo que le pasa y el monitor se llena de basura. Los sketches
corrigen el divisor al arrancar, y sólo si la placa arrancó dividida; ver
`libraries/BoardStart/`. `bringup()` lo verifica midiendo la frecuencia real de
las filas contra el reloj de la computadora.

**El ADC.** El del LGT8F328P es de 12 bits y el del ATmega328P de 10. La placa
cuenta en 12 bits en las dos --corre la lectura del UNO dos lugares--, así que
los miliamperes salen iguales; lo que cambia es que el UNO cuenta de a cuatro.

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

**4. Verificar el sensor**, antes de meter el motor en el medio. Se graba el
sketch de puesta en marcha y se abre el monitor serie a **115200**:

```
arduino-cli compile -b arduino:avr:uno --libraries ./libraries AS5600_Bringup
arduino-cli upload  -b arduino:avr:uno --libraries ./libraries -p <puerto> AS5600_Bringup
```

Vuelca la configuración del AS5600 y después el ángulo a 5 Hz. Lo que hay que
mirar es `STATUS=[MD -- --]`, que es imán detectado y AGC en rango, y que el
ángulo siga al imán al girarlo. `AS5600_Loop5k` es el paso siguiente y opcional:
muestrea a 5 kHz e informa la frecuencia efectiva, para confirmar que el bus
aguanta el ritmo. `Puente_Bringup` hace lo propio con el actuador, sin usar el
sensor.

**5. Abrir el notebook.**

```
./.venv/bin/jupyter lab notebooks/hardware.ipynb
```

Corre igual sin la placa: si no encuentra el banco cae en uno simulado y lo
dice, así que sirve para mostrarlo en clase con el cable desenchufado.

Elegir el kernel *Arduino Control (.venv)* y correr las celdas en orden. La
primera compila y graba `Banco` sola; la placa se encuentra sin nombrar ningún
puerto. `dev.bringup()` verifica el equipo subsistema por subsistema y es lo que
conviene correr ante cualquier duda.

`sync_board()` recompila si se editó el sketch, graba si cambió el binario y
reabre el enlace, lo que resetea la placa. Ese reset devuelve la placa a los
valores del sketch; lo que no es genérico --la calibración del sensor-- lo
repone la computadora; ver *Calibrar el sensor*.

---

## Qué se puede tocar desde el notebook

Los parámetros son atributos, siempre en unidades reales, y son pocos:

| Parámetro | Qué es |
|---|---|
| `uff` | el comando sobre el puente, de -255 a 255 |
| `tickdiv` | divisor del muestreador de 5 kHz: 10 → 500 Hz (por omisión), 5 → 1 kHz, 50 → 100 Hz |
| `izero` | cuenta del ADC que se lee como corriente cero; `zero_current()` lo mide |
| `cal` | 1 si se aplica la tabla de calibración del sensor; ver más abajo |
| `lutw`, `lutsum` | una entrada de la tabla de calibración, y la suma que verifica las 64 |
| `maxlate`, `missed`, `sovr`, `serr` | contadores de salud del muestreo |
| `spres`, `mstat` | estado del sensor: si contesta en el bus, y qué dice del imán |

Y las capturas son `DataFrame`s con `t`, `y_raw`, `y_uw`, `u` e `i`. Lo que se
hace con ellas está en `python/ensayo.py`, que son cien líneas y conviene leer
enteras: `velocidad()` deriva el ángulo y promedia, `signo()` dice para dónde va
el ángulo con un comando positivo, `normalizar()` pasa todo a segundos, por
ciento, radianes, radianes por segundo y amperes, y `guardar()` / `cargar()` lo
llevan a un archivo y lo traen de vuelta.

Un parámetro nuevo es una línea en la tabla `g_params[]` de
`Banco/Banco.ino`, y aparece solo en el notebook: del lado de Python no hay nada
que cambiar.

---

## Calibrar el sensor

El AS5600 no mide el ángulo que uno cree. Un imán descentrado --la hoja de datos
pide un cuarto de milímetro-- corre la lectura en una cantidad que depende del
ángulo y se repite vuelta tras vuelta, y al derivar se presenta como una
ondulación de velocidad que parece del motor.

`notebooks/calibracion.ipynb` la mide, decide cuánto de lo que midió es el sensor
y cuánto es el motor, y arma una tabla de 128 bytes que la corrige adentro del
Arduino. Corre igual sin la placa. El método está en
`Docs/CALIBRACION_AS5600.md`.

**La tabla no vive en la placa.** El dispositivo arranca siempre sin calibrar y la
dueña de la tabla es la computadora, que la empuja al conectarse:
`sync_board_cal()` es `sync_board()` más la calibración de este banco. Una
calibración es una propiedad de *este banco* --este imán, en este eje-- y no del
programa, y el caso feo no es la tabla que falta, es la tabla vieja de otro
montaje aplicándose en silencio. Para un tablero que se enciende solo,
`cal.escribir_header()` genera `Banco/Calibracion.h` y el sketch lo toma en la
próxima compilación.

Para una primera identificación se puede arrancar sin calibrar: la ondulación
está enganchada a la vuelta, y promediar la velocidad sobre una vuelta la tapa en
régimen. Lo que no tapa es el transitorio; ahí conviene saber que está.

---

## Cuando algo no anda

| Síntoma | Dónde mirar |
|---|---|
| `sync_board()` no encuentra `arduino-cli` | está en el `PATH`? En Windows hay que reabrir la terminal después de instalarlo: el `PATH` se lee una sola vez al arrancar |
| «no se pudo abrir el puerto» | algo más lo tiene tomado: el monitor serie del IDE, o un kernel de una sesión anterior. Un puerto serie es exclusivo |
| «no se encontro ningun puerto serie USB» | placa desenchufada, o cable de sólo alimentación |
| el monitor serie muestra basura, o `sync_board()` no encuentra el sketch que acaba de grabar | la placa no está corriendo a 16 MHz. Los sketches lo corrigen solos al arrancar; si el sketch grabado es de antes de eso, recompilar |
| `bringup` marca falla en `bus i2c` con **cero muestras** y un desborde por período | el bus quedó tomado por el sensor, que se quedó a medio hablar cuando la grabación reseteó la placa. Los sketches lo destraban al arrancar; si vuelve a pasar, cortar y dar alimentación |
| `bringup` marca falla en `sensor` | el AS5600 no contesta en el bus: SDA (A4), SCL (A5), alimentación, pull-ups |
| `bringup` marca falla en `iman` | el sensor contesta pero el imán está ausente, muy lejos o muy cerca; el mensaje dice cuál |
| `bringup` marca falla en `bus i2c` | errores intermitentes con el sensor presente: cableado o pull-ups |
| `bringup` marca falla en `cero de i` | el sensor de corriente no reposa lejos de los rieles: sin alimentar, mal cableado, o no es un ACS712 de 5 V |
| `bringup` marca falla en `motor` y el eje no gira | grabar `Puente_Bringup`: la placa lee sus propios pines de vuelta y separa «no sale el comando» de «el puente no lo sigue». La causa más común es la alimentación de potencia del puente |
| `bringup` anota `signo` | un comando positivo hace bajar el ángulo. No es una falla: `ensayo.normalizar()` lo da vuelta, o se dan vuelta los dos cables del motor |
| «el dispositivo declara sus parametros en un formato anterior» | la placa tiene grabado un sketch viejo: `sync_board(force_upload=True)` |
| se interrumpió una celda en medio de una captura | nada: la operación siguiente resincroniza el enlace sola. `dev.resync()` lo fuerza a mano |
| se pierden períodos | subir `tickdiv` |
| se descartan filas de telemetría | subir `dec`, o emitir menos canales |

Toda captura verifica su propia salud y avisa por `stderr` si se perdieron
períodos o filas: una serie temporal a la que le faltan muestras se ve igual que
una sana hasta que uno va a fijarse.

**Periféricos que se apropia `Banco`:** el Timer2, así que `analogWrite()` en
los pines 3 y 11 y `tone()` dejan de funcionar; el Timer1, que modula el puente
con su propio TOP, así que `analogWrite()` en los pines 9 y 10 y `Servo` dejan
de servir; y el ADC, que se maneja directamente, así que no hay que llamar a
`analogRead()`. El Timer0 queda intacto: `millis()` y el PWM de los pines 5 y 6
andan como siempre.

**El PWM del puente va a 1 kHz**, y es `PWM_TOP` en el sketch. En abstracto
conviene modular más rápido: fuera del rango audible, y con la ondulación de
corriente lejos de la banda que interesa. Pero un L298N alimentado con 5 V no lo
tolera: es un puente de Darlington bipolares, cae unos 2 V entre sus dos lados y
tarda unos 2 µs en conmutar, y a 20 kHz lo que se pierde en cada transición se
lleva una fracción grande de un tiempo de encendido que ya venía escaso. Medido
en este banco: **a 20 kHz el motor no arranca y a 1 kHz anda**. Con un puente
MOSFET --un TB6612FNG, un DRV8833, que caen 0,3 V en lugar de 2-- lo correcto
sería subirla: 400 son 20 kHz.

---

## Organización

```
Banco/                   el banco: PWM afuera, ángulo y corriente adentro, telemetría
AS5600_Bringup/          verificación del sensor, con volcado de configuración
AS5600_Loop5k/           prueba de muestreo a 5 kHz
Puente_Bringup/          verificación del accionamiento, sin usar el sensor
libraries/CtrlLink/      el protocolo, lado placa
libraries/AS5600Async/   lectura asincrónica del AS5600
libraries/nI2C/          bus I2C por interrupciones (submódulo, de terceros)
libraries/BoardStart/    el reloj, el ADC y el destrabe del bus, antes de todo lo demás
python/ctrllink.py       el protocolo, lado computadora
python/bench.py          compilación, conexión y verificación de este equipo
python/ensayo.py         la velocidad, las unidades y el archivo de cada ensayo
python/calib.py          calibración del AS5600: medición, decisión y tabla
python/banco_simulado.py un banco de mentira, para dar la clase sin la placa
python/fakeuno.py        simulación del dispositivo, fiel byte a byte
python/test_*.py         pruebas, no necesitan hardware
notebooks/               los notebooks: hardware y calibración
PROTOCOL.md              el protocolo: diseño, formato de línea y mediciones
Docs/CALIBRACION_AS5600.md  por qué la calibración es como es
```

Compilar y grabar a mano, si hiciera falta:

```
arduino-cli compile -b arduino:avr:uno --libraries ./libraries \
  --build-property compiler.c.extra_flags=-O2 \
  --build-property compiler.cpp.extra_flags=-O2 \
  --build-property compiler.c.elf.extra_flags=-O2 \
  Banco
arduino-cli upload  -b arduino:avr:uno --libraries ./libraries -p <puerto> Banco
```

Las tres `--build-property` son las que compilan con optimización plena. El core
de AVR trae `-Os` --optimizar por tamaño--, y este sketch quiere ciclos y no
bytes; `sync_board()` las pasa solas, así que sólo hacen falta al compilar a
mano.
