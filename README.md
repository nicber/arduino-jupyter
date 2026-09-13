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
ensayo.esperar_quieto(dev)                                  # el eje, parado de verdad
dev.ctl_uff = 102                                           # 40 % sobre el actuador
df = dev.step('ctl_uff', 204, pre=3.0, post=4.0, back=0)    # escalón, t = 0 en el escalón
t, w = ensayo.velocidad(df)                                 # rad/s, derivada de este lado
ensayo.guardar(df, 'datos/escalon_40_80.csv')               # t, u, theta, omega, i
```

Lo que recorre `notebooks/hardware.ipynb`:

- cómo está armado el banco, y verificar el equipo parte por parte;
- mirar la **corriente**, y decidir si el canal la resuelve;
- **la API para el TP2**, con un ejemplo que se puede correr para cada cosa:
  aplicar un comando, capturar, un **escalón** desde un régimen, una **serie de
  escalones** o una escalera, la **velocidad** a partir del ángulo, y **guardar
  cada ensayo** en un archivo con las unidades de un modelo.

La identificación en sí --el modelo, el ajuste, la validación-- es el trabajo del
TP y no está en el notebook.

En la placa no hay ley de control ni filtros: el comando va derecho al actuador y
el ángulo vuelve tal como lo entregó el sensor. Todo lo que se hace con la
medición --elegir el signo, derivar, filtrar, ajustar-- pasa del lado de la
computadora, donde se ve y se puede cambiar. Un filtro en la placa se confunde
con la planta que se está midiendo, y por eso no hay ninguno. Los detalles del
enlace están en [`PROTOCOL.md`](PROTOCOL.md).

---

## Qué hace falta

### Hardware

El recorrido completo del montaje --las configuraciones por las que crece el
banco, los diagramas de cableado y qué se puede medir con cada una-- está en
[`notebooks/hardware.ipynb`](notebooks/hardware.ipynb), que es además el que se
muestra en clase.

| Señal | Pin del UNO | |
|---|---|---|
| AS5600 SDA | A4 | |
| AS5600 SCL | A5 | |
| AS5600 VDD / GND | 5V / GND | |
| Salida del ACS712 | A0 | opcional |
| `ENA` del puente, o la puerta del transistor (PWM, 1 kHz) | 9 | |
| L298N `IN1` | 6 | sólo con un puente |
| L298N `IN2` | 7 | sólo con un puente |

**No todos los bancos tienen el mismo actuador.** Algunos tienen un **puente en H**
(L298N, configuración **B** del notebook), que acciona en los dos sentidos con un
comando de -255 a 255. Otros tienen un **transistor a masa con su diodo de rueda
libre** (configuración **B′**), gobernado sólo desde el pin 9. La celda de la
sección 2.2 de `hardware.ipynb` dice cuál tiene cada banco. El sketch y el notebook
son los mismos para los dos.

Con el transistor, el actuador es de un solo cuadrante, y eso se nota en todo lo
que se mide:

- **empuja y no frena**: con el comando en cero, o más bajo, el eje sólo lo frena
  el rozamiento, y bajar de velocidad tarda casi el doble que subir;
- **un comando negativo empuja para el mismo lado**: el sentido está en los
  cables, no en el comando;
- **con el comando en cero el eje sigue girando muchos segundos**, así que todo
  ensayo empieza con `ensayo.esperar_quieto(dev)`;
- **la corriente se extingue antes de terminar cada período de PWM** a
  velocidades medias, y eso dobla la curva estática: la ganancia cae varias veces
  entre un comando bajo y uno alto.

Con el puente en H, `ENA` lleva la magnitud e `IN1`/`IN2` el sentido, y un comando
negativo hace girar el motor para el otro lado. Con el comando en cero el puente
también queda abierto, así que el eje tampoco frena solo. Si un comando positivo hace bajar el
ángulo medido, eso es de qué lado están los cables del motor y de qué lado mira el
imán: `ensayo.signo()` lo detecta y `ensayo.normalizar()` lo aplica.

**La medición de corriente lee A0 contra Vcc.** Un ACS712 es bipolar y
ratiométrico: reposa en la mitad de su alimentación para poder bajar cuando la
corriente cambia de sentido, así que contra la referencia interna de 1,1 V
satura en reposo. Contra Vcc reposa en media escala por construcción. Lo que se
paga es resolución: con 185 mV/A son 6,6 mA por cuenta en el clon y 26 en el UNO,
que cuenta de a cuatro. **Con este motor el canal no alcanza**: consume decenas
de mA y el sensor tiene 91 mA RMS de ruido por muestra. Sirve para ver que hay
corriente, no para medirla; `bringup()` lo dice cuando lo ve.

Dos números que conviene verificar una vez por banco, los dos en el sketch:

- `SENSE_MV_PER_A`. La sensibilidad del sensor, que es lo único que convierte
  cuentas en amperes. `bringup()` calibra el cero, que tiene una condición
  conocida --el actuador abierto--, pero para la ganancia haría falta una
  corriente conocida. Un tester en serie con el motor, una vez, alcanza.
- `ADC_REF_MV`. Vcc, que se mide una vez con un tester.

El AS5600 necesita un imán **magnetizado diametralmente** girando sobre el chip,
a un par de milímetros. Las plaquetas de AS5600 traen su propio regulador y los
pull-ups del bus; un chip pelado en modo 3,3 V necesita adaptación de niveles.

**Se puede empezar sin motor y sin medición de corriente.** Sin nada conectado a
los pines 9, 6 y 7 el muestreo corre igual y la telemetría se comporta de manera
idéntica: alcanza con girar el imán a mano para ver al sensor trabajar;
`bringup(motor=False)` saltea la parte que lo haría girar. Sin ACS712,
`bringup()` detecta que la entrada quedó contra el riel del ADC y lo dice.

Incluso sin el AS5600 el muestreo mantiene su período: al no obtener respuesta,
pasa a sondear el bus dos veces por segundo en lugar de cinco mil, y `bringup()`
informa `sensor: no contesta`.

> ⚠️ **Con el motor conectado, el motor se mueve.** Conviene revisar que el eje
> esté libre antes de correr cualquier celda.

### Software

- El [Arduino IDE 2](https://www.arduino.cc/en/software), con el soporte para
  placas AVR. El notebook compila y graba solo con el compilador que trae el IDE,
  así que el IDE hace falta instalado pero no abierto.
- El entorno de conda **`dyc`** del curso, más `git`.

Cómo instalar las dos cosas está en [*Puesta en marcha*](#puesta-en-marcha).

### Clones del UNO

Sirve cualquier placa con un ATmega328P a 16 MHz, y también los clones que no
son exactamente eso. Nada de lo que sigue hay que configurar: el notebook se da
cuenta solo. Está acá porque cuando algo falla conviene saber qué se estaba
compensando.

**El bootloader.** Un UNO escucha a 115200 y muchos clones baratos traen el
bootloader viejo del Nano, que escucha a 57600. Elegir mal no da un error legible
sino diez líneas de «not in sync». `sync_board()` prueba los dos, se queda con el
que anduvo y lo recuerda por puerto. Un tipo de placa que no esté en la lista se
agrega en `UPLOAD_FQBNS`, en `bench.py`.

**El reloj.** Hay clones armados sobre un LGT8F328P, que no lleva cristal: usa un
RC interno de 32 MHz y arranca dividido por 8, o sea a 4 MHz. Todo este proyecto
está calculado para 16 MHz, así que a 4 MHz no anda nada, y el síntoma es el peor
posible: el puerto serie también emite cuatro veces lento y el monitor se llena
de basura. Los sketches corrigen el divisor al arrancar, y sólo si la placa
arrancó dividida; ver `libraries/BoardStart/`. `bringup()` lo verifica midiendo la
frecuencia real de las filas contra el reloj de la computadora.

**El ADC.** El del LGT8F328P es de 12 bits y el del ATmega328P de 10. La placa
cuenta en 12 bits en las dos --corre la lectura del UNO dos lugares--, así que
los miliamperes salen iguales; lo que cambia es que el UNO cuenta de a cuatro. Y
en el clon los bits `REFS` no eligen la referencia: el sketch escribe los
registros que sí lo hacen, para que Vcc sea Vcc en las dos.

---

## Puesta en marcha

Esta guía está pensada para quien nunca usó una terminal, git ni un Arduino. Se
probó en Windows 11 con Miniforge, el Arduino IDE 2.3 y un clon del UNO con
LGT8F328P. Supone que Miniforge (o Miniconda, o Anaconda: los comandos son los
mismos) ya está instalado.

Se hace una sola vez. Lleva unos veinte minutos, casi todos de descarga.

### 1. Instalar el Arduino IDE

1. Descargarlo de <https://www.arduino.cc/en/software> (*Windows, Win 10 and
   newer, 64 bits*) e instalarlo con las opciones que vienen marcadas.
2. Abrirlo una vez. La primera vez descarga sus herramientas y suele ofrecer
   instalar **Arduino AVR Boards** y algunos drivers: aceptar todo.
3. Verificar que quedó el soporte para el UNO: menú **Herramientas → Placa →
   Gestor de placas**, buscar `Arduino AVR Boards` y, si dice *Instalar*,
   instalarlo.
4. Enchufar la placa por USB y mirar **Herramientas → Puerto**: tiene que aparecer
   un `COM3`, `COM4` o parecido. Si no aparece nada, ver *Problemas frecuentes*
   más abajo.
5. **Cerrar el IDE.** El notebook usa el mismo puerto, y un puerto serie sólo lo
   puede tener abierto un programa a la vez.

No hace falta tocar el `PATH` ni instalar nada más de Arduino.

### 2. Crear el entorno `dyc`

Todos los comandos que siguen se escriben en el **Miniforge Prompt** (o
*Anaconda Prompt*), que se encuentra escribiendo `miniforge` en el menú Inicio.
Cada línea se escribe y se confirma con Enter.

El entorno se crea en `C:\envs\dyc` y no en la carpeta del usuario, a propósito:
si el nombre de usuario de Windows tiene espacios o acentos --`C:\Users\Juan
Pérez`--, algunas herramientas fallan con rutas así.

La lista de paquetes del entorno está en [`dyc.yml`](dyc.yml), en este
repositorio. La primera línea la descarga a *Descargas*, y la segunda crea el
entorno a partir de ella:

```bash
curl -L -o "%USERPROFILE%\Downloads\dyc.yml" https://raw.githubusercontent.com/nicber/arduino-jupyter/main/dyc.yml
conda env create -f "%USERPROFILE%\Downloads\dyc.yml" -p C:\envs\dyc
```

Las comillas van: son las que hacen que funcione aunque el nombre de usuario
tenga espacios. Tarda varios minutos. Si el entorno ya se había creado antes,
este paso se saltea.

### 3. Activar el entorno e instalar `git`

```bash
conda activate C:\envs\dyc
conda install -c conda-forge git
```

Cuando pregunte `Proceed ([y]/n)?`, escribir `y` y Enter. `git` es lo único que
le falta a `dyc` para este proyecto, y sirve para descargarlo en el paso
siguiente.

Después de `conda activate`, la línea empieza con `(C:\envs\dyc)`. **Si no
empieza así, el entorno no está activado**, y lo que se instale o se corra va a
parar a otro Python. `conda activate C:\envs\dyc` hay que repetirlo cada vez que
se abre el Miniforge Prompt.

### 4. Descargar el proyecto

Esto lo baja a `C:\envs`, al lado del entorno, también fuera de la carpeta del
usuario:

```bash
cd C:\envs
git clone --recurse-submodules https://github.com/nicber/arduino-jupyter.git
cd arduino-jupyter
```

**Si la cátedra entregó `arduino-jupyter-tp2.zip`**, en lugar de lo de arriba alcanza
con descomprimirlo en `C:\envs`, de modo que quede `C:\envs\arduino-jupyter`, y
después `cd C:\envs\arduino-jupyter`. El zip trae todo lo necesario para el TP2.

No sirve el botón *Download ZIP* de GitHub: ese ZIP no trae la biblioteca `nI2C`,
y sin ella no compila nada. Si el proyecto ya se había descargado sin
`--recurse-submodules`, se completa entrando a la carpeta y corriendo
`git submodule update --init`.

### 5. Probar la instalación, sin la placa

```bash
python herramientas\verificar.py
```

Verifica que estén todos los paquetes, que el programa de la placa compile (no
graba nada), que pasen las pruebas de Python y que los notebooks corran contra el
banco simulado. La primera vez tarda un par de minutos. Tiene que terminar con
`las ... verificaciones pasaron`; si algo falla, dice qué. Si dice que faltan
paquetes, el entorno no está activado: volver al paso 3.

### 6. Abrir el notebook

Cada vez que se quiera trabajar, en un Miniforge Prompt nuevo:

```bash
conda activate C:\envs\dyc
cd C:\envs\arduino-jupyter
jupyter lab notebooks\hardware.ipynb
```

Se abre JupyterLab en el navegador. **La ventana negra del Miniforge Prompt tiene
que quedar abierta** mientras se usa: cerrarla cierra Jupyter.

### 7. Correr el notebook con la placa

Enchufar la placa y correr las celdas **en orden**, de arriba hacia abajo
(Shift+Enter corre una celda y pasa a la siguiente). La primera celda:

- **verifica el entorno**: si falta algún paquete, avisa cuál y casi siempre
  quiere decir que Jupyter se abrió sin activar `dyc` (volver al paso 6);
- **compila** el programa de la placa, lo que la primera vez tarda uno o dos
  minutos;
- lo **graba** en la placa, sin que haga falta elegir el puerto;
- y se conecta. Si todo anduvo, dice algo como
  `COM3: CtrlLink 1 Banco ...  (compilado, cargado como clon con bootloader viejo)`.

Con un clon, que la primera grabación tarde medio minuto más es normal: prueba
primero como UNO original y después como clon, y a partir de ahí recuerda cuál
anduvo. El aviso `sin calibracion del sensor` también es normal.

Sin la placa enchufada el notebook también corre, sobre un banco simulado, y lo
dice en la primera celda.

`dev.bringup()` verifica el equipo parte por parte, y es lo que conviene correr
ante cualquier duda. Cada vez que se conecta, `sync_board()` recompila si se editó
el sketch, graba si cambió el binario y reabre el enlace, lo que resetea la placa
a los valores del sketch; la calibración del sensor la repone la computadora (ver
*Calibrar el sensor*).

> ⚠️ **Varias celdas hacen girar el motor.** Antes de correrlas, revisar que el
> eje esté libre y que no haya nada cerca.

### Problemas frecuentes

| Qué pasa | Qué hacer |
|---|---|
| La primera celda dice `Faltan paquetes` | Jupyter se abrió sin activar el entorno. Cerrar JupyterLab y el Miniforge Prompt, y repetir el paso 6. El mensaje dice con qué Python está corriendo: tiene que ser `C:\envs\dyc\python.exe` |
| `conda activate` dice que el entorno no existe | Falta el paso 2, o se creó en otro lugar. `conda env list` muestra dónde están los entornos |
| La placa no aparece en **Herramientas → Puerto** del IDE | Probar otro cable: muchos cables USB sólo dan alimentación y no llevan datos. Si la placa tiene un chip CH340 (dice *CH340* cerca del USB), instalar su driver desde <https://www.wch-ic.com/downloads/CH341SER_EXE.html> y reenchufar |
| «no se pudo abrir el puerto» | Otro programa lo tiene abierto: el Monitor Serie del IDE, el IDE mismo, u otro notebook. Cerrarlos, o reiniciar el kernel (menú **Kernel → Restart Kernel**) |
| «Falta el soporte para placas AVR» | Hacer el paso 1.3: instalar *Arduino AVR Boards* desde el Gestor de placas del IDE |
| «no se encontro arduino-cli» | El IDE no está instalado, o se instaló en una carpeta poco común. Reinstalarlo con las opciones por omisión |
| «se encontraron varios puertos serie USB» | Hay más de una placa, o algún otro aparato USB-serie, enchufado. Desenchufar lo que sobra |
| Un gráfico tira un error sobre DLL, o el kernel se muere sin avisar | El Python de `dyc` se está usando sin activar el entorno. Repetir el paso 6 |

---

## Qué se puede tocar desde el notebook

Los parámetros son atributos, siempre en unidades reales, y son pocos:

| Parámetro | Qué es |
|---|---|
| `ctl_uff` | el comando sobre el actuador, de -255 a 255 |
| `loop_div` | divisor del muestreador de 5 kHz: 10 → 500 Hz (por omisión), 5 → 1 kHz, 50 → 100 Hz |
| `cur_zero` | cuenta del ADC que se lee como corriente cero; `zero_current()` la mide |
| `ang_cal` | 1 si se aplica la tabla de calibración del sensor; ver más abajo |
| `ang_lutw`, `ang_lutsum` | una entrada de la tabla de calibración, y la suma que verifica las 64 |
| `loop_late`, `loop_missed`, `ang_busovr`, `ang_buserr` | contadores de salud |
| `ang_present`, `ang_status`, `ang_agc`, `ang_mag` | estado del sensor: si contesta en el bus, y qué dice del imán |

Las capturas son `DataFrame`s con `t`, `y_raw`, `y_uw`, `u` e `i`. Lo que se hace
con ellas está en `python/ensayo.py`, que conviene leer entero:
`esperar_quieto()` no deja arrancar un ensayo con el eje girando, `velocidad()`
deriva el ángulo y promedia con una ventana centrada, `signo()` dice para dónde va
el ángulo con un comando positivo, `normalizar()` pasa todo a segundos, por
ciento, radianes, radianes por segundo y amperes, y `guardar()` / `cargar()` lo
llevan a un archivo y lo traen de vuelta.

No hay ninguna lista de parámetros escrita del lado de Python: la placa declara su
tabla al conectarse. Poner `dev` en una celda la muestra entera, con el valor de
ahora, la unidad y una línea de qué es cada uno; `dev.describe('ang')` filtra. Las
explicaciones viven en `python/catalogo.py`, y `test_catalogo.py` falla si sobra o
falta una entrada.

Un parámetro nuevo es una línea en la tabla `g_params[]` de `Banco/Banco.ino`, y
aparece solo en el notebook. `python/test_tablas.py` compara esa tabla contra un
golden guardado, así que un renombre o un reordenamiento se ve en la revisión en
lugar de descubrirse desde un notebook.

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
montaje aplicándose en silencio.

El filtro lento del AS5600 va en 2x, 0,286 ms de retardo en lugar de los 2,2 ms
de fábrica, y el sketch lo escribe al arrancar. No es una perilla: ese retardo se
identificaría después como un tiempo muerto del motor.

---

## Cuando algo no anda

| Síntoma | Dónde mirar |
|---|---|
| `sync_board()` no encuentra `arduino-cli` | busca primero en el `PATH` y después adentro del Arduino IDE 2, instalado en su lugar por omisión. Reinstalar el IDE con las opciones por omisión |
| la primera celda dice `Faltan paquetes` | el kernel no es el entorno `dyc`; el mensaje dice cuál es. Ver el paso 6 de *Puesta en marcha* |
| «no se pudo abrir el puerto» | algo más lo tiene tomado: el monitor serie del IDE, o un kernel de una sesión anterior. Un puerto serie es exclusivo |
| «no se encontro ningun puerto serie USB» | placa desenchufada, o cable de sólo alimentación |
| el monitor serie muestra basura, o `sync_board()` no encuentra el sketch que acaba de grabar | la placa no está corriendo a 16 MHz. Los sketches lo corrigen solos al arrancar; si el sketch grabado es de antes de eso, recompilar |
| `bringup` marca falla en `bus i2c` con **cero muestras** y un desborde por período | el bus quedó tomado por el sensor, que se quedó a medio hablar cuando la grabación reseteó la placa. Los sketches lo destraban al arrancar; si vuelve a pasar, cortar y dar alimentación |
| `bringup` marca falla en `sensor` | el AS5600 no contesta en el bus: SDA (A4), SCL (A5), alimentación, pull-ups |
| `bringup` marca falla en `iman` | el sensor contesta pero el imán está ausente, muy lejos o muy cerca; el mensaje dice cuál |
| `bringup` marca falla en `bus i2c` | errores intermitentes con el sensor presente: cableado o pull-ups |
| `bringup` marca falla en `cero de i` | el sensor de corriente no reposa lejos de los rieles: sin alimentar, mal cableado, o no es un ACS712 de 5 V |
| `bringup` marca falla en `motor` y el eje no gira | grabar `Puente_Bringup`: la placa lee sus propios pines de vuelta y separa «no sale el comando» de «el actuador no lo sigue». La causa más común es la alimentación de potencia |
| `bringup` anota `signo` | un comando positivo hace bajar el ángulo. No es una falla: `ensayo.normalizar()` lo da vuelta |
| `bringup` anota `canal de i` | el pico de corriente del arranque no se despega del ruido: el sensor no resuelve este motor |
| un ensayo sale distinto cada vez que se corre | ¿esperó a que el eje pare? Con el comando en cero el motor sigue girando muchos segundos: `ensayo.esperar_quieto(dev)` |
| «el dispositivo declara sus parametros en un formato anterior» | la placa tiene grabado un sketch viejo: `sync_board(force_upload=True)` |
| se interrumpió una celda en medio de una captura | nada: la operación siguiente resincroniza el enlace sola. `dev.resync()` lo fuerza a mano |
| se pierden períodos | emitir sólo los canales que se van a mirar, con `dev.capture(..., canales=['y_uw', 'u'])`: formatear y enviar la fila entera le cuesta a la placa más que el paso. Si no alcanza, subir `loop_div` |
| se descartan filas de telemetría | subir `dec`, o emitir menos canales con `canales=` |

Toda captura verifica su propia salud y avisa por `stderr` si se perdieron
períodos o filas: una serie temporal a la que le faltan muestras se ve igual que
una sana hasta que uno va a fijarse.

**Periféricos que se apropia `Banco`:** el Timer2, así que `analogWrite()` en
los pines 3 y 11 y `tone()` dejan de funcionar; el Timer1, que modula el actuador
con su propio TOP, así que `analogWrite()` en los pines 9 y 10 y `Servo` dejan
de servir; y el ADC, que se maneja directamente, así que no hay que llamar a
`analogRead()`. El Timer0 queda intacto: `millis()` y el PWM de los pines 5 y 6
andan como siempre.

**El PWM va a 1 kHz**, y es `PWM_TOP` en el sketch. En abstracto conviene modular
más rápido, fuera del rango audible. Pero un L298N alimentado con 5 V no lo
tolera: es un puente de Darlington bipolares, cae unos 2 V y tarda unos 2 µs en
conmutar, y a 20 kHz lo que se pierde en cada transición se lleva una fracción
grande de un tiempo de encendido que ya venía escaso. Medido: **a 20 kHz el motor
no arranca y a 1 kHz anda**. Con un transistor MOSFET o un puente MOSFET lo
correcto sería subirla: 400 son 20 kHz.

**Si el banco vuelve a tener un L298N**, conviene accionarlo frenando en lugar de
soltando: `ENA` en alto y el PWM sobre la entrada del sentido, con la otra en cero,
de modo que en la parte baja del ciclo el puente cortocircuita el motor en vez de
abrirlo. Eso deja la planta lineal con un solo comando. Lo que no conviene es
modular bipolar a 1 kHz.

---

## Organización

```
Banco/                   el banco: PWM afuera, ángulo y corriente adentro, telemetría
AS5600_Bringup/          verificación del sensor, con volcado de configuración
AS5600_Loop5k/           prueba de muestreo a 5 kHz
Puente_Bringup/          verificación del accionamiento, sin usar el sensor
libraries/CtrlLink/      el protocolo, lado placa
libraries/Actuator/      el puente en H, o el transistor desde ENA
libraries/Sampler/       el reloj del muestreo: período rígido y divisor
libraries/Sense/         el conversor libre, y una corriente alrededor de su cero
libraries/AngleSensor/   ángulo desenrollado, y salud del sensor
libraries/Calibracion/   la corrección del error de ángulo
libraries/AS5600Async/   lectura asincrónica del AS5600
libraries/AS5600Regs/    el mapa de registros, sin ningún transporte
libraries/nI2C/          bus I2C por interrupciones (submódulo, de terceros)
libraries/BoardStart/    el reloj, el ADC y el destrabe del bus, antes de todo lo demás
test/test_modulos.cpp    los módulos que son aritmética pura, en la de escritorio
python/ctrllink.py       el protocolo, lado computadora
python/bench.py          compilación, conexión y verificación de este equipo
python/ensayo.py         esperar al eje, la velocidad, las unidades y el archivo
python/catalogo.py       qué significa cada parámetro, y cómo mostrarlo
python/calib.py          calibración del AS5600: medición, decisión y tabla
python/entorno.py        que el notebook corra en el entorno dyc, y qué hacer si no
python/banco_simulado.py un banco de mentira que sigue al de verdad, para dar la clase sin la placa
python/fakeuno.py        simulación del dispositivo, fiel byte a byte
python/test_*.py         pruebas, no necesitan hardware ni compilar
                         (test_notebooks.py además corre los notebooks simulados)
python/test_hardware.py  la única que sí necesita la placa: --motor mueve el eje
notebooks/               los notebooks: hardware y calibración
herramientas/verificar.py        que la instalación ande sin la placa: entorno, compilación, pruebas, notebooks
herramientas/empaquetar_tp2.py   arma dist/arduino-jupyter-tp2.zip, lo necesario para el TP2
dyc.yml                  el entorno de conda del curso
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

Lo que **no** se pasa es `-flto`, y no por olvido: se probó y no cambia nada.
Todas las librerías de este proyecto son sólo de cabecera, así que el sketch
entero ya es una sola unidad de traducción y no hay ninguna frontera que LTO pueda
disolver.
