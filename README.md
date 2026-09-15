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
dev = sync_board(bidir=False)                               # False: transistor; True: puente en H
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

En la placa no hay ley de control, y el ángulo no tiene ningún filtro: el comando
va derecho al actuador y el ángulo vuelve como lo entregó el sensor, con el signo
del banco. Todo lo que se hace con la medición --derivar, filtrar, ajustar-- pasa
del lado de la computadora, donde se ve y se puede cambiar. Un filtro en la placa se
confunde con la planta que se está midiendo. La única excepción es la corriente, que
se promedia en la placa con un retardo fijo y declarado. Lo único que la placa sabe
del cableado es lo que se le dice al conectar: si el actuador acciona en los dos
sentidos (`bidir`), los signos del imán y del sensor de corriente, que mide
`bringup()`, y el divisor de A1, si hay. El enlace está resumido en
[*El enlace*](#el-enlace).

**Dónde está explicado cada cosa.** Por qué el hardware es como es --el actuador, el
PWM de 1050 Hz, cómo se mide la corriente y qué se le puede creer-- está en
[`notebooks/hardware.ipynb`](notebooks/hardware.ipynb), y sólo ahí. Este README dice
cómo instalar, qué perillas hay y qué hacer cuando algo falla; los comentarios del
código explican las decisiones de implementación.

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
| Salida del ACS712, con el sensor del lado de +5 V del motor | A0 | opcional |
| Divisor de 5V a A1: 5,1 kΩ de 5V a A1, 2 kΩ de A1 a GND, y 100 nF de A1 a GND | A1 | con el ACS712, si la placa funciona a 3,3 V |
| `ENA` del puente, o la base del transistor por 220 Ω (PWM, 1050 Hz) | 9 | |
| L298N `IN1` | 6 | sólo con un puente |
| L298N `IN2` | 7 | sólo con un puente |

**No todos los bancos tienen el mismo actuador.** Algunos tienen un **puente en H**
(L298N, configuración **B**) y otros un **transistor a masa con su diodo de rueda
libre** (configuración **B′**, en este banco un BD139). Qué actuador hay lo declara
quien conecta: `sync_board(bidir=True)` con un puente, `bidir=False` con un
transistor; `bringup()` verifica que sea cierto y mide los signos del imán y del
sensor de corriente, que guarda en `notebooks/cableado.json`. Qué cambia con un solo
cuadrante --empuja y no frena, un comando negativo sale como cero, el eje sigue
girando con el comando en cero--: `hardware.ipynb`, secciones 2.1 y 2.2.

**La corriente**: un ACS712 en A0, en serie del lado de +5 V del motor, promediado en
la placa sobre `cur_filas` filas (20 ms por omisión, con ~10 ms de retardo). En una
placa a 3,3 V, como el clon del banco, A1 lee los 5 V del sensor por un divisor y la
placa usa el cociente A0/A1; el divisor se declara una vez por banco con
`dev.declarar_divisor(5100, 2000)`. **Un divisor suelto no da error**: corre la
corriente cientos de mA, y `sync_board()` avisa si A1 lee fuera de lo que puede dar.
Con una ventana corta, `cur_notch = 3` saca la red sin calibrarla. La escala en mA no
está verificada con un tester y depende de `SENSE_MV_PER_A`, en el sketch, y de la
relación del divisor. El montaje, los números medidos y qué se le puede creer al
canal: `hardware.ipynb`, sección 4.

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
agrega en `UPLOAD_FQBNS`, en `placa.py`.

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
a los valores del sketch. Después le carga `bidir`, los signos y el divisor de
`cableado.json` y el cero de la corriente, que mide ahí mismo; la calibración del sensor la repone la
computadora (ver *Calibrar el sensor*).

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
| `ctl_uff` | el comando sobre el actuador, de -255 a 255 (de 0 a 255 con `mot_bidir` en 0) |
| `mot_bidir` | 1 con puente en H, 0 con un solo cuadrante; lo fija `sync_board(bidir=...)` |
| `ang_inv`, `cur_inv` | los signos del banco; los mide `bringup()` y los carga `sync_board()` |
| `cur_filas` | filas sobre las que se promedia la corriente: 10 → 20 ms (por omisión), 1 → sólo la fila |
| `loop_div` | divisor del muestreador de 5 kHz: 10 → 500 Hz (por omisión), 5 → 1 kHz, 50 → 100 Hz. El ángulo se desenrolla a 5 kHz, así que cualquier valor sirve hasta ~15 000 rad/s |
| `cur_zero` | cuenta del ADC que se lee como corriente cero; `zero_current()` la mide |
| `cur_div`, `cur_a1` | la relación del divisor de A1 en diezmilésimas (0 = sin divisor, contra AVCC), que fija `declarar_divisor()`, y lo que lee A1 |
| `cur_notch`, `cur_notchr` | notch de la red para la corriente: cuántos armónicos (0 apagado, 1 = 50 Hz, 3 = 50, 100 y 150), con dos notch en 49,5 y 50,5 Hz cada uno, y el radio del polo |
| `ang_cal` | 1 si se aplica la tabla de calibración del sensor; ver *Calibrar el sensor* |
| `ang_lutw`, `ang_lutsum` | una entrada de la tabla de calibración, y la suma que verifica las 64 |
| `loop_late`, `loop_missed`, `ang_busovr`, `ang_buserr` | contadores de salud |
| `ang_present`, `ang_status`, `ang_agc`, `ang_mag` | estado del sensor: si contesta en el bus, y qué dice del imán |

Las capturas son `DataFrame`s con `t`, `y_raw`, `y_uw`, `y_rep`, `u` e `i`. `y_rep`
vale 1 en una fila cuyo ángulo repite el anterior porque el bus I2C no llegó a
traer la muestra. Lo que se hace con ellas está en `python/ensayo.py`, que conviene
leer entero: `esperar_quieto()` no deja arrancar un ensayo con el eje girando,
`velocidad()` deriva el ángulo y promedia con una ventana centrada,
`normalizar()` pasa todo a segundos, por
ciento, radianes, radianes por segundo y amperes, y `guardar()` / `cargar()` lo
llevan a un archivo y lo traen de vuelta. Cada captura registra en
`df.attrs['config']` la placa y las perillas que cambian lo que se mide
(`loop_div`, `cur_filas`, `cur_notch`, los signos, el cero y el divisor de la
corriente), y `guardar()` lo escribe arriba del CSV en líneas que empiezan con `#`.
Para una herramienta que no acepta esas líneas (PID Tuner, APMonitor), se exporta
sin ellas: `datos[ensayo.COLUMNAS].to_csv(ruta, index=False)`.

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

Es un **extra**: el TP2 funciona sin él. El AS5600 no mide exactamente el ángulo
--un imán descentrado corre la lectura en una cantidad que se repite vuelta tras
vuelta, y al derivar aparece como una ondulación de velocidad--, y
[`extras/calibracion_as5600/`](extras/calibracion_as5600/) tiene el notebook que la
mide y arma una tabla que la corrige adentro del Arduino, el módulo `calib.py` y el
método completo en `CALIBRACION_AS5600.md`.

La tabla no vive en la placa: la placa arranca siempre sin calibrar, y
`calib.sync_board_cal()` es `sync_board()` más la calibración de este banco. Si la
calibración existe, la primera celda de `hardware.ipynb` la aplica sola.

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
| `bringup` marca falla en `motor` y el eje no gira | la causa más común es la alimentación de potencia: la fuente del motor prendida, la masa común con el Arduino, y el pin 9 llegando a `ENA` o a la base del transistor |
| `bringup` marca falla en `actuador` | lo declarado en `bidir` no es lo que hay: `-u` invierte el giro con `bidir=False`, o empuja igual con `bidir=True`. El mensaje dice con qué conectar |
| `bringup` marca falla en `-u` | con `bidir=True` el eje no gira al revés, o con `bidir=False` el comando negativo no sale como cero: un sketch viejo en la placa, `sync_board(force_upload=True)` |
| `sync_board` dice «sin signos medidos» | no hay `notebooks/cableado.json`: correr `dev.bringup()` con el motor, que lo escribe |
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

**El PWM va a 1050 Hz** (`PWM_TOP` en el sketch). Por qué no más rápido, por qué
no 1000 Hz justos y cómo convendría accionar un L298N: `hardware.ipynb`, sección 2.1.

---

## El enlace

La placa y la computadora hablan **CtrlLink**, un protocolo de texto por el puerto
serie a 1 Mbaud (divisor exacto en un AVR de 16 MHz). Las líneas que empiezan con
`#` son respuestas y eventos; cualquier otra es una fila de telemetría en
hexadecimal de ancho fijo, que se decodifica de un solo golpe con numpy y se puede
leer igual en el Monitor Serie.

| Comando | Respuesta |
|---|---|
| `id` | `# id CtrlLink 1 <sketch> chans=<n> row=<bytes> dt_us=<n>` |
| `params` | `# p <nombre> <tipo> <frac> <valor>`, uno por parámetro |
| `chans` | `# c <i> <nombre> <tipo> <escala> <unidad>`, uno por canal |
| `get <nombre>`, `set <nombre> <valor>` | `# v <nombre> <valor>` |
| `start` | un encabezado terminado en `# data`, y después las filas |
| `stop` | `# end rows=<n> drops=<n>` |

La placa **declara sola sus parámetros y canales** al conectarse, así que la
computadora no sabe nada de un sketch en particular: agregar una línea a
`g_params[]` la hace aparecer en el notebook. Un `set` durante una captura se
informa con `# mark <tick> <nombre> <valor>`, que es lo que da `t = 0` exacto en un
escalón. Los bytes que llegan a la placa pueden perderse (el USART guarda dos y la
ISR del muestreo no espera), así que `set` verifica el eco y reintenta. A 500 Hz la
fila de `Banco` usa el 14 % del enlace; lo que se acaba primero es la CPU de la
placa, y por eso `capture(..., canales=[...])` emite sólo lo que se pide.

---

## Organización

```
Banco/                   el banco: PWM afuera, ángulo y corriente adentro, telemetría
libraries/CtrlLink/      el protocolo, lado placa
libraries/Actuator/      el puente en H, o el transistor desde ENA
libraries/Sampler/       el reloj del muestreo: período rígido y divisor
libraries/Sense/         la corriente: ADC libre, promedio por ventana, contra la alimentación del sensor, notch de la red
libraries/AngleSensor/   ángulo desenrollado, y salud del sensor
libraries/Calibracion/   la corrección del error de ángulo
libraries/AS5600Async/   lectura asincrónica del AS5600
libraries/AS5600Regs/    el mapa de registros, sin ningún transporte
libraries/nI2C/          bus I2C por interrupciones (submódulo, de terceros)
libraries/BoardStart/    el reloj, el ADC y el destrabe del bus, antes de todo lo demás
test/test_modulos.cpp    los módulos que son aritmética pura, en la de escritorio
python/ctrllink.py       el protocolo, lado computadora
python/bench.py          el banco: conexión, bringup() y lo que significan sus números
python/placa.py          compilar y grabar el sketch, encontrar el puerto
python/ensayo.py         esperar al eje, la velocidad, las unidades y el archivo
python/catalogo.py       qué significa cada parámetro, y cómo mostrarlo
python/entorno.py        que el notebook corra en el entorno dyc, y qué hacer si no
python/banco_simulado.py un banco de mentira que sigue al de verdad, para dar la clase sin la placa
python/fakeuno.py        simulación del dispositivo, fiel byte a byte
python/test_*.py         pruebas, no necesitan hardware ni compilar
                         (test_notebooks.py además corre los notebooks simulados)
python/test_hardware.py  la única que sí necesita la placa: --motor mueve el eje
notebooks/hardware.ipynb el banco, cómo está armado y la API para el TP2
extras/calibracion_as5600/  opcional: calibración del AS5600 (notebook, calib.py, método, pruebas)
herramientas/verificar.py        que la instalación ande sin la placa: entorno, compilación, pruebas, notebooks
dyc.yml                  el entorno de conda del curso
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
