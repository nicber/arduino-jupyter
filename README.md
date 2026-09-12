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
dev.ctl_mode = MODE_PID

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

El recorrido completo del montaje --las configuraciones por las que crece el
banco, los diagramas de cableado y qué se puede medir con cada una-- está en
[`notebooks/hardware.ipynb`](notebooks/hardware.ipynb), que es además el que se
muestra en clase y el que arranca una práctica de identificación. Su sección 2.2
es el caso de **un solo cuadrante** --un puente cableado para un lado, o un
transistor y un diodo--, que es el de muchos bancos: qué cambia, qué hay que
verificar a mano, y por qué la identificación sale completa igual.

| Señal | Pin del UNO | |
|---|---|---|
| AS5600 SDA | A4 | |
| AS5600 SCL | A5 | |
| AS5600 VDD / GND | 5V / GND | |
| Salida del ACS712 | A0 | opcional |
| L298N `ENA` (PWM, 1 kHz) | 9 | opcional |
| L298N `IN1` | 6 | opcional |
| L298N `IN2` | 7 | opcional |

**La medición de corriente lee A0 contra la referencia alta**, que en el UNO es
AVcc y en el clon son los 4,096 V que trae trimados de fábrica. Manda el techo: un
ACS712 es bipolar y reposa en la mitad de su alimentación —2,5 V con 5 V— para
poder bajar cuando la corriente cambia de sentido, así que contra la referencia
interna de 1,1 V satura en reposo y no mide nada. Con la alta reposa en media
escala, que es justo donde tiene que estar.

Se paga en resolución: un LSB son 4,9 mV en vez de 1,07, y con 185 mV/A eso deja
200 mA en siete u ocho cuentas. `SENSE_REF_INTERNAL = true` vuelve a la interna y
recupera esas 4,5 veces, y es lo que corresponde si el sensor es unipolar —el que
va en la alimentación del puente, que reposa cerca de cero— o si hay un divisor a
la salida.

Dos números que conviene verificar una vez por banco, los dos en el sketch:

- `ADC_REF_MV`. Con la referencia alta es AVcc, que se mide una vez con un tester.
  Con la interna es el bandgap, especificado entre 1,0 y 1,2 V —o sea ±10 % de
  error de ganancia entre chips— y se mide sin instrumental leyendo el canal 14 del
  multiplexor del ADC contra AVcc; `bringup()` hace esa cuenta y dice qué poner.
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
cableado para un solo cuadrante se declara con `dev.mot_bidir = 0`, y ahí el comando
se recorta en cero y el anti-windup se entera; es un parámetro y no un `#define`,
así que se contesta desde el notebook y sin recompilar.

**Se puede empezar sin motor y sin medición de corriente.** Sin nada conectado a
los pines 9, 6 y 7 el lazo corre igual y la telemetría se comporta de manera
idéntica: alcanza con girar el imán a mano para ver al sensor y al filtro
trabajar; `bringup(motor=False)` saltea la parte que lo haría girar. Sin ACS712,
`bringup()` detecta que la entrada quedó contra el riel del ADC y lo dice; todo lo
demás sigue en pie.

Contra el riel de arriba hay una segunda causa que conviene descartar antes de ir
a buscar un cable: un sensor que reposa por encima de la referencia satura, y desde
el ADC eso se ve igual que una entrada al aire. Es lo que le pasa a un ACS712
alimentado a 5 V si alguien dejó `SENSE_REF_INTERNAL = true`. `bringup()` nombra
las dos causas en lugar de dar por sentado que falta un cable.

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
`libraries/BoardStart/`. `bringup()` lo verifica midiendo la frecuencia real del
lazo contra el reloj de la computadora.

**El ADC.** El del LGT8F328P es de 12 bits y el del ATmega328P de 10, así que
informa cuatro veces más cuentas por la misma tensión. Sin sensor de corriente da
igual; con uno, los amperes salen multiplicados por cuatro y hay que dividir
`SENSE_MV_PER_A` por cuatro. `bringup()` lo dice cuando lo ve.

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

Para el recorrido del hardware, `notebooks/hardware.ipynb`. Ése corre igual sin la
placa: si no encuentra el banco cae en uno simulado y lo dice, así que sirve para
mostrarlo en clase con el cable desenchufado.

Elegir el kernel *Arduino Control (.venv)* y correr las celdas en orden. La
primera compila y graba `ControlDemo` sola; la placa se encuentra sin nombrar
ningún puerto. La sección 1 es `dev.bringup()`, que verifica el equipo subsistema
por subsistema y es lo que conviene correr ante cualquier duda.

Cada celda arranca con `sync_board()`, que recompila si se editó el sketch, graba
si cambió el binario y reabre el enlace, lo que resetea la placa. Por eso las
celdas se pueden correr en cualquier orden: ninguna depende de lo que dejó la
anterior.

Ese reset devuelve la placa a los valores del sketch, que describen un banco
genérico. Lo que no es genérico --la calibración del sensor y los tres números del
cableado-- lo repone la computadora en cada celda; ver *El cableado del banco* y
*La calibración del sensor*.

---

## El cableado del banco

Tres números describen cómo está cableado este banco y no tienen nada que ver con
el programa:

| | |
|---|---|
| `bidir` | si el puente acciona en los dos sentidos o en uno solo |
| `uinvert` | el signo que hace que un comando positivo **suba** el ángulo |
| `iinvert` | el signo que hace que ese mismo comando dé una corriente **positiva** |

Son dos signos y no uno porque arreglan cosas distintas. `uinvert` da vuelta el
puente, así que da vuelta el ángulo y la corriente a la vez; si con el ángulo ya
derecho la corriente sigue saliendo al revés, lo que está dado vuelta es por dónde
entra el sensor de corriente, y eso sólo lo arregla `iinvert`. Importa más allá de
la telemetría: con `target = CURRENT` el lazo cierra sobre `i`, y realimentar con
el signo cambiado no se establece, se escapa.

`dev.bringup()` los mide, **los deja puestos** --no se limita a aconsejar-- y
devuelve un `Cableado` con los tres. De paso los anota en `notebooks/cableado.json`,
que es de donde `sync_board()` los vuelve a sacar después de cada reset.

```python
cab = dev.bringup()             # mide, corrige y anota
dev = sync_board()              # y cada celda arranca con el banco bien descripto
```

El archivo no entra en el repositorio, por lo mismo que la calibración: es un dato
de *este* banco. Para arrancar a propósito con los valores de fábrica,
`sync_board(cableado=Cableado())`.

Un banco de un solo cuadrante no se queda sin verificación. La del **sentido** no
se puede hacer --no hay una inversión que comprobar-- pero la de la **polaridad** y
la del **signo de la corriente** sí, porque para las dos alcanza con accionar para
un lado solo, y son justamente las que hacen falta para cerrar el lazo.

Lo que ningún parámetro arregla es un sentido fijado en cobre: si `IN1` e `IN2`
están atados a riel, o el actuador es un solo transistor, el puente no escucha qué
pin levanta el sketch. Por eso `bringup()` vuelve a accionar el motor después de
dar vuelta `uinvert` en lugar de darlo por arreglado; si el ángulo sigue bajando,
el arreglo son dos cables y lo dice así.

---

## Qué se puede tocar desde el notebook

Los parámetros son atributos, siempre en unidades reales:

| Parámetro | Qué es |
|---|---|
| `kp`, `ki`, `kd` | ganancias del PID, **por muestra** |
| `ctl_mode` | `MODE_OPEN`, `MODE_PID`, `MODE_RAMP` |
| `target` | `POSITION` o `CURRENT`: sobre qué magnitud cierra el lazo |
| `ref`, `refrate` | referencia y pendiente de rampa, en unidades del `target` |
| `ctl_uff` | comando de lazo abierto / prealimentación |
| `loop_div` | divisor del muestreador de 5 kHz: 10 → 500 Hz (por omisión), 5 → 1 kHz, 50 → 100 Hz |
| `bidir` | 1 si el puente acciona en los dos sentidos; 0 lo recorta en cero |
| `uinvert` | 1 si un comando positivo hace *bajar* el ángulo medido |
| `iinvert` | 1 si un comando positivo da una corriente *negativa* |
| `pwmtop` | TOP del Timer1: la frecuencia del PWM, `f = 16 MHz / (2·pwmtop)` |
| `alpha_y`, `alpha_i`, `alpha_e` | polos de los filtros de posición, corriente y error |
| `ang_offset` | cuenta del sensor de ángulo que se lee como cero |
| `izero` | LSB del ADC que se lee como corriente cero |
| `ang_cal` | 1 si se aplica la tabla de calibración del sensor; ver más abajo |
| `ang_sfilt` | filtro lento del AS5600: 0 es 16x (2,2 ms de retardo), 3 es 2x (0,286 ms) |
| `ang_lutw`, `ang_lutsum` | una entrada de la tabla de calibración, y la suma que verifica las 64 |
| `maxlate`, `missed`, `sovr`, `serr` | contadores de salud del lazo |
| `spres`, `mstat` | estado del sensor: si contesta en el bus, y qué dice del imán |
| `agc`, `mag` | ganancia y campo que ve el AS5600: con `agc` contra un extremo, el imán está a la distancia equivocada |

`bench.py` agrega encima las conversiones de este equipo, que son las que conviene
usar: `dev.gains(kp, ki, kd)` toma las ganancias **en tiempo continuo** y las
convierte a por-muestra, `dev.deg()` y `dev.ma()` escriben referencias en grados y
miliamperes, `dev.smooth('y', tau)` fija un filtro por constante de tiempo,
`dev.pwm(20000)` fija la frecuencia del PWM en Hz y devuelve la que realmente
quedó, `dev.zero()` toma la posición actual como cero y `dev.zero_current()` hace
lo propio con el sensor de corriente, que es la calibración de offset que `bringup()`
ya corre sola.

Para cambiar la *ley* de control —y no sus parámetros— el sketch arma módulos y
cada uno vive en `libraries/`. La ley está en `Control/Pid.h`, y agregar un
controlador es una clase nueva ahí más un caso en el `switch` de `control_step()`;
agregar una realimentación nueva es una rama en `measured_value()`, que es la única
función del sketch que sabe sobre qué magnitud cierra el lazo. Un parámetro nuevo es
una línea en la tabla `g_params[]`, y aparece solo en el notebook: del lado de
Python no hay nada que cambiar.

Los nombres de esa tabla llevan prefijo de módulo, que es lo que hace que tres
docenas de entradas planas digan de quién es cada una. `python/test_tablas.py` la
compara contra un golden guardado, así que un renombre o un reordenamiento se ve en
la revisión en lugar de descubrirse desde un notebook.

---

## Calibrar el sensor

El AS5600 no mide el ángulo que uno cree. Un imán descentrado --la hoja de datos
pide un cuarto de milímetro-- corre la lectura en una cantidad que depende del
ángulo y se repite vuelta tras vuelta, y al lazo se le presenta como una
ondulación de velocidad que ninguna ganancia arregla.

`notebooks/calibracion.ipynb` la mide, decide cuánto de lo que midió es el sensor
y cuánto es el motor, y arma una tabla de 128 bytes que la corrige adentro del
Arduino. Corre igual sin la placa: si no encuentra el banco cae en uno simulado y
lo dice. El método está en `Docs/CALIBRACION_AS5600.md`.

**La tabla no vive en la placa.** El dispositivo arranca siempre sin calibrar y la
dueña de la tabla es la computadora, que la empuja al conectarse. En el notebook
eso ya está hecho: cada celda arranca con `sync_board_cal()`, que es
`sync_board()` más la calibración de este banco.

```python
dev = sync_board_cal()          # compila, graba, reconecta y calibra
dev = sync_board()              # lo mismo sin calibrar, para medir el sensor crudo
```

Cargar la tabla son 64 escrituras de parámetro: medido en este banco, 1,2 s por
celda sobre los 1,9 s que ya cuesta resetear la placa. Debajo de todo está
`calib.asegurar(dev, ruta)`, que la aplica sólo si no está puesta.

No es una limitación de memoria. Una calibración es una propiedad de *este banco*
--este imán, en este eje-- y no del programa: en un archivo se lee, se compara y
se revisa; adentro de la placa es estado invisible que sobrevive a la
reprogramación. El caso feo no es la tabla que falta, es la tabla vieja de otro
montaje aplicándose en silencio. Para un tablero que se enciende solo,
`cal.escribir_header()` genera `ControlDemo/Calibracion.h` y el sketch lo toma en
la próxima compilación; ahí la calibración queda adentro de la placa, pero a la
vista en el código.

Y antes de calibrar nada, `sfilt = 3`. El AS5600 arranca con su filtro lento en
16x, que son 2,2 ms de retardo; en 2x son 0,286 ms. A cinco vueltas por segundo
esa diferencia son 39 cuentas de ángulo, contra un error que se espera de unas
pocas. Es la mejora más grande y más barata del asunto, y le sirve al lazo de
control tanto como a la medición.

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
| `bringup` marca falla en `cero de i` | el sensor de corriente no reposa en media escala: sin alimentar, mal cableado, o no es un ACS712 de 5 V |
| `bringup` marca falla en `polaridad` | dio vuelta `uinvert` y el ángulo siguió bajando, así que el sentido de este banco está fijado en cobre: `IN1` e `IN2` atados, o un solo transistor. Dar vuelta los dos cables del motor, o el imán |
| `bringup` marca falla en `sentido` | el puente no invierte. Si con el comando negativo no se mueve nada, o si gira para el mismo lado con las dos polaridades porque `IN1` (6) e `IN2` (7) están fijos por cable, es de un solo cuadrante y va `dev.mot_bidir = 0`. Si no, revisar esos dos pines |
| `bringup` marca falla en `motor` y el eje no gira | grabar `Puente_Bringup`: la placa lee sus propios pines de vuelta y separa «no sale el comando» de «el puente no lo sigue». Con el imán mal montado el ángulo es ruido y `bringup` no puede distinguirlos. La causa más común es la alimentación de potencia del puente |
| `bringup` dice «no se pudo evaluar» | falta el sensor del que esa verificación depende; arreglar primero el que sí falla |
| «el dispositivo declara sus parametros en un formato anterior» | la placa tiene grabado un sketch viejo: `sync_board(force_upload=True)` |
| se interrumpió una celda en medio de una captura | nada: la operación siguiente resincroniza el enlace sola. `dev.resync()` lo fuerza a mano |
| se pierden períodos de control | subir `loop_div`, o sacarle trabajo al paso de control |
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

## Encontrar las perillas

No hay ninguna lista de parámetros escrita del lado de Python. La placa declara su
tabla al conectarse, así que se le pregunta a ella:

```python
dev                      # en un notebook: la tabla entera, agrupada por subsistema
print(dev.describe())    # lo mismo como texto
dev.describe('ang')      # sólo lo del sensor de ángulo
```

Cada fila trae el valor de ahora, la unidad, y si el parámetro es una **perilla**
--se fija--, una **lectura** --la placa la publica-- o una **cuenta** --un total que
se puede poner en cero--. El nombre lleva un prefijo que dice de quién es: `ctl_`
qué se le pide al lazo, `pid_` la ley de control, `mot_` el puente, `ang_` el sensor
de ángulo, `cur_` la corriente, `loop_` el reloj y `board_` lo que la placa mide de
sí misma. Con eso `dev.ang_<TAB>` lista todo lo del sensor.

Las explicaciones viven en `python/catalogo.py` y no en el firmware, porque son
texto para una persona y el firmware tiene 32 kB. Que no se desactualicen no depende
de nadie: `test_catalogo.py` compara ese archivo contra la tabla del sketch y falla
si sobra o falta una entrada.

---

## Organización

```
ControlDemo/             el lazo de control: ley, parámetros y telemetría
AS5600_Bringup/          verificación del sensor, con volcado de configuración
AS5600_Loop5k/           prueba de muestreo a 5 kHz
Puente_Bringup/          verificación del accionamiento, sin usar el sensor
libraries/CtrlLink/      el protocolo, lado placa
libraries/ControlMath/   punto fijo y filtros enteros
libraries/Control/       el PID y la referencia
libraries/Actuator/      el puente en H
libraries/Sampler/       el reloj del lazo: período rígido y divisor
libraries/Sense/         el conversor libre, y una corriente con sentido
libraries/AngleSensor/   ángulo desenrollado, y salud del sensor
libraries/Calibracion/   la corrección del error de ángulo
libraries/AS5600Async/   lectura asincrónica del AS5600
libraries/AS5600Regs/    el mapa de registros, sin ningún transporte
libraries/nI2C/          bus I2C por interrupciones (submódulo, de terceros)
libraries/BoardStart/    el reloj, el ADC y el bus, antes de todo lo demás
test/test_modulos.cpp    los módulos que son aritmética pura, en la de escritorio
python/ctrllink.py       el protocolo, lado computadora
python/bench.py          compilación, conexión y unidades de este equipo
python/catalogo.py       qué significa cada parámetro, y cómo mostrarlo
python/calib.py          calibración del AS5600: medición, decisión y tabla
python/banco_simulado.py un banco de mentira, para dar la clase sin la placa
python/fakeuno.py        simulación del dispositivo, fiel byte a byte
python/test_*.py         pruebas, no necesitan hardware ni compilar
                         (test_notebooks.py además corre los notebooks simulados)
python/test_hardware.py  la única que sí necesita la placa: --motor mueve el eje
notebooks/               los notebooks: hardware, demostración y calibración
PROTOCOL.md              el protocolo: diseño, formato de línea y mediciones
Docs/CALIBRACION_AS5600.md  por qué la calibración es como es
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

Lo que **no** se pasa es `-flto`, y no por olvido: se probó y no cambia nada. Con
`-flto -fno-fat-lto-objects` al compilar y `-flto -fuse-linker-plugin` al enlazar,
los objetos intermedios salen distintos y el `.hex` final sale idéntico al byte,
medido antes y después de partir el sketch en módulos. Tiene sentido: todas las
librerías de este proyecto son sólo de cabecera, así que el sketch entero ya es una
sola unidad de traducción y no hay ninguna frontera que LTO pueda disolver. Lo que
queda afuera --nI2C y el core-- se alcanza por punteros de función y desde una ISR,
que no es algo que convenga inclinar hacia adentro. Así que serían tres
`--build-property` más para no ganar un byte.
