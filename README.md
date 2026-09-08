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

El lazo corre a 1 kHz con aritmética entera de punta a punta —ni una instrucción
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
| PWM del motor | 5 | opcional |
| Sentido de giro | 8 | opcional, desactivado en el sketch |

El AS5600 necesita un imán **magnetizado diametralmente** girando sobre el chip,
a un par de milímetros. Las plaquetas de AS5600 traen su propio regulador y los
pull-ups del bus; un chip pelado en modo 3,3 V necesita adaptación de niveles.

**Se puede empezar sin motor y sin medición de corriente.** Comentando
`#define MOTOR_PWM_PIN 5` en `ControlDemo/ControlDemo.ino` el lazo corre igual y
la telemetría se comporta de manera idéntica: alcanza con girar el imán a mano
para ver al sensor y al filtro trabajar. Sin ACS712, `bringup()` detecta que la
entrada quedó contra el riel del ADC y lo dice; todo lo demás sigue en pie.

Incluso sin el AS5600 el lazo mantiene su período: al no obtener respuesta, el
muestreo pasa a sondear el bus dos veces por segundo en lugar de cinco mil, y
`bringup()` informa `sensor: no contesta`. Sirve para probar la cadena completa
—compilar, grabar, capturar, graficar— antes de tener el sensor sobre la mesa,
teniendo en cuenta que el ángulo queda congelado en cero.

> ⚠️ **Con el motor conectado, el motor se mueve.** Conviene revisar que el eje
> esté libre antes de correr cualquier celda.

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
| `tickdiv` | divisor del muestreador de 5 kHz: 5 → 1 kHz, 50 → 100 Hz |
| `alpha_y`, `alpha_i`, `alpha_e` | polos de los filtros de posición, corriente y error |
| `offset` | cuenta del sensor que se lee como cero |
| `maxlate`, `missed`, `sovr`, `serr` | contadores de salud del lazo |
| `spres`, `mstat` | estado del sensor: si contesta en el bus, y qué dice del imán |

`bench.py` agrega encima las conversiones de este equipo, que son las que conviene
usar: `dev.gains(kp, ki, kd)` toma las ganancias **en tiempo continuo** y las
convierte a por-muestra, `dev.deg()` y `dev.ma()` escriben referencias en grados y
miliamperes, `dev.smooth('y', tau)` fija un filtro por constante de tiempo, y
`dev.zero()` toma la posición actual como cero.

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
| `bringup` dice «no se pudo evaluar» | falta el sensor del que esa verificación depende; arreglar primero el que sí falla |
| «el dispositivo declara sus parametros en un formato anterior» | la placa tiene grabado un sketch viejo: `sync_board(force_upload=True)` |
| se pierden períodos de control | subir `tickdiv`, o sacarle trabajo al paso de control |
| se descartan filas de telemetría | subir `dec`, o emitir menos canales |

Toda captura verifica su propia salud y avisa por `stderr` si el lazo perdió
períodos o si se perdieron filas: una serie temporal a la que le faltan muestras
se ve igual que una sana hasta que uno va a fijarse.

**Periféricos que se apropia `ControlDemo`:** el Timer2, así que `analogWrite()`
en los pines 3 y 11 y `tone()` dejan de funcionar; y el ADC, que se maneja
directamente, así que no hay que llamar a `analogRead()`. Los pines 9 y 10
(Timer1) y 5 y 6 (Timer0) no se ven afectados.

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
arduino-cli compile -b arduino:avr:uno --libraries ./libraries ControlDemo
arduino-cli upload  -b arduino:avr:uno --libraries ./libraries -p <puerto> ControlDemo
```
