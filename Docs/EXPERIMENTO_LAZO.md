# Dónde se va el tiempo del lazo, y qué cambia con el control en la ISR

Medido el 12/09/2026 en el banco: clon con CH340 en `/dev/cu.usbserial-1140`
(bootloader viejo de Nano), AS5600, puente L298N con el motor conectado. Firmware:
`ControlDemo` compilado con `-O2 -DCTRL_PROFILE`; sin esa bandera el sketch es el de
siempre. Los datos crudos están en `Docs/experimento_lazo/`, y los scripts en
`python/exp_loop*.py`.

Todos los tiempos están en µs. Donde hay dos números son promedio/máximo.

## La pregunta

¿Conviene pasar la ley de control a la ISR y dejar la telemetría en `loop()`? ¿El
costo grande es la UART? ¿Conviene transmitir desde una ISR, o por encuesta?

## Qué se midió

La instrumentación cronometra cada parte del lazo con `micros()`. La ISR de muestreo
y su retardo de entrada se miden con `TCNT2`, que cuenta de a 2 µs. `x_mode` cambia
la arquitectura en caliente:

| `x_mode` | Dónde corre `control_step()` | Dónde corre `emit()` |
|---|---|---|
| 0 | `loop()` (como hoy) | `loop()` |
| 1 | ISR, interrupciones cerradas | `loop()` |
| 2 | ISR, reabriéndolas antes (patrón de Grbl) | `loop()` |
| 3 | ISR, cerradas | ISR |
| 4 | ISR, reabiertas | ISR |

## 1. El presupuesto de un período

PID activo, flujo de 7 canales (45 bytes por fila), 1 Mbaud, modo 0:

| Parte | µs por período de control |
|---|---|
| ISR de muestreo (27 µs × 10 ticks a 500 Hz) | 270 |
| `control_step()` con PID (lazo abierto: ~130) | 274 / 292 |
| `emit()`: formatear en hexadecimal | 117 / 132 |
| `emit()`: **`Serial.write` de 45 bytes** | **574 / 592** |
| `poll()` sin comando, `refresh_magnet_status()` | < 10 |
| **Total** | **~1235**, sobre 2000 a 500 Hz |

**Enviar una fila cuesta el doble que calcular el PID.** A 1 kHz la cuenta no da:
135 + 274 + 690 ≈ 1100 µs contra 1000. Se pierden 310 de 5000 períodos, y el
histograma de atención se va a 400–800 µs. Pasa lo mismo en todas las variantes
(470 perdidos con el control en la ISR): el problema es cuánto trabajo hay, no
dónde se hace.

## 2. Por qué escribir cuesta tanto

Banco de escritura: N bytes con `HardwareSerial`, con las interrupciones cerradas
(sólo la copia al buffer), abiertas (con la ISR `UDRE` vaciando) y por encuesta
(esperar `UDRE0`, escribir `UDR0`). Mediana de seis repeticiones:

| N | Cerradas | Abiertas | Encuesta | Abiertas a 2 Mbaud | Encuesta a 2 Mbaud |
|---|---|---|---|---|---|
| 23 | 108 | 288 | 252 | 162 | 160 |
| 45 | 208 | 552 | 516 | 370 | 320 |
| 62 | 282 | 776 | 732 | 524 | 420 |

- **Por byte a 1 Mbaud:** ~4,5 µs copiando más ~7 µs en la ISR de transmisión,
  unos 12 µs en total. El cable pide 10 µs por byte, así que `HardwareSerial` ya
  está casi en el límite físico.
- **La encuesta ahorra ~7 %** a 1 Mbaud: 45 bytes en 516 µs contra 552.
- **Enviar desde una ISR no ahorra nada.** Ya se envía desde una: la `UDRE`, que
  a 1 Mbaud corre cada 10 µs y se lleva ~70 % de la CPU mientras sale una fila.
- **El costo es lineal en la cantidad de bytes.** La palanca es mandar menos bytes.

## 3. Latencia y fluctuación de la ley de control

500 Hz, PID, con flujo. `lte` es el retardo desde el tick hasta que `loop()` lo
atiende. En los modos 1 y 2 la ley ya corrió en la ISR, así que ese retardo deja
de afectar al actuador.

| Modo | Tick → inicio del cálculo | Histograma de atención (<100, <200, <400, <800, más) | Perdidos |
|---|---|---|---|
| 0 `loop()` | 35 / 330 | 2546, 1, 1, 0, 0 | 0 |
| 1 ISR | ~30 fijo (sólo la ISR de muestreo) | — | 0 |
| 2 ISR + `sei` | ~30 fijo | — | 0 |

- Con el control en `loop()`, el 99,9 % de los ticks se atiende en menos de
  100 µs. Los picos salen de eventos puntuales:
  - `refresh_tuning()`: ~250 µs;
  - un `set` durante la captura: 0,4–0,8 ms, porque la respuesta llena el buffer
    de transmisión y `print` bloquea;
  - el encabezado de `start`: **~11 ms**;
  - con una ráfaga de comandos el máximo llegó a 1,7 ms.
- Con el control en la ISR, el instante de actuación queda fijo en ~300 µs
  después del tick, pase lo que pase en `loop()`.

## 4. Pérdida de bytes de comando

Ráfaga de 50 `get x_mode` sin espaciar entre bytes:

| Condición | Respuestas correctas |
|---|---|
| Sin flujo, muestreador andando | 21 / 50 |
| **Sin flujo, muestreador pausado** | **50 / 50** |
| Flujo `dec=1` / `dec=10` | 20 / 50, 23 / 50 |
| Sin flujo, control en ISR cerrada (+270 µs de ISR) | 16 / 50 |
| Sin flujo, control en ISR reabierta | 21 / 50 |

**La pérdida la causa el muestreador de 5 kHz** (ISR de 27 µs cada 200 µs, más el
TWI), no la UART ni la ubicación del control. Poner el control en la ISR la empeora
poco o nada. La computadora ya espacia los bytes de comando, y eso sigue siendo
necesario con cualquier arquitectura.

## 5. Formatear y enviar desde la ISR (modos 3 y 4)

- **A 500 Hz** anda, pero la ISR pasa a durar ~960 µs. Además aparecen filas
  corruptas y huecos (1 a 4 por corrida) cuando una respuesta a un comando se
  escribe desde `loop()` al mismo tiempo: `HardwareSerial` no admite dos escritores.
- **A 1 kHz** colapsa. La ISR ocupa casi todo el período: 4300 de 5000 períodos
  perdidos, 600 solapamientos y `poll()` bloqueado hasta 15 ms.

**No conviene.**

## 6. 2 Mbaud

El divisor es exacto en 16 MHz (U2X, UBRR = 0) y el CH340 engancha.

| 1 kHz, PID, flujo | 1 Mbaud | 2 Mbaud |
|---|---|---|
| `emit()` (`Serial` / encuesta) | 684 / 644 | 498 / 415 |
| Períodos perdidos, modo 0 (`Serial` / encuesta) | 310 / 120 | **0 / 0** |
| Filas corruptas | 0 | **~0,7 %** (18–35 por corrida) |

El lazo de 1 kHz entra holgado, pero con este puente se corrompen filas y algún
comando. No es usable sin detección de errores, o sin probar otro puente USB-serie.

## Conclusiones

1. **El costo dominante es la cantidad de bytes en el cable**, ~12 µs cada uno a
   1 Mbaud, y no el formateo ni `HardwareSerial`. Una fila de 45 bytes cuesta
   ~690 µs; el PID, ~270.
   - Enviar desde una ISR no lo reduce.
   - La encuesta lo reduce un 7 %.
   - Lo que sí lo reduce es mandar menos bytes. Por la linealidad medida, una fila
     binaria de ~22 bytes costaría ~270 µs de escritura y casi nada de formateo,
     y con eso 1 kHz entraría a 1 Mbaud. Lo mismo vale para sacar canales
     redundantes (`y_raw` e `y_uwf` son iguales a `y_uw` sin calibración ni filtro)
     o subir `dec`.
2. **El control en la ISR reabriendo interrupciones (modo 2) funciona:** a 500 Hz
   no pierde períodos, fija el instante de actuación y no empeora de forma medible
   la pérdida de bytes. Lo que gana es inmunidad a los eventos lentos de `loop()`
   (comandos, encabezado, ajustes), no rendimiento. Este experimento **no**
   resolvió la carrera de los parámetros que escribe `loop()`: sólo midió tiempos.
3. **Formatear o enviar desde la ISR es contraproducente:** colapsa a 1 kHz y
   mezcla la salida con las respuestas.
4. **La encuesta sólo tiene sentido con el control en la ISR**, porque bloquea
   `loop()` todo el tiempo de cable.
5. **La pérdida de bytes de comando es del muestreador**, no de la telemetría.
