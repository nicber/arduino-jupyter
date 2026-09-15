// El banco en lazo abierto: un comando de PWM que entra, y el ángulo y la
// corriente que salen. Gobernado desde un notebook de Jupyter por CtrlLink.
//
// Hardware:
// Arduino UNO
// Sensor de posición de efecto Hall: AS5600 (I2C)   SDA -> A4, SCL -> A5
// Medición de corriente (opcional): ACS712 en A0, en serie entre +5 V y el motor con
// el diodo abarcando sensor y motor, y en A1 los 5 V que lo alimentan por un divisor
// resistivo (5,1 k arriba, 2 k abajo, 100 nF), si la placa no funciona a 5 V
// Actuador: ENA -> 9 (PWM, 1050 Hz), IN1 -> 6, IN2 -> 7. Un puente L298N, o un
// transistor a masa con su diodo de rueda libre gobernado desde el pin 9 (en este
// banco un BD139, NPN, con 220 Ω en la base).
//
// Acá no hay ley de control: `ctl_uff` va derecho al actuador, y el ángulo no tiene
// ningún filtro. Derivar, filtrar y ajustar se hace en la computadora. La corriente
// es la única excepción: cada fila publica el promedio de las conversiones de las
// últimas `cur_filas` filas --lo que hay que sacarle, el rizado del PWM y la red, ya
// no se puede sacar de filas de 2 ms, que lo traen plegado--, y `cur_notch` saca la red
// con una ventana corta. Por qué el hardware es como es --actuador, PWM, medición de
// corriente-- está explicado una sola vez, en notebooks/hardware.ipynb; acá quedan
// sólo las decisiones de implementación.
//
// Lo que se fija con cables lo carga la computadora al conectar: cuántos cuadrantes
// tiene el actuador (`mot_bidir`), los signos del imán y del sensor de corriente
// (`ang_inv`, `cur_inv`), que mide bringup(), y la relación del divisor de A1
// (`cur_div`).
//
// Este archivo no hace casi nada por sí mismo: arma los módulos y publica sus
// parámetros. Cada cosa que se puede medir o accionar tiene un dueño:
//
//   g_clock    el reloj del muestreo          Sampler/SampleClock.h
//   g_adc      el conversor corriendo libre   Sense/RowAdc.h
//   g_ratio    contra la alimentación del sensor  Sense/SupplyRatio.h
//   g_window   el promedio de la corriente    Sense/WindowMean.h
//   g_notch    el notch de la red             Sense/MainsNotch.h
//   g_current  la corriente                   Sense/CurrentSense.h
//   g_lut      la corrección del ángulo       Calibracion/AngleLut.h
//   g_turns    el ángulo desenrollado         AngleSensor/AngleTracker.h
//   g_health   qué se le puede creer al sensor  AngleSensor/SensorHealth.h
//   g_motor    el actuador                    Actuator/HBridge.h
//
// El Timer2 muestrea el AS5600 a 5 kHz; cada `loop_div` muestras se emite una fila
// de telemetría, así que la frecuencia de las filas es 5000/loop_div Hz y por
// omisión vale 500 Hz. El muestreo mantiene un período rígido aunque el resto
// fluctúe; `loop_late` informa cuánta fluctuación hubo y `loop_missed` cuenta los
// períodos que se saltearon del todo.
//
// El ángulo y la corriente de una fila se congelan en la ISR, en el tick mismo, y
// no cuando loop() llega a atenderlo. Si no, una fila atendida tarde llevaría la
// marca de su tick con una medición de hasta `loop_late` después, y eso al derivar
// es un error de velocidad que no se ve en ninguna parte. Congelado, el ángulo
// tiene un retardo fijo de un tick (200 us): es la transferencia que lanzó el tick
// anterior; y la ventana de la corriente termina en el tick.
//
// Y el ángulo se desenrolla en la ISR, sobre cada muestra de 5 kHz, y no una vez por
// fila: desenrollar sólo vale mientras el eje gire menos de media vuelta entre dos
// lecturas. Por fila, con `loop_div = 25` (200 Hz) eso son 628 rad/s, y un motor a
// 700 rad/s daba la velocidad con el signo cambiado sin ningún aviso. A 5 kHz el
// límite son 15 700 rad/s. La tabla de calibración se aplica en la fila, como una
// corrección chica sobre lo ya desenrollado.
//
// El puerto serie va a 1 Mbaud. En un AVR de 16 MHz ése es un divisor exacto
// (UBRR=1), a diferencia de 115200, que queda 2,1 % desviado. Los bytes entrantes
// llegan cada 10 us y el USART guarda sólo dos, así que entre el muestreador y la
// interrupción de TWI se pierde un pequeño porcentaje de los bytes de un comando
// enviado de corrido; la computadora los espacia para compensarlo. El protocolo está
// resumido en la sección «El enlace» del README.
//
// Periféricos de los que se apropia este sketch: el Timer2, así que analogWrite()
// en los pines 3 y 11 y tone() dejan de funcionar; el Timer1, que modula el
// actuador con su propio TOP, así que analogWrite() en los pines 9 y 10 y Servo
// dejan de servir; y el ADC, que se maneja directamente acá, así que no hay que
// llamar a analogRead(). El Timer0 queda intacto: millis() y el PWM de los pines
// 5 y 6 andan como siempre.

#include <util/atomic.h>

#include <nI2C.h>

#include <AS5600.h>
#include <NI2CBus.h>
#include <BoardStart.h>
#include <CtrlLink.h>

#include <AngleLut.h>
#include <AngleTracker.h>
#include <SensorHealth.h>
#include <CurrentSense.h>
#include <RowAdc.h>
#include <MainsNotch.h>
#include <SupplyRatio.h>
#include <WindowMean.h>
#include <HBridge.h>
#include <SampleClock.h>

// -------------------------------------------------------------------- el banco

static const uint32_t BAUD           = 1000000;
static const uint16_t SAMPLE_HZ      = 5000;
static const int16_t  COUNTS_PER_REV = 4096;

// ENA va al pin 9 porque es OC1A, y el Timer1 es el único que queda libre: el
// Timer0 (pines 5 y 6) lleva millis() y el Timer2 (pines 3 y 11) es el
// muestreador. IN1 e IN2 son salidas digitales comunes. Ver HBridge.h.
static const uint8_t MOTOR_PWM_PIN = 9;     // ENA, OC1A
static const uint8_t MOTOR_IN1_PIN = 6;     // IN1
static const uint8_t MOTOR_IN2_PIN = 7;     // IN2

// El TOP del Timer1, phase-correct con preescalador 1: f = 16 MHz / (2 * TOP) =
// 1050 Hz. Por qué ~1 kHz y no 20 kHz, y por qué 1050 y no 1000: hardware.ipynb,
// sección 2.1. Lo que importa acá: 20 ms de ventana de corriente son 21 períodos
// justos, así que el rizado que queda se pliega a 50 Hz, donde la ventana tiene un
// cero. Con 1010 Hz el pliegue caía en 10 Hz, un ripple de ~15 mA en régimen.
static const uint16_t PWM_TOP = 7619;

// Medición de corriente en A0. Nada de este bloque mueve el motor: sólo fija las
// unidades que se le informan a la computadora. SENSE_MV_PER_A es lo único que
// convierte cuentas en amperes: 185 mV/A el ACS712 de 5 A, 100 mV/A el de 20 A. Con
// `cur_div` la corriente sale de A0/A1, contra la alimentación del sensor (ver
// Sense/SupplyRatio.h); con `cur_div = 0`, de A0 contra AVCC. En las dos formas una
// cuenta publicada son 1,25 mV contra 5 V (6,8 mA con 185 mV/A). Por qué hace falta
// el divisor en el clon a 3,3 V: hardware.ipynb, sección 4. El ruido es de 120 mA RMS
// por conversión, medido en el clon, y por eso se promedia.
static const uint8_t  SENSE_CHANNEL    = 0;
static const uint8_t  SUPPLY_CHANNEL   = 1;
static const float    SENSE_MV_PER_A   = 185.0f;
static const uint16_t ADC_FULL         = 4096;
static const int16_t  SENSE_ZERO       = ADC_FULL / 2;
static const float    SENSE_MA_PER_LSB =
    (float)RowAdc<SENSE_CHANNEL, SUPPLY_CHANNEL>::UV_PER_COUNT / SENSE_MV_PER_A;

// ------------------------------------------------------------------ los módulos

typedef AS5600<NI2CBus>                                        Sensor;
typedef AngleLut<COUNTS_PER_REV, 64>                           Lut;
typedef AngleTracker<COUNTS_PER_REV>                           Angle;
typedef HBridge<MOTOR_PWM_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN>   Motor;
typedef RowAdc<SENSE_CHANNEL, SUPPLY_CHANNEL>                  Adc;

// Filas en la ventana de la corriente, por omisión: 10 filas de 2 ms son 20 ms, un
// período entero de la red de 50 Hz y veinte del PWM.
static const uint8_t CURRENT_ROWS = 10;

// 10 muestras de 5 kHz por fila son 500 Hz.
static SampleClock  g_clock(10);
static Adc          g_adc;
static WindowMean   g_window(CURRENT_ROWS);
static WindowMean   g_supply_window(CURRENT_ROWS);   // A1, con la misma ventana
static CurrentSense g_current(SENSE_ZERO);
static Lut          g_lut;
static Angle        g_turns;      // la cuenta cruda, desenrollada en la ISR
static SensorHealth g_health;
static Motor        g_motor(PWM_TOP);

// ------------------------------------------------ lo que es de este sketch y de nadie

// El comando que pide la computadora, -255..255. Lo que de verdad salió al
// actuador, ya recortado, es `u` en la telemetría.
static Motor::Command g_uff = 0;

// El filtro lento del AS5600. Arranca en 16x, que son 2,2 ms de retardo; en 2x son
// 0,286 ms. Ese retardo se identificaría después como si fuera del motor, así que
// se lo baja al mínimo al arrancar y no se lo vuelve a tocar.
static const uint8_t SENSOR_FILTER = Sensor::SF_2X;

// La cuenta cruda del sensor, sin corregir y sin el signo del banco. Es lo que
// indexa la tabla de calibración, así que es lo que la computadora necesita para
// calcularla.
static uint16_t g_y_raw = 0;

// Lo que se publica: el ángulo desenrollado y la corriente, con el signo del banco.
static int32_t g_y_uw = 0;
static int16_t g_i    = 0;

// 1 si la cuenta de esta fila repite la anterior: la transferencia del AS5600 que
// tenía que traerla no terminó a tiempo (un desborde). Derivada, esa fila da una
// velocidad falsa, y sin la marca no hay manera de saber cuál es.
static uint8_t g_y_rep = 0;

// Los signos del banco: 1 si un comando positivo, sin corregir, hace bajar el
// ángulo o sale como corriente negativa. Los mide bringup().
static uint8_t g_ang_inv = 0;
static uint8_t g_cur_inv = 0;

// La corriente contra la alimentación del sensor. Arranca sin divisor, contra AVCC: la
// relación es del cableado de cada banco y la carga la computadora.
static SupplyRatio g_ratio;

// El notch de la red sobre la corriente, fila por fila. Arranca apagado: con la
// ventana de 20 ms por omisión no hace falta. Ver Sense/MainsNotch.h.
static MainsNotch g_notch;

// Lo que la ISR congela en el tick de cada fila, y el contador de muestras del
// AS5600 en el tick anterior, que es lo que dice si la cuenta es nueva.
static volatile uint16_t g_tick_raw     = 0;
static volatile int32_t  g_tick_raw_uw  = 0;
static volatile uint8_t  g_tick_fresh   = 0;
static uint16_t          g_isr_samples  = 0;

// Prende y apaga la corrección en caliente, que es lo que permite medir cuánto
// sirve en lugar de suponerlo.
static uint8_t g_cal = 0;

// Una entrada de la tabla de calibración por escritura. Ver AngleLut::apply().
static uint32_t g_lutw   = Lut::NOTHING;
static uint16_t g_lutsum = 0;

// Si había flujo en la pasada anterior, y el contador de escrituras que se vio la
// última vez.
static bool     g_was_streaming = false;
static uint16_t g_last_writes   = 0;

// ------------------------------------------------------------ la calibración

// La tabla NO se guarda en la placa. El dispositivo arranca siempre sin calibrar,
// y quien tiene la tabla es la computadora, que la empuja al conectarse --ver
// extras/calibracion_as5600/calib.py--: una calibración es una propiedad del banco
// --este imán, en este eje-- y no del programa, y una tabla vieja aplicándose en
// silencio es peor que ninguna.

// --------------------------------------------------------------------- tablas

static const CtrlParam PROGMEM g_params[] =
{
    { "ctl_uff",     CTRL_I16, &g_uff,               0 },

    { "mot_bidir",   CTRL_U8,  &g_motor.bidir,       0 },

    { "ang_inv",     CTRL_U8,  &g_ang_inv,           0 },
    { "ang_cal",     CTRL_U8,  &g_cal,               0 },
    { "ang_lutw",    CTRL_U32, &g_lutw,              0 },
    { "ang_lutsum",  CTRL_U16, &g_lutsum,            0 },
    { "ang_status",  CTRL_U8,  &g_health.status,     0 },
    { "ang_present", CTRL_U8,  &g_health.present,    0 },
    { "ang_agc",     CTRL_U8,  &g_health.agc,        0 },
    { "ang_mag",     CTRL_U16, &g_health.magnitude,  0 },
    { "ang_busovr",  CTRL_U16, &g_health.overruns,   0 },
    { "ang_buserr",  CTRL_U16, &g_health.errors,     0 },

    { "cur_zero",    CTRL_I16, &g_current.zero,      0 },
    { "cur_inv",     CTRL_U8,  &g_cur_inv,           0 },
    { "cur_filas",   CTRL_U8,  &g_window.rows,       0 },
    { "cur_div",     CTRL_U16, &g_ratio.div_e4,      0 },
    { "cur_a1",      CTRL_U16, &g_ratio.supply,      0 },
    { "cur_notch",   CTRL_U8,  &g_notch.harmonics,   0 },
    { "cur_notchr",  CTRL_U16, &g_notch.pole_milli,  0 },

    { "loop_div",    CTRL_U8,  &g_clock.divide,      0 },
    { "loop_late",   CTRL_U16, &g_clock.late,        0 },
    { "loop_missed", CTRL_U16, &g_clock.missed,      0 },
};

static const float COUNTS_TO_DEG = 360.0f / COUNTS_PER_REV;

// Los canales no llevan prefijo: son columnas de una serie temporal y viajan hasta
// un DataFrame, donde el nombre lo escribe quien grafica.
static const CtrlChannel PROGMEM g_channels[] =
{
    { "y_raw", CTRL_U16, &g_y_raw,        COUNTS_TO_DEG,    "deg" },
    { "y_uw",  CTRL_I32, &g_y_uw,         COUNTS_TO_DEG,    "deg" },
    { "y_rep", CTRL_U8,  &g_y_rep,        1.0f,             ""    },
    { "u",     CTRL_I16, &g_motor.u,      1.0f,             "pwm" },
    { "i",     CTRL_I16, &g_i,            SENSE_MA_PER_LSB, "mA"  },
};

// ----------------------------------------------------------------------- la ISR

ISR(TIMER2_COMPA_vect)
{
    // Antes de lanzar la transferencia de este tick: si el contador no avanzó
    // desde el tick anterior, la que se lanzó entonces no terminó, y la cuenta que
    // hay es la de antes.
    const uint16_t samples = Sensor::samples();
    const uint8_t  fresh   = (samples != g_isr_samples);
    g_isr_samples = samples;

    Sensor::do_transfer();

    // Una vuelta de desenrollado por muestra. Una muestra repetida no avanza nada.
    const uint16_t counts = Sensor::counts();
    g_turns.update((Angle::Counts)counts);

    if (g_clock.on_isr())
    {
        g_tick_raw    = counts;
        g_tick_raw_uw = g_turns.y_uw;
        g_tick_fresh  = fresh;
        g_adc.close_row();
    }
}

// El ADC corre libre, sin relación con el muestreador: cada conversión se suma a la
// fila en curso. Ver Sense/RowAdc.h.
ISR(ADC_vect)
{
    g_adc.on_conversion();
}

// ------------------------------------------------------------------- el paso

// Escribe los bits SF del CONF del sensor. Lee-modifica-escribe, porque CONF
// también lleva la histéresis, el modo de potencia y la salida.
//
// La lectura viaja en un tick del muestreador, así que hace falta que ya esté
// corriendo. La escritura no: nI2C la encola y reserva memoria al hacerlo, y el
// muestreador llama a nI2C desde una ISR de temporizador, así que se lo para un par
// de milisegundos para que nadie más pida el bus. Devuelve false si el sensor no
// contestó.
static bool apply_sensor_filter(void)
{
    uint8_t conf[2];

    if (!Sensor::read_registers(Sensor::REG_CONF_H, conf, 2))
    {
        return false;
    }

    uint16_t value = (uint16_t)(((uint16_t)conf[0] << 8) | conf[1]);
    value = (uint16_t)((value & ~0x0300u) | ((uint16_t)(SENSOR_FILTER & 0x03) << 8));

    conf[0] = (uint8_t)(value >> 8);
    conf[1] = (uint8_t)value;

    g_clock.pause();

    const uint32_t deadline = millis() + 5;
    while (Sensor::busy() && (int32_t)(millis() - deadline) < 0)
    {
    }

    const bool queued = Sensor::write_registers(Sensor::REG_CONF_H, conf, 2);

    delay(2);   // cuatro bytes a 400 kHz son unos 100 us; esto es holgura

    g_clock.resume();

    return queued;
}

// Toma lo que la ISR congeló en el tick y pone el comando sobre el actuador, una
// vez por fila.
//
// El ángulo llega desenrollado desde la ISR. La corrección de la tabla se indexa con
// la cuenta cruda de adentro de la vuelta y se suma como diferencia --unas pocas
// cuentas, sin vuelta de por medio--. El signo va al final: la tabla y el
// desenrollado hablan del imán, y el signo, del banco.
//
// La corriente es el promedio de todas las conversiones de las últimas `cur_filas`
// filas, de A0 y, con divisor, de A1, llevado a cuentas equivalentes; el cero se
// resta después, sobre el promedio.
static void step(void)
{
    uint16_t raw;
    int32_t  raw_uw;
    uint8_t  fresh;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        raw    = g_tick_raw;
        raw_uw = g_tick_raw_uw;
        fresh  = g_tick_fresh;
    }

    uint32_t sum, supply_sum;
    uint16_t n, supply_n;
    g_adc.row(sum, n, supply_sum, supply_n);

    g_window.push(sum, n);
    g_supply_window.push(supply_sum, supply_n);
    g_current.update(g_ratio.counts(g_window.total(), g_window.count(),
                                    g_supply_window.total(), g_supply_window.count()));

    const int16_t i = g_notch.step(g_current.i);
    g_i = g_cur_inv ? (int16_t)-i : i;

    g_y_raw = raw;
    g_y_rep = !fresh;
    const int32_t uw = raw_uw
        + (g_cal ? Angle::wrapped_error(g_lut.corrected((Lut::Counts)raw), (Angle::Counts)raw)
                 : 0);
    g_y_uw = g_ang_inv ? -uw : uw;

    g_motor.write(g_uff);
}

// Aplica lo que la computadora acaba de escribir. Lo dispara el contador de
// escrituras de CtrlLink: una comparación de 16 bits por pasada de loop().
static void refresh_tuning(void)
{
    CtrlLink::set_period_us(g_clock.apply(SAMPLE_HZ));

    g_lut.apply(g_lutw);

    g_ratio.apply();
    g_adc.alternate(g_ratio.active());

    g_window.apply();
    g_supply_window.rows = g_window.rows;
    g_supply_window.apply();

    // Con la frecuencia de las filas de ahora: `loop_div` también la mueve.
    g_notch.apply((float)SAMPLE_HZ / (float)g_clock.divide);

    // Se recalcula siempre y no sólo al escribir la tabla: así `ang_lutsum`
    // describe lo que hay, y la computadora verifica 64 entradas con una lectura.
    g_lutsum = g_lut.checksum();
}

// La visión que el propio AS5600 tiene del imán: detectado, muy débil, muy fuerte,
// y con qué ganancia lee. Leerla le cuesta al muestreo una muestra, así que sólo se
// hace entre capturas, y un registro por vez: una lectura de tres bytes no entra en
// el período de 200 us. Ver SensorHealth::turn().
static void refresh_magnet_status(void)
{
    if (CtrlLink::streaming() || !g_health.due(millis()))
    {
        return;
    }

    if (!Sensor::present())
    {
        g_health.forget_mounting();
        return;
    }

    uint8_t buf[2];

    switch (g_health.turn())
    {
        case 0:
            if (Sensor::read_registers(Sensor::REG_STATUS, buf, 1))
            {
                g_health.status = buf[0];
            }
            break;

        case 1:
            if (Sensor::read_registers(Sensor::REG_AGC, buf, 1))
            {
                g_health.agc = buf[0];
            }
            break;

        default:
            if (Sensor::read_registers(Sensor::REG_MAGNITUDE_H, buf, 2))
            {
                g_health.magnitude =
                    (uint16_t)((((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF);
            }
            break;
    }

    g_health.advance(3);
}

// Los contadores de salud describen la ventana de emisión, así que se ponen en cero
// cuando se abre una: arrancarla cuesta unos milisegundos de puerto serie, y los
// períodos que se pierden ahí son el precio de arrancar la captura.
static void reset_health_on_capture(void)
{
    const bool now = CtrlLink::streaming();

    if (now && !g_was_streaming)
    {
        g_clock.clear_health();
    }

    g_was_streaming = now;
}

// ------------------------------------------------------------------- Arduino

void setup()
{
    // El reloj antes que nada: si la placa no corre a 16 MHz, el UART emite al
    // ritmo equivocado y ni el mensaje de error llega. Después el ADC contra Vcc,
    // y el bus: grabar la placa la resetea, y un reset en medio de una lectura deja
    // al AS5600 sujetando SDA. Ver BoardStart.h.
    board::clock_begin();

    const uint16_t adc_full = board::adc_full_scale();
    board::adc_select_vcc(adc_full >= ADC_FULL);

    board::bus_recover();

    // El actuador: ENA abierto y las dos entradas de sentido en bajo.
    g_motor.begin();

    CtrlLink::set_id(F("Banco"));
    CtrlLink::begin(BAUD,
                    g_params,   sizeof(g_params)   / sizeof(g_params[0]),
                    g_channels, sizeof(g_channels) / sizeof(g_channels[0]),
                    (uint32_t)g_clock.divide * 1000000UL / SAMPLE_HZ);

    refresh_tuning();

    g_adc.begin(adc_full);
    Sensor::begin();
    g_clock.begin(SAMPLE_HZ);

    // El filtro del sensor, recién ahora: la lectura que precede a la escritura
    // viaja en un tick de muestreo. Unos pocos intentos, por si la primera muestra
    // todavía no salió; sin sensor en el bus se sigue igual.
    for (uint8_t i = 0; i < 10 && !apply_sensor_filter(); i++)
    {
        delay(2);
    }

    CtrlLink::note(F("Banco listo"));
}

void loop()
{
    if (g_clock.take())
    {
        step();

        g_health.accumulate(Sensor::overruns(), Sensor::errors(), Sensor::present());

        CtrlLink::emit();
    }

    // Después del paso, nunca antes: un `set` que caiga justo cuando se dispara un
    // tick quedaría de otro modo por delante de él.
    const uint16_t writes = CtrlLink::writes();
    if (writes != g_last_writes)
    {
        g_last_writes = writes;
        refresh_tuning();
    }

    refresh_magnet_status();

    CtrlLink::poll();

    // Después de poll(), que es donde se atiende `start` y se imprime el
    // encabezado: así la ventana empieza a contar recién cuando ya salió.
    reset_health_on_capture();
}
