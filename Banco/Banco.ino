// El banco en lazo abierto: un comando de PWM que entra, y el ángulo y la
// corriente que salen. Gobernado desde un notebook de Jupyter por CtrlLink.
//
// Hardware:
// Arduino UNO
// Sensor de posición de efecto Hall: AS5600 (I2C)   SDA -> A4, SCL -> A5
// Medición de corriente (opcional): ACS712 en A0
// Actuador: ENA -> 9 (PWM, 1 kHz), IN1 -> 6, IN2 -> 7. Un puente L298N, o un
// transistor a masa con su diodo de rueda libre gobernado desde el pin 9.
//
// Acá no hay ley de control: `ctl_uff` va derecho al actuador. Todo lo que se hace
// con la medición --elegir el signo, derivar la velocidad, filtrar, ajustar un
// modelo-- pasa del lado de la computadora, donde se ve y se puede cambiar sin
// recompilar. Un filtro en la placa se identifica después como si fuera un polo
// del motor, y por eso no hay ninguno.
//
// Este archivo no hace casi nada por sí mismo: arma los módulos y publica sus
// parámetros. Cada cosa que se puede medir o accionar tiene un dueño:
//
//   g_clock    el reloj del muestreo          Sampler/SampleClock.h
//   g_adc      el conversor corriendo libre   Sense/FreeAdc.h
//   g_current  la corriente                   Sense/CurrentSense.h
//   g_lut      la corrección del ángulo       Calibracion/AngleLut.h
//   g_angle    el ángulo desenrollado         AngleSensor/AngleTracker.h
//   g_health   qué se le puede creer al sensor  AngleSensor/SensorHealth.h
//   g_motor    el actuador                    Actuator/HBridge.h
//
// El Timer2 muestrea el AS5600 a 5 kHz; cada `loop_div` muestras se emite una fila
// de telemetría, así que la frecuencia de las filas es 5000/loop_div Hz y por
// omisión vale 500 Hz. El muestreo mantiene un período rígido aunque el resto
// fluctúe; `loop_late` informa cuánta fluctuación hubo y `loop_missed` cuenta los
// períodos que se saltearon del todo.
//
// El puerto serie va a 1 Mbaud. En un AVR de 16 MHz ése es un divisor exacto
// (UBRR=1), a diferencia de 115200, que queda 2,1 % desviado. Los bytes entrantes
// llegan cada 10 us y el USART guarda sólo dos, así que entre el muestreador y la
// interrupción de TWI se pierde un pequeño porcentaje de los bytes de un comando
// enviado de corrido; la computadora los espacia para compensarlo. Ver PROTOCOL.md.
//
// Periféricos de los que se apropia este sketch: el Timer2, así que analogWrite()
// en los pines 3 y 11 y tone() dejan de funcionar; el Timer1, que modula el
// actuador con su propio TOP, así que analogWrite() en los pines 9 y 10 y Servo
// dejan de servir; y el ADC, que se maneja directamente acá, así que no hay que
// llamar a analogRead(). El Timer0 queda intacto: millis() y el PWM de los pines
// 5 y 6 andan como siempre.

#include <nI2C.h>

#include <AS5600.h>
#include <NI2CBus.h>
#include <BoardStart.h>
#include <CtrlLink.h>

#include <AngleLut.h>
#include <AngleTracker.h>
#include <SensorHealth.h>
#include <CurrentSense.h>
#include <FreeAdc.h>
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

// El TOP del Timer1, phase-correct con preescalador 1: f = 16 MHz / (2 * TOP), o
// sea 1 kHz. Es lo que tolera un L298N alimentado con 5 V: un puente de Darlington
// bipolares cae unos 2 V y tarda unos 2 us en conmutar, y a 20 kHz lo que se pierde
// en cada transición se lleva una fracción grande de un tiempo de encendido que ya
// venía escaso. Medido en este banco: a 20 kHz el motor no arranca y a 1 kHz anda.
//
// 1 kHz además entra dos veces justas en cada fila de 500 Hz, así que el
// muestreador toma siempre las mismas fases de la ondulación de corriente: un
// sesgo fijo en `i` en lugar de un batido lento.
static const uint16_t PWM_TOP = 8000;

// Medición de corriente en A0. Nada de este bloque mueve el motor: sólo fija las
// unidades que se le informan a la computadora.
//
// La sensibilidad es lo único que convierte cuentas en amperes, y depende de cuál
// sensor esté puesto: 185 mV/A el ACS712 de 5 A, 100 mV/A el de 20 A. OJO si no
// cierra con lo que mide un tester en serie con el motor: un divisor en la salida
// divide el cero y la sensibilidad a la vez.
//
// Contra Vcc, que es donde un sensor bipolar y ratiométrico reposa en media escala
// solo. Se paga en resolución: un LSB son 1,2 mV en 12 bits, y con 185 mV/A eso
// son 6,6 mA por cuenta --26 en el UNO, que cuenta de a cuatro--. Un motor chico,
// de decenas de mA en régimen, queda en pocas cuentas: el canal sirve para ver el
// arranque y comparar picos, y hay que medir su ruido antes de creerle algo más.
//
// Todo cuenta en 12 bits en las dos placas del banco: la lectura del UNO se corre
// dos bits para arriba adentro de FreeAdc. Vcc son los 5006 mV medidos con un
// tester en el UNO de este banco.
static const uint8_t  SENSE_CHANNEL    = 0;
static const float    SENSE_MV_PER_A   = 185.0f;
static const float    ADC_REF_MV       = 5006.0f;
static const uint16_t ADC_FULL         = 4096;
static const int16_t  SENSE_ZERO       = ADC_FULL / 2;
static const float    SENSE_MA_PER_LSB = 1000.0f * (ADC_REF_MV / ADC_FULL) / SENSE_MV_PER_A;

// ------------------------------------------------------------------ los módulos

typedef AS5600<NI2CBus>                                        Sensor;
typedef AngleLut<COUNTS_PER_REV, 64>                           Lut;
typedef AngleTracker<COUNTS_PER_REV>                           Angle;
typedef HBridge<MOTOR_PWM_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN>   Motor;
typedef FreeAdc<SENSE_CHANNEL>                                 Adc;

// 10 muestras de 5 kHz por fila son 500 Hz.
static SampleClock  g_clock(10);
static Adc          g_adc;
static CurrentSense g_current(SENSE_ZERO);
static Lut          g_lut;
static Angle        g_angle;
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

// La cuenta cruda del sensor, sin corregir. Es lo que indexa la tabla de
// calibración, así que es lo que la computadora necesita para calcularla.
static uint16_t g_y_raw = 0;

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
// python/calib.py--: una calibración es una propiedad del banco --este imán, en
// este eje-- y no del programa, y una tabla vieja aplicándose en silencio es peor
// que ninguna. Para dejarla fija en un tablero que se enciende solo,
// `calib.escribir_header()` genera Calibracion.h y este sketch lo toma si está.
#if defined(__has_include)
#  if __has_include("Calibracion.h")
#    include "Calibracion.h"
#    define TIENE_CALIBRACION 1
#  endif
#endif

// --------------------------------------------------------------------- tablas

static const CtrlParam PROGMEM g_params[] =
{
    { "ctl_uff",     CTRL_I16, &g_uff,               0 },

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
    { "y_uw",  CTRL_I32, &g_angle.y_uw,   COUNTS_TO_DEG,    "deg" },
    { "u",     CTRL_I16, &g_motor.u,      1.0f,             "pwm" },
    { "i",     CTRL_I16, &g_current.i,    SENSE_MA_PER_LSB, "mA"  },
};

// ----------------------------------------------------------------------- la ISR

ISR(TIMER2_COMPA_vect)
{
    Sensor::do_transfer();
    g_adc.on_isr();
    g_clock.on_isr();
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

// Lee los sensores y pone el comando sobre el actuador, una vez por fila.
//
// La corrección de la tabla se aplica sobre la cuenta cruda y antes de desenrollar,
// porque la tabla se indexa con el ángulo de adentro de la vuelta.
static void step(void)
{
    g_current.update(g_adc.read());

    const Lut::Counts raw = (Lut::Counts)Sensor::counts();

    g_y_raw = (uint16_t)raw;
    g_angle.update(g_cal ? g_lut.corrected(raw) : raw);

    g_motor.write(g_uff);
}

// Aplica lo que la computadora acaba de escribir. Lo dispara el contador de
// escrituras de CtrlLink: una comparación de 16 bits por pasada de loop().
static void refresh_tuning(void)
{
    CtrlLink::set_period_us(g_clock.apply(SAMPLE_HZ));

    g_lut.apply(g_lutw);

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

#ifdef TIENE_CALIBRACION
    memcpy_P(g_lut.entry, CAL_LUT, sizeof(g_lut.entry));
    g_cal = 1;
#endif

    CtrlLink::set_id(F("Banco"));
    CtrlLink::begin(BAUD,
                    g_params,   sizeof(g_params)   / sizeof(g_params[0]),
                    g_channels, sizeof(g_channels) / sizeof(g_channels[0]),
                    (uint32_t)g_clock.divide * 1000000UL / SAMPLE_HZ);

    refresh_tuning();

    g_adc.begin(adc_full, ADC_FULL);
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
