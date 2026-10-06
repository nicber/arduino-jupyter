// El banco en lazo abierto: un comando de PWM que entra, y el ángulo y la
// corriente que salen. Gobernado desde un notebook de Jupyter por CtrlLink.
//
// Hardware:
// Arduino UNO
// Sensor de posición de efecto Hall: AS5600 (I2C)   SDA -> A4, SCL -> A5
// Medición de corriente (opcional): ACS712 en A0, en serie entre +5 V y el motor con
// el diodo abarcando sensor y motor, y en A1 los 5 V que lo alimentan por un divisor
// resistivo (5,1 k arriba, 2 k abajo), si la placa no funciona a 5 V
// Actuador: ENA -> 9 (PWM, 1250 Hz), IN1 -> 6, IN2 -> 7. Un puente L298N, o un
// transistor a masa con su diodo de rueda libre gobernado desde el pin 9 (en este
// banco un BD139, NPN, con 220 Ω en la base).
//
// Acá no hay ley de control: `ctl_uff` va derecho al actuador, y el ángulo no tiene
// ningún filtro. Derivar, filtrar y ajustar se hace en la computadora. La corriente
// es la única excepción: cada fila publica el promedio de las conversiones de las
// últimas `cur_filas` filas, en dieciseisavos de cuenta del ADC --lo que hay que
// sacarle, el rizado del PWM y la red, ya
// no se puede sacar de filas de 2 ms, que lo traen plegado--. Antes de sumar la fila,
// cada tick de 5 kHz pasa por una media de 4 ticks, un período del PWM (`cur_ma`), y
// después de promediar, un notch en 250 Hz saca lo que el PWM deja en el Nyquist de
// las filas (`cur_nyq`). El notch de la red (`cur_notch`) arranca apagado. Por
// qué el hardware es como es --actuador, PWM, medición de corriente-- está explicado
// una sola vez, en notebooks/hardware.ipynb; acá quedan sólo las decisiones de
// implementación.
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
//   g_adc      el conversor, encadenado       Sense/RowAdc.h
//   g_current  la corriente: la ventana, el divisor, el cero, el notch y el signo
//                                             Sense/CurrentSense.h
//   g_lut      la corrección del ángulo       Calibracion/AngleLut.h
//   g_angle    el ángulo, muestreado y desenrollado en la ISR
//                                             AngleSensor/AngleSampler.h
//   g_health   la confiabilidad del sensor    AngleSensor/SensorHealth.h
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
// lecturas. Por fila, con `loop_div = 25` (200 Hz) eso son 628 rad/s, y un motor
// más rápido daría la velocidad con el signo cambiado sin ningún aviso. A 5 kHz el
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
// llamar a analogRead(). El Timer0 sigue llevando millis(), pero a 1000 Hz en lugar
// de 976,6 Hz, así que millis() queda un 2,4 % rápido y el PWM de los pines 5 y 6 deja
// de servir (ver BoardStart/BoardClock.h).

#include <nI2C.h>

#include <AS5600.h>
#include <NI2CBus.h>
#include <BoardStart.h>
#include <CtrlLink.h>

#include <AngleLut.h>
#include <AngleSampler.h>
#include <SensorHealth.h>
#include <CurrentSense.h>
#include <RowAdc.h>
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
// 1250 Hz. Por qué ~1 kHz y no 20 kHz: hardware.ipynb, sección 2.1. Por qué 1250: son
// exactamente 4 ticks del muestreador de 5 kHz, que sale del mismo reloj. Una media de
// 4 ticks tiene ceros en 1250, 2500, 3750... Hz, todos los armónicos del PWM, antes de
// decimar a las filas (ver Sense/RowAdc.h); lo que queda del PWM cae en 250 Hz, el
// Nyquist de las filas, donde va un notch (Sense/MainsNotch.h), y en 20 ms, que son
// 25 períodos justos, la ventana de la corriente lo anula.
static const uint16_t PWM_TOP = 6400;

// Medición de corriente en A0. Nada de este bloque mueve el motor: sólo fija las
// unidades que se le informan a la computadora. SENSE_MV_PER_A es lo único que
// convierte cuentas en amperes: 185 mV/A el ACS712 de 5 A, 100 mV/A el de 20 A. Con
// `cur_div` la corriente sale de A0/A1, contra la alimentación del sensor (ver
// Sense/SupplyRatio.h); con `cur_div = 0`, de A0 contra AVCC. En las dos formas una
// cuenta equivalente son 1,25 mV contra 5 V (6,8 mA con 185 mV/A). Por qué hace falta
// el divisor en el clon a 3,3 V: hardware.ipynb, sección 4. El ruido es de 120 mA RMS
// por conversión, medido en el clon, y por eso se promedia.
//
// El canal `i` no publica cuentas sino **dieciseisavos de cuenta**: el redondeo a
// cuenta entera era, medido, el que más ruido ponía en toda la cadena, y el sensor usa
// 740 de las 2047 cuentas que entran en un int16 en esa unidad. `cur_frac` dice
// cuántos bits fraccionarios son, para que la computadora reconstruya la cuenta cruda
// del ADC --que es la unidad de `cur_zero`-- sin tenerlo escrito en ninguna parte.
static const uint8_t  SENSE_CHANNEL    = 0;
static const uint8_t  SUPPLY_CHANNEL   = 1;
static const float    SENSE_MV_PER_A   = 185.0f;
static const uint16_t ADC_FULL         = 4096;
static const int16_t  SENSE_ZERO       = ADC_FULL / 2;
static const uint8_t  SENSE_FRAC_BITS  = CurrentSense::FRAC_BITS;
static const float    SENSE_MA_PER_LSB =
    (float)SupplyRatio::UV_PER_COUNT / SENSE_MV_PER_A / (float)(1 << SENSE_FRAC_BITS);

// ------------------------------------------------------------------ los módulos

typedef AS5600<NI2CBus>                                        Sensor;
typedef AngleLut<COUNTS_PER_REV, 64>                           Lut;
typedef HBridge<MOTOR_PWM_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN>   Motor;
typedef RowAdc<SENSE_CHANNEL, SUPPLY_CHANNEL>                  Adc;

// Filas en la ventana de la corriente, por omisión: 10 filas de 2 ms son 20 ms, un
// período entero de la red de 50 Hz y 21 del PWM.
static const uint8_t CURRENT_ROWS = 10;

// 10 muestras de 5 kHz por fila son 500 Hz.
static SampleClock  g_clock(10);
static Adc          g_adc;
static CurrentSense g_current(SENSE_ZERO, CURRENT_ROWS);
static Lut          g_lut;
static AngleSampler<Sensor, COUNTS_PER_REV> g_angle;
static SensorHealth g_health;
static Motor        g_motor(PWM_TOP);

// ------------------------------------------------ el estado propio del sketch

// El comando que pide la computadora, -255..255. Lo que efectivamente salió al
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

// Lo que se publica del ángulo: desenrollado, con el signo del banco. La corriente
// se publica desde g_current.
static int32_t g_y_uw = 0;

// Los bits fraccionarios de la unidad de `i`. Es una constante del sketch y se publica
// para que la computadora no la tenga escrita.
static uint8_t g_cur_frac = SENSE_FRAC_BITS;

// 1 si la cuenta de esta fila repite la anterior: la transferencia del AS5600 que
// tenía que traerla no terminó a tiempo (un desborde). Derivada, esa fila da una
// velocidad falsa, y sin la marca no hay manera de saber cuál es.
static uint8_t g_y_rep = 0;

// Las perillas de diagnóstico del conversor, compiladas afuera salvo -DSENSE_DIAG=1.
// Ver SENSE_DIAG en Sense/RowAdc.h: son con las que se eligió el relleno.
#if SENSE_DIAG
static uint8_t  g_dbg_i2c = 1;   // 0 corta las transferencias del AS5600 en la ISR
static uint8_t  g_dbg_ocr = 99;  // el TOP del Timer2: mueve el ritmo del tick
static uint16_t g_a0 = 0;        // la media cruda de A0 de la fila, en dieciseisavos
static uint16_t g_a1 = 0;
static uint16_t g_conv = 0;      // cuántas conversiones entraron en la fila
#endif

// El signo del ángulo: 1 si un comando positivo, sin corregir, lo hace bajar. Lo
// mide bringup(), igual que el de la corriente (`cur_inv`).
static uint8_t g_ang_inv = 0;

// La corriente arranca sin divisor, contra AVCC: la relación es del cableado de cada
// banco y la carga la computadora (`cur_div`).
//
// Los notch sobre la corriente, fila por fila. El de 250 Hz (`cur_nyq`) arranca
// prendido. El de la red arranca apagado: medido con 1 fila, deja una oscilación en el
// arranque de un escalón de hasta 90 mA con r = 0,95 y de ~50 mA con r = 0,98 o 0,99,
// y con el divisor en A1 la red ya es de ~1 mA. `cur_notch` lo prende, cada armónico
// por separado (máscara: 1 = 50, 2 = 100, 4 = 150 Hz). Ver Sense/MainsNotch.h.
static const uint8_t MAINS_HARMONICS = 0;

// Prende y apaga la corrección en caliente, que es lo que permite medir cuánto
// sirve en lugar de suponerlo.
static uint8_t g_cal = 0;

// Una entrada de la tabla de calibración por escritura. Ver AngleLut::apply().
static uint32_t g_lutw = Lut::NOTHING;

// El contador de escrituras que se vio la última vez.
static uint16_t g_last_writes = 0;

// ------------------------------------------------------------ la calibración

// La tabla NO se guarda en la placa. El dispositivo arranca siempre sin calibrar,
// y quien tiene la tabla es la computadora, que la empuja al conectarse --ver
// extras/calibracion_as5600/calib.py--: una calibración es una propiedad del banco
// --este imán, en este eje-- y no del programa, y una tabla desactualizada
// aplicándose en silencio es peor que ninguna.

// --------------------------------------------------------------------- tablas

static const CtrlParam PROGMEM g_params[] =
{
    { "ctl_uff",     CTRL_I16, &g_uff,               0 },

    { "mot_bidir",   CTRL_U8,  &g_motor.bidir,       0 },

    { "ang_inv",     CTRL_U8,  &g_ang_inv,           0 },
    { "ang_cal",     CTRL_U8,  &g_cal,               0 },
    { "ang_lutw",    CTRL_U32, &g_lutw,              0 },
    { "ang_lutsum",  CTRL_U16, &g_lut.sum,           0 },
    { "ang_status",  CTRL_U8,  &g_health.status,     0 },
    { "ang_present", CTRL_U8,  &g_health.present,    0 },
    { "ang_agc",     CTRL_U8,  &g_health.agc,        0 },
    { "ang_mag",     CTRL_U16, &g_health.magnitude,  0 },
    { "ang_busovr",  CTRL_U16, &g_health.overruns,   0 },
    { "ang_buserr",  CTRL_U16, &g_health.errors,     0 },

    { "cur_zero",    CTRL_I16, &g_current.zero,      0 },
    { "cur_frac",    CTRL_U8,  &g_cur_frac,          0 },
#if SENSE_DIAG
    { "dbg_i2c",     CTRL_U8,  &g_dbg_i2c,           0 },
    { "dbg_ocr",     CTRL_U8,  &g_dbg_ocr,           0 },
    { "dbg_settle",  CTRL_U8,  &g_adc.settle_us,     0 },
    { "dbg_lock",    CTRL_U8,  &g_adc.lock_n,        0 },
    { "dbg_pre",     CTRL_U8,  &g_adc.pre,           0 },
    { "dbg_mix",     CTRL_U8,  &g_adc.mix,           0 },
#endif
    { "cur_inv",     CTRL_U8,  &g_current.invert,            0 },
    { "cur_filas",   CTRL_U8,  &g_current.window.rows,       0 },
    { "cur_div",     CTRL_U16, &g_current.ratio.div_e4,      0 },
    { "cur_a1",      CTRL_U16, &g_current.ratio.supply,      0 },
    { "cur_notch",   CTRL_U8,  &g_current.notch.harmonics,   0 },
    { "cur_notchr",  CTRL_U16, &g_current.notch.pole_milli,  0 },
    { "cur_nyq",     CTRL_U8,  &g_current.notch.nyquist,     0 },
    { "cur_ma",      CTRL_U8,  &g_adc.ma,            0 },

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
    { "i",     CTRL_I16, &g_current.i,    SENSE_MA_PER_LSB, "mA"  },
#if SENSE_DIAG
    { "a0",    CTRL_U16, &g_a0,           0.0625f,          ""    },
    { "a1",    CTRL_U16, &g_a1,           0.0625f,          ""    },
    { "conv",  CTRL_U16, &g_conv,         1.0f,             ""    },
#endif
};

// ----------------------------------------------------------------------- la ISR

ISR(TIMER2_COMPA_vect)
{
    // Lo que el ADC sumó en este tick, con la media de 4 ticks si está prendida. El ADC
    // no interrumpe esta ISR, así que el borde del tick es éste.
    g_adc.close_tick();

#if SENSE_DIAG
    g_angle.on_tick(g_dbg_i2c);
#else
    g_angle.on_tick();
#endif

    if (g_clock.on_isr())
    {
        g_angle.freeze();
        g_adc.close_row();
    }
}

// El ADC convierte de corrido, sin relación con el muestreador: cada conversión se
// suma a la fila en curso. Ver Sense/RowAdc.h.
ISR(ADC_vect)
{
    g_adc.on_conversion();
}

// ------------------------------------------------------------------- el paso

// Toma lo que la ISR congeló en el tick y pone el comando sobre el actuador, una
// vez por fila.
//
// El ángulo llega desenrollado desde la ISR. La corrección de la tabla se indexa con
// la cuenta cruda de adentro de la vuelta y se resta sobre lo desenrollado --a lo
// sumo 512 cuentas, sin vuelta de por medio--. El signo va al final: la tabla y el
// desenrollado hablan del imán, y el signo, del banco.
//
// La corriente es el promedio de todas las conversiones de las últimas `cur_filas`
// filas, de A0 y, con divisor, de A1, llevado a cuentas equivalentes; el cero se
// resta después, sobre el promedio. De ahí hasta el canal todo va en dieciseisavos de
// cuenta, sin volver a redondear. Ver Sense/CurrentSense.h.
static void step(void)
{
    uint16_t raw;
    int32_t  raw_uw;
    bool     fresh;
    g_angle.take(raw, raw_uw, fresh);

    uint32_t sum, supply_sum;
    uint16_t n, supply_n;
    g_adc.row(sum, n, supply_sum, supply_n);

#if SENSE_DIAG
    g_adc.raw_means(g_a0, g_a1, g_conv);
#endif

    g_current.push(sum, n, supply_sum, supply_n);

    g_y_raw = raw;
    g_y_rep = !fresh;
    const int32_t uw = raw_uw - (g_cal ? g_lut.correction((Lut::Counts)raw) : 0);
    g_y_uw = g_ang_inv ? -uw : uw;

    g_motor.write(g_uff);
}

// Aplica lo que la computadora acaba de escribir. Lo dispara el contador de
// escrituras de CtrlLink: una comparación de 16 bits por pasada de loop().
static void refresh_tuning(void)
{
    CtrlLink::set_period_us(g_clock.apply(SAMPLE_HZ));

    g_lut.apply(g_lutw);

    // Con la frecuencia de las filas de ahora: `loop_div` también mueve los notch.
    g_current.apply((float)SAMPLE_HZ / (float)g_clock.divide);
    g_adc.alternate(g_current.ratio.active());

    // Siempre: si la cadena del ADC quedó parada, esto es lo único que la levanta.
    g_adc.reconfigure();

#if SENSE_DIAG
    if (g_dbg_ocr >= 40)
    {
        OCR2A = g_dbg_ocr;
    }
#endif
}

// ------------------------------------------------------------------- Arduino

void setup()
{
    // El reloj antes que nada: si la placa no corre a 16 MHz, el UART emite al
    // ritmo equivocado y ni el mensaje de error llega. Después el ADC contra Vcc,
    // y el bus: grabar la placa la resetea, y un reset en medio de una lectura deja
    // al AS5600 sujetando SDA. Ver BoardStart.h.
    board::clock_begin();

    // millis() en fase fija con el muestreador. Ver BoardStart/BoardClock.h.
    board::millis_1000hz();

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

    // El conversor antes que refresh_tuning(), que lo reconfigura.
    g_adc.begin(adc_full);

    g_current.notch.harmonics = MAINS_HARMONICS;
    g_current.notch.nyquist   = 1;
    g_adc.ma                  = 1;
    refresh_tuning();

    Sensor::begin();
    g_clock.begin(SAMPLE_HZ);

    // El filtro del sensor, recién ahora: la lectura que precede a la escritura
    // viaja en un tick de muestreo. Unos pocos intentos, por si la primera muestra
    // todavía no salió; sin sensor en el bus se sigue igual.
    for (uint8_t i = 0; i < 10 && !Sensor::write_slow_filter(SENSOR_FILTER, g_clock); i++)
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

    // El montaje del imán, sólo entre capturas: cada lectura le cuesta al muestreo
    // dos muestras. Ver SensorHealth::refresh_mounting().
    if (!CtrlLink::streaming())
    {
        g_health.refresh_mounting<Sensor>(millis());
    }

    CtrlLink::poll();

    // Después de poll(): ver SampleClock::window().
    g_clock.window(CtrlLink::streaming());
}
