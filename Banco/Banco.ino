// El banco en lazo abierto: un comando de PWM que entra, y el ángulo y la
// corriente que salen. Gobernado desde un notebook de Jupyter por CtrlLink.
//
// Hardware:
// Arduino UNO
// Sensor de posición de efecto Hall: AS5600 (I2C)   SDA -> A4, SCL -> A5
// Medición de corriente (opcional): ACS712 en A0
// Actuador (opcional): puente L298N, ENA -> 9 (PWM, 1 kHz), IN1 -> 6, IN2 -> 7
//
// El Timer2 muestrea el AS5600 a 5 kHz; cada `tickdiv` muestras se emite una
// fila de telemetría, así que la frecuencia de las filas es 5000/tickdiv Hz y
// por omisión vale 500 Hz. El muestreo mantiene un período rígido aunque el
// resto fluctúe; `maxlate` informa cuánta fluctuación hubo y `missed` cuenta los
// períodos que se saltearon del todo.
//
// Acá no hay ley de control: `uff` va derecho al puente. Todo lo que se hace con
// la medición --derivar la velocidad, filtrar, ajustar un modelo-- pasa del lado
// de la computadora, donde se ve y se puede cambiar sin recompilar. Un filtro en
// la placa se confunde con la planta que se está midiendo, y por eso no hay
// ninguno.
//
// El puerto serie va a 1 Mbaud. En un AVR de 16 MHz ése es un divisor exacto
// (UBRR=1), a diferencia de 115200, que queda 2,1 % desviado. Los bytes
// entrantes llegan cada 10 us y el USART guarda sólo dos, así que entre el
// muestreador de 5 kHz y la interrupción de TWI se pierde un pequeño porcentaje
// de los bytes de un comando enviado de corrido; la computadora los espacia para
// compensarlo. Ver PROTOCOL.md.
//
// Periféricos de los que se apropia este sketch: el Timer2, así que analogWrite()
// en los pines 3 y 11 y tone() dejan de funcionar; el Timer1, que modula el
// puente con su propio TOP, así que analogWrite() en los pines 9 y 10 y Servo
// dejan de servir; y el ADC, que se maneja directamente acá, así que no hay que
// llamar a analogRead(). El Timer0 queda intacto: millis() y el PWM de los pines
// 5 y 6 andan como siempre.

#include <nI2C.h>

#include <AS5600.h>
#include <NI2CBus.h>
#include <BoardStart.h>
#include <CtrlLink.h>

typedef AS5600<NI2CBus> Sensor;

static const uint32_t BAUD           = 1000000;
static const uint16_t SAMPLE_HZ      = 5000;
static const int16_t  COUNTS_PER_REV = 4096;

// Actuador: un puente en H L298N. ENA lleva la magnitud por PWM y el par
// IN1/IN2 el sentido. Ese reparto deja toda la modulación en un solo pin y el
// sentido en dos salidas digitales comunes, y es lo que permite apagar el puente
// entero con una sola escritura; ver drive().
//
// Se puede correr sin nada conectado acá: los pines conmutan igual y todo lo
// demás, telemetría incluida, se comporta idéntico.
//
// ENA va al pin 9, modulado por el Timer1. El pin no es una preferencia: el
// Timer0 (pines 5 y 6) lleva millis() y el Timer2 (pines 3 y 11) es el
// muestreador, así que el único temporizador libre es el Timer1 y sus salidas son
// los pines 9 y 10. Este código habla con OC1A directamente, así que mudar ENA al
// pin 10 es cambiar acá y además OCR1A por OCR1B y COM1A1 por COM1B1.
static const uint8_t MOTOR_PWM_PIN = 9;     // ENA del L298N, OC1A
static const uint8_t MOTOR_IN1_PIN = 6;     // IN1
static const uint8_t MOTOR_IN2_PIN = 7;     // IN2

// El TOP del Timer1, phase-correct con preescalador 1: f = 16 MHz / (2 * TOP), o
// sea 1 kHz. Es lo que tolera un L298N alimentado con 5 V: es un puente de
// Darlington bipolares, cae unos 2 V entre sus dos lados y tarda unos 2 us en
// conmutar, así que a 20 kHz lo que se pierde en cada transición se lleva una
// fracción grande de un tiempo de encendido que ya venía escaso. Medido en este
// banco: a 20 kHz el motor no arranca y a 1 kHz anda. Con un puente MOSFET --un
// TB6612FNG, un DRV8833, que caen 0,3 V-- conviene subirla fuera del rango
// audible: 400 son 20 kHz.
//
// 1 kHz además entra dos veces justas en cada período de 500 Hz, así que el
// muestreador toma siempre las mismas fases de la ondulación de corriente: un
// sesgo fijo en `i` en lugar de un batido lento.
static const uint16_t PWM_TOP = 8000;

// El techo del comando. 255 no es negociable sin tocar pwm_write(), que
// aprovecha que U_MAX + 1 sea una potencia de dos para escalar con un
// corrimiento en vez de una división.
static const int16_t U_MAX = 255;

// Medición de corriente en A0. Nada de este bloque mueve el motor: sólo fija las
// unidades que se le informan a la computadora, así que equivocarlo mueve una
// etiqueta, no un lazo.
//
// La sensibilidad es lo único que convierte cuentas en amperes, y depende de
// cuál ACS712 esté puesto: 185 mV/A el de 5 A, 100 mV/A el de 20 A, 66 mV/A el
// de 30 A. El de 5 A es el que está pensado para este banco. OJO si no cierra con
// lo que mide un tester en serie con el motor: un reposo muy por debajo de los
// 2500 mV que da un ACS712 alimentado a 5 V delata un divisor en la salida, y un
// divisor divide las dos cosas a la vez --el cero y la sensibilidad--, así que
// ahí va la sensibilidad dividida por lo mismo.
static const uint8_t  SENSE_CHANNEL  = 0;
static const float    SENSE_MV_PER_A = 185.0f;

// El ADC lee A0 contra Vcc. Un ACS712 es un sensor bipolar y ratiométrico: reposa
// en la mitad de su alimentación --2,5 V con 5 V-- para poder bajar cuando la
// corriente cambia de sentido, y medirlo contra la misma tensión que lo alimenta
// deja ese reposo en media escala por construcción. Contra la referencia interna
// de 1,1 V, en cambio, satura en reposo y no mide nada.
//
// Lo que se paga es resolución: un LSB son 4,9 mV, y con 185 mV/A eso deja
// 200 mA en apenas siete u ocho cuentas. Es el precio de poder medir el sensor
// que está puesto. Un motor chico --decenas de mA en régimen-- queda por debajo
// de un escalón, y ahí el canal sirve para ver el arranque y comparar picos, no
// para leer miliamperes.
//
// Todo lo que sigue cuenta en 12 bits, en las dos placas del banco. El UNO tiene
// un ADC de 10 bits y el clon con LGT8F328P uno de 12, así que la misma tensión
// mide cuatro veces más en una que en la otra. La lectura del UNO se corre dos
// bits para arriba y las escalas de abajo son una sola; el corrimiento vive en
// g_adcshift. Vcc son los 5006 mV medidos con un tester en el UNO de este banco.
static const uint16_t ADC_FULL         = 4096;
static const int16_t  SENSE_ZERO       = ADC_FULL / 2;
static const float    ADC_REF_MV       = 5006.0f;
static const float    SENSE_MA_PER_LSB = 1000.0f * (ADC_REF_MV / ADC_FULL) / SENSE_MV_PER_A;

// El filtro lento del AS5600. Arranca en 16x, que son 2,2 ms de retardo; en 2x
// son 0,286 ms. Ese retardo se identifica después como si fuera del motor, así
// que se lo baja al mínimo en el arranque. Ver AS5600.h.
static const uint8_t SENSOR_FILTER = Sensor::SF_2X;

// ------------------------------------------------------------------ variables
// Acá vive todo lo que la computadora puede leer o escribir. Los canales los lee
// CtrlLink::emit() a través de sus direcciones, así que tienen que escribirse
// desde el mismo contexto que llama a emit(): loop(), no la ISR.

static int16_t g_uff     = 0;           // el comando, -U_MAX..U_MAX
static int16_t g_izero   = SENSE_ZERO;  // cero del sensor de corriente, en LSBs del ADC
static uint8_t g_tickdiv = 10;          // muestras de 5 kHz por fila: 10 -> 500 Hz

// Calibración del sensor: `cal` prende y apaga la corrección en caliente, que es
// lo que permite medir cuánto sirve en lugar de suponerlo. Ver más abajo.
static uint8_t g_cal = 0;

// Una entrada de la tabla por escritura, empaquetada como (índice << 16) | valor.
// El índice viaja adentro del mismo valor para que dos escrituras seguidas nunca
// sean iguales por casualidad: el sketch aplica la escritura al ver que este
// parámetro cambió, y con índice y valor en parámetros separados una tabla con
// dos entradas iguales seguidas perdería la segunda. 0xFFFFFFFF es "nada que
// hacer", porque ese índice no existe; de ahí sale el valor de arranque.
static uint32_t g_lutw   = 0xFFFFFFFFUL;
static uint16_t g_lutsum = 0;   // suma de Fletcher de la tabla; la computadora la verifica

static uint16_t g_y_raw = 0;    // cuenta cruda del sensor, 0..4095, sin corregir
static int16_t  g_y     = 0;    // ángulo dentro de la vuelta, cuentas, corregido
static int32_t  g_y_uw  = 0;    // ángulo desenrollado, cuentas
static int16_t  g_i     = 0;    // corriente, LSBs del ADC alrededor de izero
static int16_t  g_u     = 0;    // lo que salió al puente

// Contadores de salud. Todos los puede escribir la computadora, así que poner uno
// en cero reinicia esa cuenta.
static uint16_t g_maxlate = 0;  // peor retardo observado entre la ISR y su atención, us
static uint16_t g_missed  = 0;  // períodos que loop() nunca atendió
static uint16_t g_sovr    = 0;  // muestras del sensor que el bus I2C no llegó a seguir
static uint16_t g_serr    = 0;  // transferencias del sensor que fallaron
static uint8_t  g_spres   = 1;  // el sensor contesta en el bus
static uint8_t  g_mstat   = 0;  // registro STATUS del AS5600: imán presente, muy débil, muy fuerte

// La ISR los escribe, loop() los limpia.
static volatile bool     g_tick       = false;
static volatile uint32_t g_tick_us    = 0;
static volatile uint16_t g_missed_isr = 0;
static volatile int16_t  g_adc        = 0;   // última conversión completada de A0
static volatile uint8_t  g_divider    = 10;

static uint8_t g_adcshift = 0;   // cuánto se corre cada lectura para contar en 12 bits

// --------------------------------------------------------- calibración del AS5600

// El AS5600 no mide el ángulo que uno cree. Un imán descentrado respecto del
// integrado --la hoja de datos pide ±0,25 mm-- corre la lectura en una cantidad
// que depende del ángulo y que se repite vuelta tras vuelta: al derivar se
// presenta como una ondulación de velocidad enganchada a la vuelta, y parece del
// motor. Ver Docs/CALIBRACION_AS5600.md, que además explica cómo se lo mide sin
// tener un encoder de referencia.
//
// Acá vive nada más que la corrección: una tabla indexada por el ángulo crudo.
// Quién la calcula y de dónde sale es asunto de la computadora.
//
// La tabla NO se guarda en la placa. El dispositivo arranca siempre sin
// calibrar, y quien tiene la tabla es la computadora, que la empuja al conectarse
// --ver python/calib.py--: una calibración es una propiedad del banco --este
// imán, en este eje-- y no del programa, y una tabla vieja aplicándose en
// silencio es peor que ninguna. Para dejarla fija en un tablero que se enciende
// solo, `calib.escribir_header()` genera Calibracion.h y este sketch lo toma si
// está.
static const uint8_t LUT_SIZE = 64;

// Entradas en octavos de cuenta, int16: hasta ±511 cuentas, ±45 grados. La
// resolución de un octavo existe porque el error puede ser de unas pocas
// cuentas; el rango, porque medido en este banco el segundo armónico llegó a 105
// cuentas con el imán a la distancia correcta.
static const int16_t LUT_MAX = 4095;

static int16_t g_lut[LUT_SIZE];

#if defined(__has_include)
#  if __has_include("Calibracion.h")
#    include "Calibracion.h"
#    define TIENE_CALIBRACION 1
#  endif
#endif

// Suma de Fletcher de 16 bits sobre la tabla, byte por byte, primero el bajo y
// después el alto de cada entrada. La computadora la calcula por su lado y la
// compara con `lutsum`, así que las 64 escrituras se verifican con una sola
// lectura. Fletcher y no una suma pelada porque una suma no distingue una tabla
// de otra con dos entradas intercambiadas.
static uint16_t lut_checksum(void)
{
    uint8_t a = 0;
    uint8_t b = 0;

    for (uint8_t i = 0; i < LUT_SIZE; i++)
    {
        uint16_t v = (uint16_t)g_lut[i];

        a += (uint8_t)v;
        b += a;
        a += (uint8_t)(v >> 8);
        b += a;
    }

    return ((uint16_t)b << 8) | a;
}

// Corrección en cuentas para un ángulo crudo, interpolada linealmente entre las
// dos entradas que lo rodean. 64 entradas son 64 cuentas --5,6 grados-- de
// separación; la interpolación se hace cargo del resto.
static int16_t lut_lookup(int16_t counts)
{
    uint8_t i    = (uint8_t)(counts >> 6) & (LUT_SIZE - 1);
    uint8_t frac = (uint8_t)counts & 0x3F;

    int32_t a = (int32_t)g_lut[i];
    int32_t b = (int32_t)g_lut[(uint8_t)(i + 1) & (LUT_SIZE - 1)];

    // Un int32 en el medio: con entradas de hasta ±4095 octavos la suma llega a
    // 262080, que no entra en 16 bits.
    int32_t eighths = (a * (int32_t)(64 - frac) + b * (int32_t)frac) >> 6;

    // Redondeo al medio hacia arriba, válido para los dos signos con corrimiento
    // aritmético.
    return (int16_t)((eighths + 4) >> 3);
}

// --------------------------------------------------------------------- tablas

static const CtrlParam PROGMEM g_params[] =
{
    { "uff",     CTRL_I16, &g_uff,     0 },
    { "izero",   CTRL_I16, &g_izero,   0 },
    { "tickdiv", CTRL_U8,  &g_tickdiv, 0 },
    { "cal",     CTRL_U8,  &g_cal,     0 },
    { "lutw",    CTRL_U32, &g_lutw,    0 },
    { "lutsum",  CTRL_U16, &g_lutsum,  0 },
    { "spres",   CTRL_U8,  &g_spres,   0 },
    { "mstat",   CTRL_U8,  &g_mstat,   0 },
    { "maxlate", CTRL_U16, &g_maxlate, 0 },
    { "missed",  CTRL_U16, &g_missed,  0 },
    { "sovr",    CTRL_U16, &g_sovr,    0 },
    { "serr",    CTRL_U16, &g_serr,    0 },
};

static const float COUNTS_TO_DEG = 360.0f / COUNTS_PER_REV;

static const CtrlChannel PROGMEM g_channels[] =
{
    // La cuenta cruda, sin corregir. Es lo que indexa la tabla de calibración,
    // así que es lo que la computadora necesita para calcularla.
    { "y_raw", CTRL_U16, &g_y_raw, COUNTS_TO_DEG,    "deg" },
    { "y_uw",  CTRL_I32, &g_y_uw,  COUNTS_TO_DEG,    "deg" },
    { "u",     CTRL_I16, &g_u,     1.0f,             "pwm" },
    { "i",     CTRL_I16, &g_i,     SENSE_MA_PER_LSB, "mA"  },
};

// ----------------------------------------------------------------- temporizador

// Timer2, CTC, preescalador 32: 16 MHz / 32 / 100 = exactamente 5,000 kHz.
static void startSampleTimer(void)
{
    TCCR2A = _BV(WGM21);                // CTC, TOP = OCR2A
    TCCR2B = _BV(CS21) | _BV(CS20);     // preescalador /32
    OCR2A = 99;
    TCNT2 = 0;
    TIMSK2 = _BV(OCIE2A);
}

// Timer1, phase-correct con TOP = ICR1, preescalador 1. Se arranca con la salida
// de comparación desconectada, que es el puente abierto: la conecta pwm_write()
// cuando hay algo que accionar.
static void startMotorPwm(void)
{
    TCCR1A = _BV(WGM11);                    // modo 10: phase-correct, TOP = ICR1
    TCCR1B = _BV(WGM13) | _BV(CS10);        // preescalador /1
    TCNT1  = 0;
    ICR1   = PWM_TOP;
}

// ------------------------------------------------------------------------ adc

// En el LGT8F328P los bits REFS del ADMUX no eligen la referencia: la eligen
// DACON y el bit REFS2 de ADCSRD, y un sketch que escriba sólo REFS deja la que
// haya quedado de antes. Las dos placas del banco corren el mismo binario con el
// core del ATmega, así que esos registros van por dirección, y sólo se los toca
// cuando la placa es la del ADC de 12 bits: en el UNO 0xA0 y 0xAD no son
// registros. Se pide DEFAULT, que es Vcc, lo mismo que REFS=01 en el UNO.
static const uint16_t LGT_DACON  = 0xA0;
static const uint16_t LGT_ADCSRD = 0xAD;
static const uint8_t  LGT_REFS2  = 6;

// El ADC se maneja directamente desde el muestreador en lugar de a través de
// analogRead(), que espera activamente a que termine la conversión. Una
// conversión con preescalador /128 tarda 104 us, así que entra en un período de
// muestreo de 200 us: la ISR recoge el resultado que arrancó el tick anterior e
// inmediatamente lanza el siguiente. El costo es un período de muestreo de
// retardo en `i`.
static void startAdc(void)
{
    if (g_adcshift == 0)
    {
        _SFR_MEM8(LGT_ADCSRD) &= (uint8_t)~_BV(LGT_REFS2);
        _SFR_MEM8(LGT_DACON)  &= 0x0C;
    }

    ADMUX  = _BV(REFS0) | (SENSE_CHANNEL & 0x07);
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADSC);
}

ISR(TIMER2_COMPA_vect)
{
    static uint8_t count = 0;

    Sensor::do_transfer();

    if (ADCSRA & _BV(ADIF))
    {
        g_adc = (int16_t)(ADC << g_adcshift);
        // Escribir un 1 en ADIF lo borra; ese mismo almacenamiento lanza la
        // conversión siguiente, así que el ADC corre libre un resultado por
        // detrás del muestreador.
        ADCSRA |= _BV(ADIF) | _BV(ADSC);
    }

    if (++count < g_divider)
    {
        return;
    }
    count = 0;

    if (g_tick)
    {
        // loop() no atendió el tick anterior: el período se está perdiendo del
        // todo, que es peor que una simple fluctuación.
        g_missed_isr++;
    }

    g_tick_us = micros();
    g_tick    = true;
}

// ------------------------------------------------------------------- el puente

// Desconecta la salida de comparación del pin, que vuelve a ser una salida común
// con su bit de PORT en bajo desde setup(): ENA queda en bajo y el puente abierto,
// en el ciclo en el que se pide y no al final del período de PWM.
static inline void pwm_off(void)
{
    TCCR1A &= ~_BV(COM1A1);
}

// `mag` va de 0 a U_MAX y el temporizador cuenta hasta PWM_TOP, que es otra
// escala. Se divide por U_MAX + 1 = 256 en lugar de por 255, que es un corrimiento
// en vez de una división; el extremo de arriba --U_MAX tiene que ser encendido
// permanente y no 255/256 de él-- se atiende aparte. OCR1A está doblemente
// amortiguado, así que el valor nuevo entra al terminar el período en curso y
// ningún pulso sale cortado por la mitad.
static inline void pwm_write(int16_t mag)
{
    if (mag <= 0)
    {
        pwm_off();
        return;
    }

    OCR1A = (mag >= U_MAX) ? PWM_TOP
                           : (uint16_t)(((uint32_t)mag * PWM_TOP) >> 8);

    TCCR1A |= _BV(COM1A1);
}

// Pone `u` sobre el puente: la magnitud en ENA por PWM, el sentido en IN1/IN2.
//
// Un cambio de sentido no escribe las entradas de sentido con el puente vivo.
// Primero baja ENA, que apaga las cuatro llaves de una sola escritura; recién
// entonces mueve IN1 e IN2, y pasa por el estado con las dos en bajo antes de
// levantar la que corresponde. Así el cambio de sentido no atraviesa ningún
// estado conduciendo. Lo que ningún orden de escrituras arregla es la corriente
// que ya circula por el motor al invertir: sale por los diodos del puente contra
// la fuente, durante la constante L/R del motor.
//
// `u == 0` deja las entradas de sentido donde estaban: con ENA en cero el puente
// ya está abierto y el motor en punto muerto.
//
// El signo del comando es el del puente, no el del ángulo: si un comando positivo
// hace bajar el ángulo medido, eso es de qué lado están los cables del motor y
// de qué lado mira el imán, y se arregla en la computadora con un signo, o en la
// mesa dando vuelta dos cables.
static void drive(int16_t u)
{
    static int8_t dir = 0;

    int8_t sign = (u > 0) ? 1 : ((u < 0) ? -1 : 0);
    int8_t want = sign ? sign : dir;

    if (want != dir)
    {
        pwm_off();                              // ENA: puente abierto
        digitalWrite(MOTOR_IN1_PIN, LOW);
        digitalWrite(MOTOR_IN2_PIN, LOW);
        digitalWrite((want > 0) ? MOTOR_IN1_PIN : MOTOR_IN2_PIN, HIGH);
        dir = want;
    }

    pwm_write((u >= 0) ? u : (int16_t)-u);
}

// ------------------------------------------------------------------ medición

// El camino más corto de a hasta b en una circunferencia de 4096 cuentas.
static int16_t wrapped_delta(int16_t a, int16_t b)
{
    return (int16_t)(((a - b + 2048) & 0x0FFF) - 2048);
}

// Lee los sensores y actualiza todas las variables medidas. Corre una vez por
// período, y `u` sale al puente en el mismo paso.
static void paso(void)
{
    int16_t adc;
    noInterrupts();
    adc = g_adc;
    interrupts();

    // Crudo: sin filtrar. Filtrar acá metería un polo que después se identifica
    // como si fuera del motor.
    g_i = (int16_t)(adc - g_izero);

    // La corrección se aplica sobre la cuenta cruda y antes de desenrollar,
    // porque la tabla se indexa con el ángulo dentro de la vuelta.
    int16_t counts = (int16_t)Sensor::counts();
    g_y_raw = (uint16_t)counts;

    if (g_cal)
    {
        counts = (int16_t)((counts - lut_lookup(counts)) & (COUNTS_PER_REV - 1));
    }

    g_y_uw += wrapped_delta(counts, g_y);
    g_y     = counts;

    g_u = (g_uff > U_MAX) ? U_MAX : ((g_uff < -U_MAX) ? (int16_t)-U_MAX : g_uff);
    drive(g_u);
}

// Los contadores propios del sensor corren libres y no se pueden borrar, así que
// lo que se publica es el total acumulado de sus incrementos. Eso es lo que hace
// que `sovr` y `serr` los pueda escribir la computadora igual que el resto.
static void collect_sensor_health(void)
{
    static uint16_t last_overruns = 0;
    static uint16_t last_errors   = 0;

    uint16_t overruns = Sensor::overruns();
    uint16_t errors   = Sensor::errors();

    g_sovr += (uint16_t)(overruns - last_overruns);
    g_serr += (uint16_t)(errors   - last_errors);

    last_overruns = overruns;
    last_errors   = errors;

    g_spres = Sensor::present() ? 1 : 0;
}

// --------------------------------------------------------------------- ajuste

// Aplica lo que la computadora acaba de escribir. Lo dispara el contador de
// escrituras de CtrlLink: una comparación de 16 bits por pasada de loop(), en
// lugar de vigilar parámetro por parámetro.
static void refresh_tuning(void)
{
    if (g_tickdiv == 0)
    {
        g_tickdiv = 1;
    }

    // El divisor que usa la ISR y el período que se le informa a la computadora
    // cambian juntos, así que ninguno de los dos puede quedar describiendo una
    // frecuencia a la que las filas no están saliendo.
    g_divider = g_tickdiv;
    CtrlLink::set_period_us((uint32_t)g_tickdiv * 1000000UL / SAMPLE_HZ);

    // Una entrada de la tabla de calibración por escritura de `lutw`. Aplicar al
    // ver que el parámetro cambió es lo que permite cargar la tabla sin
    // agregarle un comando al protocolo: son 64 `set` comunes.
    static uint32_t lutw_applied = 0xFFFFFFFFUL;

    if (g_lutw != lutw_applied)
    {
        lutw_applied = g_lutw;

        uint16_t index = (uint16_t)(g_lutw >> 16);
        int16_t  value = (int16_t)(uint16_t)g_lutw;

        if (index < LUT_SIZE)
        {
            if (value >  LUT_MAX) value =  LUT_MAX;
            if (value < -LUT_MAX) value = -LUT_MAX;

            g_lut[index] = value;
        }
    }

    // Se recalcula siempre y no sólo al escribir la tabla: así `lutsum` describe
    // lo que hay, y la computadora verifica 64 entradas con una sola lectura.
    g_lutsum = lut_checksum();
}

// Escribe los bits SF del CONF del sensor. Lee-modifica-escribe, porque CONF
// también lleva la histéresis, el modo de potencia y la salida.
//
// La lectura viaja en un tick del muestreador, así que hace falta que ya esté
// corriendo. La escritura no: nI2C la encola y reserva memoria al hacerlo, y el
// muestreador llama a nI2C desde una ISR de temporizador, así que se lo para un
// par de milisegundos para que nadie más pida el bus. Devuelve false si el sensor
// no contestó, y entonces se vuelve a intentar.
static bool apply_sensor_filter(void)
{
    uint8_t conf[2];

    if (!Sensor::read_registers(Sensor::REG_CONF_H, conf, 2))
    {
        return false;
    }

    uint16_t value = ((uint16_t)conf[0] << 8) | conf[1];
    value = (uint16_t)((value & ~0x0300u) | ((uint16_t)(SENSOR_FILTER & 0x03) << 8));

    conf[0] = (uint8_t)(value >> 8);
    conf[1] = (uint8_t)value;

    TIMSK2 &= ~_BV(OCIE2A);

    uint32_t deadline = millis() + 5;
    while (Sensor::busy() && (int32_t)(millis() - deadline) < 0)
    {
    }

    bool queued = Sensor::write_registers(Sensor::REG_CONF_H, conf, 2);

    delay(2);   // cuatro bytes a 400 kHz son unos 100 us; esto es holgura

    TIMSK2 |= _BV(OCIE2A);

    return queued;
}

// La visión que el propio AS5600 tiene del imán: detectado, muy débil, muy
// fuerte. Leerla le cuesta al muestreador una muestra y bloquea acá hasta que
// llegue, así que sólo se hace entre capturas.
static void refresh_magnet_status(void)
{
    static uint32_t last_ms = 0;

    if (CtrlLink::streaming() || (millis() - last_ms) < 500u)
    {
        return;
    }
    last_ms = millis();

    if (!Sensor::present())
    {
        g_mstat = 0;
        return;
    }

    uint8_t status;
    if (Sensor::read_status(status))
    {
        g_mstat = status;
    }
}

// Los contadores de salud describen la ventana de emisión, así que se ponen en
// cero cuando se abre una. Arrancarla cuesta unos milisegundos de puerto serie
// --el encabezado son varias líneas, y escribir bloquea en cuanto se llena el
// buffer--, y los períodos que se pierden ahí son el precio de arrancar la
// captura, no una falla del banco.
static void reset_health_on_capture(void)
{
    static bool was_streaming = false;

    bool now = CtrlLink::streaming();

    if (now && !was_streaming)
    {
        noInterrupts();
        g_missed_isr = 0;
        interrupts();

        g_missed  = 0;
        g_maxlate = 0;
    }

    was_streaming = now;
}

// ------------------------------------------------------------------- Arduino

void setup()
{
    // El reloj antes que nada: si la placa no corre a 16 MHz, el UART emite al
    // ritmo equivocado y ni el mensaje de error llega. Y el bus enseguida
    // después: grabar la placa la resetea, y un reset en medio de una lectura
    // deja al AS5600 sujetando SDA, con lo que el muestreador arranca trabado
    // para siempre. Ver BoardStart.h.
    boardClockBegin();
    g_adcshift = (boardAdcFullScale() >= ADC_FULL) ? 0 : 2;
    i2cBusRecover();

    // ENA primero: mientras el puente esté abierto las entradas de sentido no
    // gobiernan nada, así que ése es el orden en el que ningún estado intermedio
    // acciona el motor.
    startMotorPwm();
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    digitalWrite(MOTOR_PWM_PIN, LOW);
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    digitalWrite(MOTOR_IN1_PIN, LOW);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    digitalWrite(MOTOR_IN2_PIN, LOW);

    // Si el proyecto trae una calibración compilada, entra acá y queda activa
    // desde el arranque. Sin ella la tabla es toda ceros y `cal` arranca en 0.
#ifdef TIENE_CALIBRACION
    memcpy_P(g_lut, CAL_LUT, sizeof(g_lut));
    g_cal = 1;
#endif

    CtrlLink::set_id(F("Banco"));
    CtrlLink::begin(BAUD,
                    g_params,   sizeof(g_params)   / sizeof(g_params[0]),
                    g_channels, sizeof(g_channels) / sizeof(g_channels[0]),
                    (uint32_t)g_tickdiv * 1000000UL / SAMPLE_HZ);

    refresh_tuning();

    startAdc();
    Sensor::begin();
    startSampleTimer();

    // El filtro del sensor, recién ahora: la lectura que precede a la escritura
    // viaja en un tick de muestreo. Unos pocos intentos, por si la primera
    // muestra todavía no salió; sin sensor en el bus se sigue igual.
    for (uint8_t i = 0; i < 10 && !apply_sensor_filter(); i++)
    {
        delay(2);
    }

    CtrlLink::note(F("Banco listo"));
}

void loop()
{
    static uint16_t last_writes = 0;

    if (g_tick)
    {
        uint32_t fired;
        uint16_t missed;

        // La ISR puede caer entre las dos mitades de una lectura de 32 bits.
        noInterrupts();
        fired        = g_tick_us;
        missed       = g_missed_isr;
        g_missed_isr = 0;
        g_tick       = false;
        interrupts();

        g_missed += missed;

        uint16_t late = (uint16_t)(micros() - fired);
        if (late > g_maxlate)
        {
            g_maxlate = late;
        }

        paso();
        collect_sensor_health();

        CtrlLink::emit();
    }

    // Después del paso, nunca antes: un `set` que caiga justo cuando se dispara
    // un tick quedaría de otro modo por delante de él.
    uint16_t writes = CtrlLink::writes();
    if (writes != last_writes)
    {
        last_writes = writes;
        refresh_tuning();
    }

    refresh_magnet_status();

    CtrlLink::poll();

    // Después de poll(), que es donde se atiende `start` y se imprime el
    // encabezado: así la ventana empieza a contar recién cuando ya salió.
    reset_health_on_capture();
}
