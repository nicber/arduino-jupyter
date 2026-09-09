// Lazo de control de posición sobre CtrlLink, gobernado desde un notebook de
// Jupyter.
//
// Hardware:
// Arduino UNO
// Sensor de posición de efecto Hall: AS5600 (I2C)   SDA -> A4, SCL -> A5
// Medición de corriente (opcional): ACS712 en A0
// Actuador (opcional): puente L298N, ENA -> 9 (PWM, 1 kHz), IN1 -> 6, IN2 -> 7
//
// El Timer2 muestrea el AS5600 a 5 kHz; cada `tickdiv` muestras se ejecuta la
// ley de control, así que la frecuencia del lazo es 5000/tickdiv Hz y por
// omisión vale 1 kHz. De esa manera el muestreo mantiene un período rígido aun
// cuando el cálculo de control fluctúe. `maxlate` informa cuánta fluctuación
// hubo y `missed` cuenta los períodos de control que se saltearon del todo.
//
// La ley de control es aritmética entera de punta a punta; ver ControlMath. Y
// también lo es cada parámetro que lee: cada uno se guarda en la forma de punto
// fijo que la aritmética necesita, y la tabla de parámetros declara la escala
// que lo convierte. La computadora multiplica a la ida y divide a la vuelta, así
// que el alumno sigue escribiendo `dev.kp = 0.5` y este sketch no ejecuta una
// sola instrucción de punto flotante.
//
// El puerto serie va a 1 Mbaud. En un AVR de 16 MHz ése es un divisor exacto
// (UBRR=1), a diferencia de 115200, que queda 2,1 % desviado. La telemetría
// sostiene esa velocidad con comodidad, pero los bytes entrantes llegan cada
// 10 us y el USART guarda sólo dos, así que entre el muestreador de 5 kHz y la
// interrupción de TWI de nI2C se pierde un pequeño porcentaje de los bytes de un
// comando enviado de corrido. La computadora espacia los bytes de comando para
// compensarlo; ver PROTOCOL.md. Los comandos son raros y diminutos, así que eso
// no cuesta nada.
//
// La tabla de canales por omisión son 41 bytes por fila, o el 41 % del enlace a
// 1 kHz. Es más que el "bastante por debajo de la mitad" que le gusta a este
// protocolo; conviene subir `dec` para corridas largas, o sacar un canal.
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
#include <CtrlLink.h>
#include <FixedPoint.h>
#include <FirstOrderFilter.h>

typedef AS5600<NI2CBus> Sensor;

static const uint32_t BAUD           = 1000000;
static const uint16_t SAMPLE_HZ      = 5000;
static const int16_t  COUNTS_PER_REV = 4096;

// Actuador: un puente en H L298N. ENA lleva la magnitud por PWM y el par
// IN1/IN2 el sentido. Ese reparto deja toda la modulación en un solo pin y el
// sentido en dos salidas digitales comunes, y es lo que permite apagar el puente
// entero con una sola escritura; ver drive().
//
// Se puede correr el lazo sin nada conectado acá: los pines conmutan igual y
// todo lo demás, telemetría incluida, se comporta idéntico.
//
// ENA va al pin 9, modulado por el Timer1; ver startMotorPwm(). El pin no es una
// preferencia: el Timer0 (pines 5 y 6) lleva millis() y no se le puede tocar el
// preescalador, y el Timer2 (pines 3 y 11) es el muestreador de 5 kHz, así que el
// único temporizador que queda libre es el Timer1 y sus salidas son los pines 9 y
// 10. Este código habla con OC1A directamente, así que mudar ENA al pin 10 es
// cambiar acá y además OCR1A por OCR1B y COM1A1 por COM1B1.
//
// IN1 e IN2 son salidas digitales comunes y pueden ir a cualquier pin.
static const uint8_t MOTOR_PWM_PIN = 9;     // ENA del L298N, OC1A
static const uint8_t MOTOR_IN1_PIN = 6;     // IN1
static const uint8_t MOTOR_IN2_PIN = 7;     // IN2

// El TOP del Timer1, que es la forma en que se guarda la frecuencia del PWM:
//
//     f = 16 MHz / (2 * pwmtop)
//
// Se guarda el TOP y no los Hz por la misma razón por la que `tickdiv` se guarda
// como divisor y no como frecuencia de lazo: es por lo que cuenta el hardware, es
// exacto, y la conversión la hace la computadora, que es el lado que tiene la
// aritmética para hacerla. `dev.pwm(20000)` del lado del notebook.
//
// El piso son 255. Por debajo de ese TOP el ciclo de trabajo tendría menos
// escalones que el comando, así que `u` dejaría de ser fiel; y 31,4 kHz, que es
// lo que ese piso significa, ya está bastante más arriba de lo que le conviene a
// un puente de Darlington bipolares. El techo lo pone el propio uint16: 65535 son
// 122 Hz, lo bastante lento como para ver la ondulación con los ojos.
static const uint16_t PWM_TOP_MIN     = 255;    // 31,4 kHz
static const uint16_t PWM_TOP_DEFAULT = 8000;   //  1,0 kHz

// El techo del comando, y el único de los dos extremos que es constante. El piso
// lo decide el parámetro `bidir` en tiempo de ejecución --ver g_u_min--: un puente
// que acciona en los dos sentidos recorta en -U_MAX, y uno cableado para un solo
// cuadrante recorta en cero, que es lo que hay que informarle a la lógica
// anti-windup para que no cargue el integrador contra un límite que no existe.
// Es parámetro y no #define porque es una propiedad del banco y no del programa:
// se contesta desde el notebook y sin recompilar.
//
// 255 no es negociable sin tocar pwm_write(), que aprovecha que U_MAX + 1 sea una
// potencia de dos para escalar con un corrimiento en vez de una división.
static const int16_t U_MAX = 255;

// Medición de corriente en A0. Nada de este bloque entra en la ley de control:
// sólo fija las unidades que se le informan a la computadora, así que equivocarlo
// mueve una etiqueta, no un lazo.
//
// `i` se lee como `adc - izero`, así que crece cuando crece la tensión que entrega
// el sensor. Cuál de los dos sentidos de giro sale positivo depende de dónde esté
// insertado el sensor, y para un sensor unipolar --en la alimentación del puente--
// los dos salen positivos. Eso es una propiedad del banco, no de la aritmética.
static const uint8_t  SENSE_CHANNEL   = 0;

// La sensibilidad del sensor, que es lo único que convierte cuentas en amperes.
// 185 mV/A es un ACS712-05B conectado directo. OJO si no cierra con lo que mide
// un tester en serie con el motor: en este banco el reposo está en 489 mV y no en
// los 2500 que da un ACS712 alimentado a 5 V, y un divisor de ~5:1 en la salida
// explicaría las dos cosas a la vez --el cero corrido y la sensibilidad chica--,
// en cuyo caso acá va 36 y no 185. Ver README.
static const float    SENSE_MV_PER_A  = 185.0f;

// La referencia del ADC, que es la única perilla de ganancia que tiene el AVR de
// este lado. Con AVcc un LSB son 4,9 mV; con la referencia interna de 1,1 V son
// 1,07 mV, o sea 4,5 veces más resolución sobre la misma señal. Para un motor
// chico eso es la diferencia entre medir y no medir: un ACS712-05B da 185 mV/A, así
// que 200 mA son 37 mV, que contra AVcc es apenas un escalón de cuantización.
//
// El precio es el techo: la entrada no puede pasar de la referencia sin recortar.
// Con la interna, el reposo del sensor tiene que caer por debajo de 1,1 V, lo que
// descarta un ACS712 alimentado a 5 V --reposa en 2,5 V y quedaría fuera de escala
// desde el vamos-- y sirve para uno unipolar, que reposa cerca de cero. Poner
// SENSE_REF_INTERNAL en false vuelve a AVcc, que admite cualquiera de los dos y
// mide los dos mal.
//
// SENSE_ZERO es sólo el punto de partida de `izero`; el cero de verdad lo mide la
// computadora. Ver Bench.zero_current().
//
// La interna no vale 1,100 V: el bandgap está especificado entre 1,0 y 1,2 V, o
// sea +/-10 % de error de ganancia de chip a chip. Se mide sin instrumental,
// leyendo el canal 14 del multiplexor contra AVcc: en esta placa dio 1093 mV, y
// AVcc 5006. Cambiar el número al que mida la placa de uno; el error va derecho a
// los mA que se informan.
static const bool     SENSE_REF_INTERNAL = true;
static const float    ADC_REF_MV      = SENSE_REF_INTERNAL ? 1093.0f : 5006.0f;
static const int16_t  SENSE_ZERO      = SENSE_REF_INTERNAL ? 0 : 512;
static const float    ADC_MV_PER_LSB  = ADC_REF_MV / 1024.0f;
static const float    SENSE_MA_PER_LSB = 1000.0f * ADC_MV_PER_LSB / SENSE_MV_PER_A;

// `ref` y `refrate` llevan 8 bits fraccionarios, así que una rampa puede avanzar
// menos de una cuenta por período sin que la cuantización la anule.
static const uint8_t  REF_FRAC = 8;

// Son dos elecciones independientes, y vale la pena mantenerlas separadas.
//
// `mode` elige el controlador: la ley que convierte un error en un comando.
// `target` elige la realimentación: sobre qué magnitud medida cierra esa ley.
// Un controlador es una función y un caso en control_step(); una realimentación
// es una rama en target_error(). Ninguno de los dos sabe del otro.
enum : uint8_t
{
    MODE_OPEN = 0,      // u = uff, controlador puenteado
    MODE_PID  = 1,      // PID sobre la magnitud seleccionada
    MODE_RAMP = 2,      // PID, con ref avanzando refrate por período
};

// `ref >> REF_FRAC` está siempre en las unidades crudas de la magnitud
// seleccionada: cuentas para TARGET_POSITION, LSBs del ADC para TARGET_CURRENT.
enum : uint8_t
{
    TARGET_POSITION = 0,
    TARGET_CURRENT  = 1,
};

// Escalas de punto fijo, elegidas según el rango que realmente necesita cada
// magnitud. Ver FixedPoint.h; el compromiso es magnitud contra resolución.
//
// Las ganancias son por muestra, no por segundo: u = kp*e + ki*sum(e) + kd*diff(e),
// sin ningún dt en el medio. Eso es lo que hace la aritmética, así que eso es lo
// que significa el parámetro, y así toda escala queda como constante de tiempo de
// compilación que se le puede informar a la computadora. Una computadora que
// prefiera ganancias en tiempo continuo multiplica por dt de su lado; y cuando
// `tickdiv` cambia, que el efecto de los mismos tres números cambie con él es la
// lección, no un error.
//
// La tabla de parámetros publica FRAC directamente desde estos tipos, así que a
// la computadora se le informa cada formato desde la declaración que lo define y
// no desde una constante que hay que mantener sincronizada con ella.
typedef Fixed<int32_t, 22> Kp;      // +/-511,   resolución 2.4e-7
typedef Fixed<int32_t, 30> Ki;      // +/-1.99,  resolución 9.3e-10
typedef Fixed<int32_t, 16> Kd;      // +/-32767, resolución 1.5e-5
typedef FirstOrderFilter<4>::Alpha Alpha;   // Q16, una fracción en [0, 1]

// ------------------------------------------------------------------ variables
// Acá vive todo lo que la computadora puede leer o escribir. Los canales los lee
// CtrlLink::emit() a través de sus direcciones, así que tienen que escribirse
// desde el mismo contexto que llama a emit(): loop(), no la ISR.

// Ganancias, en la forma de punto fijo que usan los controladores. La
// computadora las fija en unidades naturales y el formato declarado en la tabla
// de parámetros hace la conversión. Por omisión es un lazo proporcional suave: el
// error está en cuentas, así que una ganancia que parece chica no lo es, y hacen
// falta 4096 cuentas para una vuelta.
static int32_t g_kp = Kp::from_float(0.002f).raw();
static int32_t g_ki = 0;
static int32_t g_kd = 0;

// Polos de los filtros: alpha = dt / (tau + dt), una fracción en [0, 1].
// alpha = 1 deja pasar la señal tal cual, que es la manera de apagar un filtro;
// así, alpha_y = 1 alimenta al lazo de posición con la cuenta cruda. Una
// computadora que piense en constantes de tiempo hace la conversión, porque es
// el lado que conoce dt y tiene la aritmética para hacerla.
static int32_t g_alpha_y = Alpha::from_int(1).raw();            // posición, dos polos
static int32_t g_alpha_i = Alpha::from_float(0.1667f).raw();    // corriente, dos polos
static int32_t g_alpha_e = Alpha::from_float(0.0909f).raw();    // error, un polo

static int32_t g_ref     = 0;   // referencia, unidades del target << REF_FRAC
static int32_t g_refrate = 0;   // pendiente de rampa, mismas unidades por período
static int16_t g_uff     = 0;   // comando prealimentado / de lazo abierto
static int16_t g_offset  = 0;   // cero del sensor de ángulo, en cuentas
static int16_t g_izero   = SENSE_ZERO;  // cero del sensor de corriente, en LSBs del ADC
static uint8_t g_target  = TARGET_POSITION;
static uint8_t g_mode    = MODE_OPEN;
static uint8_t g_tickdiv = 5;   // muestras de 5 kHz por período de control: 5 -> 1 kHz
static uint8_t g_bidir   = 1;   // 1: el puente acciona en los dos sentidos
static uint8_t g_uinvert = 1;   // 1: un comando positivo hace bajar el ángulo medido

static uint16_t g_pwmtop = PWM_TOP_DEFAULT;   // TOP del Timer1: f = 8 MHz / pwmtop

static int16_t g_y     = 0;     // ángulo medido, cuentas, con el offset aplicado
static int32_t g_y_uw  = 0;     // ángulo desenrollado, cuentas, sin filtrar
static int32_t g_y_uwf = 0;     // ángulo desenrollado, cuentas, filtrado por alpha_y
static int16_t g_i     = 0;     // corriente, LSBs del ADC alrededor de izero, filtrada
static int16_t g_e     = 0;     // error, unidades del target, recortado para la telemetría
static int16_t g_u     = 0;     // comando al actuador, u_min..U_MAX

// Contadores de salud. Todos los puede escribir la computadora, así que poner uno
// en cero reinicia esa cuenta.
static uint16_t g_maxlate = 0;  // peor retardo observado entre la ISR y su atención, us
static uint16_t g_missed  = 0;  // períodos de control que loop() nunca atendió
static uint16_t g_sovr    = 0;  // muestras del sensor que el bus I2C no llegó a seguir
static uint16_t g_serr    = 0;  // transferencias del sensor que fallaron
static uint8_t  g_mstat   = 0;  // registro STATUS del AS5600: imán presente, muy débil, muy fuerte
static uint8_t  g_spres   = 1;  // el sensor contesta en el bus

// Estado del integrador, en unidades de error sumadas a lo largo de los ticks.
// Guardar la suma cruda y aplicar ki*dt una sola vez al final es lo que permite
// que sobreviva una ganancia de 5e-5: la cuantización cae sobre la ganancia,
// donde es una fracción de un por ciento, en lugar de caer sobre la acumulación,
// donde se truncaría a cero en cada período.
static int32_t g_integral = 0;
static int32_t g_e_filt   = 0;
static int32_t g_e_prev   = 0;

// Las magnitudes derivadas, recalculadas por refresh_tuning().
static int32_t g_integral_max = INT32_MAX / 2;
static int16_t g_u_min        = -U_MAX;

static FirstOrderFilter<4> g_y_filt[2];   // posición: cuenta libre, necesita el margen
static FirstOrderFilter<8> g_i_filt[2];   // corriente: señal chica, quiere la resolución
static FirstOrderFilter<8> g_err_filt;

// La ISR los escribe, loop() los limpia. `g_tick_us` es el instante en que se
// disparó el tick, de modo que el retardo de atención quede visible para el lazo
// que lo levanta.
static volatile bool     g_tick       = false;
static volatile uint32_t g_tick_us    = 0;
static volatile uint16_t g_missed_isr = 0;
static volatile int16_t  g_adc        = 0;   // última conversión completada de A0
static volatile uint8_t  g_divider    = 5;

// --------------------------------------------------------------------- tablas

// Cada entrada se guarda exactamente como la quiere la aritmética; la columna de
// escala es lo que le permite a la computadora seguir hablando en unidades
// naturales.
static const CtrlParam PROGMEM g_params[] =
{
    { "kp",      CTRL_I32, &g_kp,      Kp::FRAC    },
    { "ki",      CTRL_I32, &g_ki,       Ki::FRAC    },
    { "kd",      CTRL_I32, &g_kd,       Kd::FRAC    },
    { "alpha_y", CTRL_I32, &g_alpha_y,  Alpha::FRAC },
    { "alpha_i", CTRL_I32, &g_alpha_i,  Alpha::FRAC },
    { "alpha_e", CTRL_I32, &g_alpha_e,  Alpha::FRAC },
    { "ref",     CTRL_I32, &g_ref,      REF_FRAC    },
    { "refrate", CTRL_I32, &g_refrate,  REF_FRAC    },
    { "uff",     CTRL_I16, &g_uff,      0           },
    { "offset",  CTRL_I16, &g_offset,   0           },
    { "izero",   CTRL_I16, &g_izero,    0           },
    { "target",  CTRL_U8,  &g_target,   0           },
    { "mode",    CTRL_U8,  &g_mode,     0           },
    { "tickdiv", CTRL_U8,  &g_tickdiv,  0           },
    { "bidir",   CTRL_U8,  &g_bidir,    0           },
    { "uinvert", CTRL_U8,  &g_uinvert,  0           },
    { "pwmtop",  CTRL_U16, &g_pwmtop,   0           },
    { "y",       CTRL_I16, &g_y,        0           },
    { "y_uw",    CTRL_I32, &g_y_uw,     0           },
    { "mstat",   CTRL_U8,  &g_mstat,    0           },
    { "spres",   CTRL_U8,  &g_spres,    0           },
    { "maxlate", CTRL_U16, &g_maxlate,  0           },
    { "missed",  CTRL_U16, &g_missed,   0           },
    { "sovr",    CTRL_U16, &g_sovr,     0           },
    { "serr",    CTRL_U16, &g_serr,     0           },
};

static const float COUNTS_TO_DEG = 360.0f / COUNTS_PER_REV;

// `ref` y `e` están en las unidades crudas de lo que seleccione `target`, así
// que acá no llevan escala de ingeniería: declarar grados sería mentir apenas el
// lazo pase a corriente. Salen en unidades del target y la computadora multiplica
// por la escala de `y_uw` o la de `i` según corresponda. Todo lo demás tiene un
// significado fijo y lleva la suya.
static const CtrlChannel PROGMEM g_channels[] =
{
    { "ref",   CTRL_I32, &g_ref,   1.0f / (1 << REF_FRAC), "tgt" },
    { "y_uw",  CTRL_I32, &g_y_uw,  COUNTS_TO_DEG,      "deg" },
    { "y_uwf", CTRL_I32, &g_y_uwf, COUNTS_TO_DEG,      "deg" },
    { "e",     CTRL_I16, &g_e,     1.0f,               "tgt" },
    { "u",     CTRL_I16, &g_u,     1.0f,               "pwm" },
    { "i",     CTRL_I16, &g_i,     SENSE_MA_PER_LSB,   "mA"  },
};

// ----------------------------------------------------------------- temporizador

// Timer2, CTC, preescalador 32: 16 MHz / 32 / 100 = exactamente 5,000 kHz.
// El Timer2 deja en paz a millis() (Timer0), pero choca con tone() y con
// analogWrite() en los pines 3 y 11.
static void startSampleTimer(void)
{
    TCCR2A = _BV(WGM21);                // CTC, TOP = OCR2A
    TCCR2B = _BV(CS21) | _BV(CS20);     // preescalador /32
    OCR2A = 99;
    TCNT2 = 0;
    TIMSK2 = _BV(OCIE2A);
}

// Timer1, phase-correct con TOP = ICR1, preescalador 1: f = 16 MHz / (2*pwmtop),
// o sea 1 kHz con el TOP por omisión. El TOP propio es lo que hace que la
// frecuencia sea un parámetro y no un modo fijo; el precio es que analogWrite()
// deja de servir sobre este pin, porque da por sentado que el TOP son 255. De ahí
// pwm_write().
//
// En abstracto conviene modular rápido: fuera del rango audible, y con la
// ondulación de corriente --que es inversamente proporcional a la frecuencia--
// bien lejos de la banda del lazo. Pero este banco no lo tolera, y el motivo es
// instructivo. El L298 es un puente de Darlington bipolares: cae del orden de 2 V
// entre sus dos lados y tarda unos 2 us en conmutar. Contra una alimentación de
// 5 V eso deja unos 2,5 V para el motor, y a 20 kHz --períodos de 50 us-- lo que
// se pierde en cada transición, más lo que se pierda en la recuperación de los
// diodos del módulo, se lleva una fracción grande de un tiempo de encendido que
// ya venía escaso: medido en este banco, a 20 kHz el motor directamente no
// arranca, y a 1 kHz anda. Bajar la frecuencia multiplica por veinte el tiempo de
// encendido sin cambiar las pérdidas por transición, y eso es lo que devuelve el
// par.
//
// 1 kHz sale exacto (TOP = 8000) y cae justo sobre el período del lazo, así que
// los dos quedan enganchados en fase en lugar de batir: el muestreador de 5 kHz
// toma siempre las mismas cinco fases de la ondulación, lo que da un sesgo fijo en
// `i` en lugar de una oscilación lenta. Con un puente MOSFET --un TB6612FNG, un
// DRV8833-- nada de esto haría falta y `dev.pwm(20000)` sería lo correcto.
//
// Se arranca con la salida de comparación desconectada, que es el puente abierto:
// la conecta pwm_write() cuando hay algo que accionar.
static void startMotorPwm(void)
{
    TCCR1A = _BV(WGM11);                    // modo 10: phase-correct, TOP = ICR1
    TCCR1B = _BV(WGM13) | _BV(CS10);        // preescalador /1
    TCNT1  = 0;
    ICR1   = g_pwmtop;
}

// ------------------------------------------------------------------------ adc

// El ADC se maneja directamente desde el muestreador en lugar de a través de
// analogRead(), que espera activamente a que termine la conversión. Una
// conversión con preescalador /128 tarda 104 us, así que entra en un período de
// muestreo de 200 us: la ISR recoge el resultado que arrancó el tick anterior e
// inmediatamente lanza el siguiente. El costo es un período de muestreo de
// retardo en `i`; el ahorro son 112 us de bloqueo dentro de un período de control
// de 1000 us.
static void startAdc(void)
{
    ADMUX  = (SENSE_REF_INTERNAL ? (_BV(REFS1) | _BV(REFS0)) : _BV(REFS0))
           | (SENSE_CHANNEL & 0x07);
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADSC);
}

ISR(TIMER2_COMPA_vect)
{
    static uint8_t count = 0;

    Sensor::do_transfer();

    if (ADCSRA & _BV(ADIF))
    {
        g_adc = (int16_t)ADC;
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
        // loop() no atendió el tick anterior: el período de control se está
        // perdiendo del todo, que es peor que una simple fluctuación.
        g_missed_isr++;
    }

    g_tick_us = micros();
    g_tick    = true;
}

// -------------------------------------------------------------------- control

// El camino más corto de a hasta b en una circunferencia de 4096 cuentas, para
// que una referencia apenas pasado el punto de vuelta no ordene una vuelta
// entera en el sentido equivocado.
static int16_t wrapped_error(int16_t a, int16_t b)
{
    return (int16_t)(((a - b + 2048) & 0x0FFF) - 2048);
}

static int16_t clamp16(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return (int16_t)lo;
    if (v > hi) return (int16_t)hi;
    return (int16_t)v;
}

// Desconecta la salida de comparación del pin, que vuelve a ser una salida común
// con su bit de PORT en bajo desde setup(): ENA queda en bajo y el puente abierto,
// en el ciclo en el que se pide y no al final del período de PWM.
static inline void pwm_off(void)
{
    TCCR1A &= ~_BV(COM1A1);
}

// `mag` va de 0 a U_MAX y el temporizador cuenta hasta `pwmtop`, que es otra
// escala. Se divide por U_MAX + 1 = 256 en lugar de por 255, que es un corrimiento
// en vez de una división y deja el ciclo de trabajo a lo sumo un escalón corto; el
// extremo de arriba, que es el que se notaría --U_MAX tiene que ser encendido
// permanente y no 255/256 de él--, se atiende aparte. Cambiar U_MAX obliga a
// cambiar el corrimiento con él. OCR1A está doblemente amortiguado, así
// que el valor nuevo entra al terminar el período en curso y ningún pulso sale
// cortado por la mitad.
static inline void pwm_write(int16_t mag)
{
    if (mag <= 0)
    {
        pwm_off();
        return;
    }

    OCR1A = (mag >= U_MAX) ? g_pwmtop
                           : (uint16_t)(((uint32_t)mag * g_pwmtop) >> 8);

    TCCR1A |= _BV(COM1A1);
}

// Pone `u` sobre el puente: la magnitud en ENA por PWM, el sentido en IN1/IN2.
//
// Un cambio de sentido no escribe las entradas de sentido con el puente vivo.
// Primero baja ENA, que apaga las cuatro llaves de una sola escritura; recién
// entonces mueve IN1 e IN2, y pasa por el estado con las dos en bajo antes de
// levantar la que corresponde. El PWM vuelve al final, ya con el sentido nuevo
// en pie.
//
// ¿Alcanza como tiempo muerto el intervalo entre apagar ENA y mover IN1? Sí, y
// con holgura: pwm_off() suelta el pin en el ciclo en que se ejecuta, y el
// digitalWrite() que sigue se pasa unos 4 us leyendo tablas en PROGMEM y
// deshabilitando interrupciones antes de llegar a tocar su propio pin --eso es lo
// que cuesta un digitalWrite() en un AVR de 16 MHz--, contra el orden de 1 a 2 us
// que tarda el L298 en abrir una salida. El tiempo muerto sobra por un factor de
// dos o tres sin escribir un solo delay.
//
// Pero la protección de verdad no es ese tiempo, y conviene no apoyar el
// argumento ahí. Cada medio puente del L298 cuelga de una sola entrada lógica, y
// el reparto entre el transistor de arriba y el de abajo es interno: desde afuera
// no hay forma de pedirle a una rama que conduzca por los dos lados a la vez, se
// escriba como se escriba. Lo que compra bajar ENA primero es que el cambio de
// sentido no atraviese ningún estado conduciendo, y eso vale por sí solo, sin
// depender de cuántos microsegundos separen las escrituras.
//
// Lo que ningún tiempo muerto arregla es lo otro que pasa al invertir: la
// corriente que ya circula por el motor no se puede cortar, así que sale por los
// diodos del puente contra la fuente. Eso es milisegundos —la constante L/R del
// motor—, no microsegundos, y la respuesta es no pedir saltos de +255 a -255, no
// separar más las escrituras.
//
// `u == 0` deja las entradas de sentido donde estaban en lugar de forzarlas: con
// ENA en cero el puente ya está abierto y el motor en punto muerto, y así un
// comando que ronda el cero no golpea IN1/IN2 en cada período. El cambio cuesta
// tres digitalWrite(), del orden de 12 us, y sólo en los períodos en los que el
// signo realmente da vuelta.
static void drive(int16_t u)
{
    static int8_t dir = 0;

    // `uinvert` reconcilia dos convenciones de signo que se fijan con cables: la
    // del motor en las salidas del puente, y la del imán sobre el sensor. Si no
    // coinciden, el lazo de posición realimenta en positivo y se escapa en lugar
    // de establecerse -- y se escapa igual con la referencia de cualquier signo,
    // así que no hay manera de descubrirlo probando. Dar vuelta los dos cables del
    // motor es el arreglo físico y equivale exactamente a esto.
    int8_t sign = (u > 0) ? 1 : ((u < 0) ? -1 : 0);

    if (g_uinvert)
    {
        sign = (int8_t)-sign;
    }

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

// El imán gira en sentido contrario al eje, de ahí la negación; `offset` es
// entonces la cuenta que se lee como cero.
static int16_t sensor_measurement(void)
{
    return wrapped_error(g_offset, (int16_t)Sensor::counts());
}

// Lee los sensores y actualiza todas las variables medidas. Corre una vez por
// período de control cualquiera sea el modo, así que la telemetría sigue viva en
// lazo abierto.
static void measure(void)
{
    int16_t adc;
    noInterrupts();
    adc = g_adc;
    interrupts();

    g_i = clamp16(g_i_filt[1].update(g_i_filt[0].update(adc - g_izero)),
                  INT16_MIN, INT16_MAX);

    int16_t y = sensor_measurement();
    g_y_uw  += wrapped_error(y, g_y);
    g_y_uwf  = g_y_filt[1].update(g_y_filt[0].update(g_y_uw));
    g_y      = y;
}

// El despacho de la realimentación: sobre qué magnitud medida cierra el lazo.
// De paso publica el error recortado para la telemetría, así que un controlador
// que ignore el valor igual deja `e` vivo para que la computadora lo mire.
static int32_t target_error(void)
{
    int32_t ref = g_ref >> REF_FRAC;
    int32_t e;

    if (g_target == TARGET_CURRENT)
    {
        e = ref - (int32_t)g_i;
    }
    else
    {
        // Posición. alpha_y = 1 hace que el filtro deje pasar la señal tal cual,
        // así que ésta es la cuenta desenrollada cruda salvo que la computadora
        // haya pedido suavizado.
        e = ref - g_y_uwf;
    }

    g_e = clamp16(e, INT16_MIN, INT16_MAX);
    return e;
}

// --------------------------------------------------------------- controladores
// Una función por modo, todas con la misma firma: leer las mediciones y los
// parámetros, dejar un comando en `g_u`. Agregar un controlador es agregar una
// función acá y un caso al switch de control_step().

// Lazo abierto: la computadora pone `uff` directamente sobre el actuador. Éste
// es el modo para identificar la planta: aplicar un escalón en uff y mirar qué
// vuelve.
static void controller_open(void)
{
    (void)target_error();
    g_u = clamp16(g_uff, g_u_min, U_MAX);
}

static void controller_pid(void)
{
    int32_t e = target_error();

    g_e_prev = g_e_filt;
    g_e_filt = g_err_filt.update(e);

    int32_t candidate = Kp::from_raw(g_kp).scale(e)
                      + Ki::from_raw(g_ki).scale(g_integral)
                      + Kd::from_raw(g_kd).scale(g_e_filt - g_e_prev)
                      + (int32_t)g_uff;

    g_u = clamp16(candidate, g_u_min, U_MAX);

    // Integración condicional: dejar de cargar el integrador en cuanto el
    // actuador satura en el sentido hacia el que el integrador está empujando.
    bool saturated = (candidate > U_MAX && e > 0)
                  || (candidate < g_u_min && e < 0);

    if (!saturated)
    {
        // Segunda línea de defensa, y la que importa cuando se cambia ki en
        // plena corrida: acotar la suma en el punto donde su término por sí solo
        // saturaría el actuador, para que el integrador siempre pueda
        // descargarse en un período o dos. La suma se forma en un tipo ancho
        // porque la cota se aplica recién después, y un único error grande
        // podría de otro modo desbordar el acumulador en el camino.
        int64_t sum = (int64_t)g_integral + e;

        if (sum >  g_integral_max) sum =  g_integral_max;
        if (sum < -g_integral_max) sum = -g_integral_max;

        g_integral = (int32_t)sum;
    }
}

// La rampa es el PID con una referencia móvil, así que es el PID más una línea y
// no un controlador aparte.
static void controller_ramp(void)
{
    g_ref += g_refrate;
    controller_pid();
}

// Se llama cuando la computadora cambia de controlador. Sin esto, un controlador
// hereda el integrador y la historia derivativa del anterior y da un salto en su
// primer período.
static void reset_controller(int32_t e)
{
    g_integral = 0;
    g_e_filt   = e;
    g_e_prev   = e;
    g_err_filt.reset(e);
}

static void control_step(void)
{
    measure();

    static uint8_t last_mode = MODE_OPEN;

    if (g_mode != last_mode)
    {
        reset_controller(target_error());
        last_mode = g_mode;
    }

    switch (g_mode)
    {
        case MODE_PID:  controller_pid();  break;
        case MODE_RAMP: controller_ramp(); break;

        // Un modo desconocido es el modo seguro: un error de tipeo en la
        // computadora no puede dejar el actuador gobernado por un controlador
        // que nadie eligió.
        case MODE_OPEN:
        default:        controller_open(); break;
    }

    drive(g_u);
}

// Los contadores propios del sensor corren libres y no se pueden borrar, así que
// lo que se publica es el total acumulado de sus incrementos. Eso es lo que hace
// que `sovr` y `serr` los pueda escribir la computadora igual que el resto: poner
// uno en cero reinicia la cuenta desde acá en lugar de que lo pisen en el período
// siguiente.
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

    // A diferencia de los contadores, esto es un estado y no una cuenta: dice
    // si el sensor está contestando ahora, no cuántas veces falló. Es lo que
    // distingue un imán mal montado -- el sensor contesta y se queja del imán --
    // de un sensor que directamente no está en el bus.
    g_spres = Sensor::present() ? 1 : 0;
}

// --------------------------------------------------------------------- ajuste

// Aplica lo que la computadora acaba de escribir. Los parámetros llegan ya en la
// forma que quiere la aritmética —esa conversión es trabajo de la computadora—,
// así que todo lo que hace esto es propagar los dos de los que depende otro
// estado. Sin punto flotante, que es la razón por la que es seguro correrlo
// inmediatamente después de un paso de control.
//
// Lo dispara el contador de escrituras de CtrlLink: una comparación de 16 bits
// por pasada de loop(), en lugar de vigilar parámetro por parámetro.
static void refresh_tuning(void)
{
    if (g_tickdiv == 0)
    {
        g_tickdiv = 1;
    }

    // El divisor que usa la ISR y el período que se le informa a la computadora
    // cambian juntos, así que ninguno de los dos puede quedar describiendo una
    // frecuencia a la que el lazo no está corriendo. Las ganancias son por
    // muestra y no dependen de ninguno de los dos.
    g_divider = g_tickdiv;
    CtrlLink::set_period_us((uint32_t)g_tickdiv * 1000000UL / SAMPLE_HZ);

    // El recorte y el anti-windup tienen que describir el puente que está
    // cableado: prometerle al integrador un sentido que el hardware no tiene lo
    // deja cargando contra un límite que no existe.
    g_u_min = g_bidir ? (int16_t)-U_MAX : (int16_t)0;

    // La frecuencia del PWM se aplica sólo cuando cambió de verdad. ICR1 no está
    // amortiguado en este modo, así que escribirlo con el contador ya pasado del
    // TOP nuevo cuesta un período largo hasta que la cuenta da la vuelta entera;
    // rearrancar el temporizador desde cero lo evita. Pero eso interrumpe el PWM,
    // y este cuerpo corre después de cada escritura de cualquier parámetro: un
    // barrido de kp no tiene por qué sacudir el puente.
    static uint16_t pwm_applied = 0;

    if (g_pwmtop < PWM_TOP_MIN)
    {
        g_pwmtop = PWM_TOP_MIN;
    }

    if (g_pwmtop != pwm_applied)
    {
        pwm_applied = g_pwmtop;
        startMotorPwm();
    }

    // Acotar el integrador en la suma cuyo término por sí solo satura el
    // actuador, para que siempre pueda descargarse en un período o dos. Un ki lo
    // bastante chico como para poner esa cota más allá de lo que entra en un
    // int32_t deja en pie el límite del propio tipo: la acumulación tiene que
    // quedar en rango le importe o no a ki.
    int32_t ki = (g_ki < 0) ? -g_ki : g_ki;

    g_integral_max = INT32_MAX / 2;

    if (ki != 0)
    {
        int64_t bound = ((int64_t)U_MAX << Ki::FRAC) / ki;

        if (bound < g_integral_max)
        {
            g_integral_max = (int32_t)bound;
        }
    }

    g_y_filt[0].set_alpha(Alpha::from_raw(g_alpha_y));
    g_y_filt[1].set_alpha(Alpha::from_raw(g_alpha_y));
    g_i_filt[0].set_alpha(Alpha::from_raw(g_alpha_i));
    g_i_filt[1].set_alpha(Alpha::from_raw(g_alpha_i));
    g_err_filt.set_alpha(Alpha::from_raw(g_alpha_e));
}

// La visión que el propio AS5600 tiene del imán: detectado, muy débil, muy
// fuerte. Leerla le cuesta al lazo de muestreo una muestra y bloquea acá hasta
// que esa muestra llegue, así que sólo se hace entre capturas; es decir, durante
// una verificación de puesta en marcha, que es la única vez que a alguien le
// interesa.
static void refresh_magnet_status(void)
{
    static uint32_t last_ms = 0;

    if (CtrlLink::streaming() || (millis() - last_ms) < 500)
    {
        return;
    }
    last_ms = millis();

    if (!Sensor::present())
    {
        // Sin sensor en el bus no hay nada que informar del imán, y dejar el
        // último valor sería peor que no decir nada.
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
// —el encabezado son siete líneas, y escribir bloquea en cuanto se llena el
// buffer de transmisión—, y los períodos que se pierden ahí son el precio de
// arrancar la captura, no una falla del lazo: se repetían idénticos, media
// docena, lo mismo en una captura de medio segundo que en una de cuatro.
//
// Hay que limpiar también el contador de la ISR, que todavía guarda los ticks
// perdidos durante ese bloqueo y los sumaría en la pasada siguiente.
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
    // ENA primero: mientras el puente esté abierto las entradas de sentido no
    // gobiernan nada, así que ése es el orden en el que ningún estado intermedio
    // acciona el motor. Las dos en bajo es el estado del que parte drive().
    startMotorPwm();
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    digitalWrite(MOTOR_PWM_PIN, LOW);
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    digitalWrite(MOTOR_IN1_PIN, LOW);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    digitalWrite(MOTOR_IN2_PIN, LOW);

    CtrlLink::set_id(F("ControlDemo"));
    CtrlLink::begin(BAUD,
                    g_params,   sizeof(g_params)   / sizeof(g_params[0]),
                    g_channels, sizeof(g_channels) / sizeof(g_channels[0]),
                    (uint32_t)g_tickdiv * 1000000UL / SAMPLE_HZ);

    refresh_tuning();

    startAdc();
    Sensor::begin();
    startSampleTimer();

    CtrlLink::note(F("ControlDemo listo"));
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

        // Se acumula en una copia común en lugar de leerse directamente del
        // contador de la ISR, para que la computadora pueda ponerlo en cero sin
        // competir con la ISR.
        g_missed += missed;

        uint16_t late = (uint16_t)(micros() - fired);
        if (late > g_maxlate)
        {
            g_maxlate = late;
        }

        control_step();
        collect_sensor_health();

        CtrlLink::emit();
    }

    // Después del paso de control, nunca antes: un `set` que caiga justo cuando
    // se dispara un tick quedaría de otro modo por delante de él.
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
