// Lazo de control de posición sobre CtrlLink, gobernado desde un notebook de
// Jupyter.
//
// Hardware:
// Arduino UNO
// Sensor de posición de efecto Hall: AS5600 (I2C)   SDA -> A4, SCL -> A5
// Medición de corriente (opcional): ACS712 en A0
// Actuador (opcional): PWM en el pin 5, sentido de giro en el pin 8
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
// en los pines 3 y 11 y tone() dejan de funcionar; y el ADC, que se maneja
// directamente acá, así que no hay que llamar a analogRead(). Los pines 9 y 10
// (Timer1) y 5 y 6 (Timer0) no se ven afectados.

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

// Dejar MOTOR_PWM_PIN sin definir para correr el lazo sin actuador conectado:
// todo lo demás, telemetría incluida, se comporta igual.
//
// Definir además MOTOR_DIR_PIN da un accionamiento bidireccional, -255..255. Con
// ese pin sin definir el puente es de un solo cuadrante y el comando se recorta
// en cero, que es lo que se le informa a la lógica anti-windup a través de U_MIN.
#define MOTOR_PWM_PIN 5
//#define MOTOR_DIR_PIN 8

#ifdef MOTOR_DIR_PIN
static const int16_t U_MIN = -255;
#else
static const int16_t U_MIN = 0;
#endif
static const int16_t U_MAX = 255;

// Medición de corriente en A0. ACS712-05B: 185 mV/A alrededor de un cero de
// 2,5 V, o sea media escala con referencia de 5 V. Cambiar SENSE_MV_PER_A para
// otro componente; sólo afecta las unidades que se le informan a la computadora,
// nunca al lazo.
static const uint8_t  SENSE_CHANNEL   = 0;
static const int16_t  SENSE_ZERO      = 512;
static const float    SENSE_MV_PER_A  = 185.0f;
static const float    ADC_MV_PER_LSB  = 5000.0f / 1024.0f;
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
static int16_t g_offset  = 0;   // cero del sensor, en cuentas
static uint8_t g_target  = TARGET_POSITION;
static uint8_t g_mode    = MODE_OPEN;
static uint8_t g_tickdiv = 5;   // muestras de 5 kHz por período de control: 5 -> 1 kHz

static int16_t g_y     = 0;     // ángulo medido, cuentas, con el offset aplicado
static int32_t g_y_uw  = 0;     // ángulo desenrollado, cuentas, sin filtrar
static int32_t g_y_uwf = 0;     // ángulo desenrollado, cuentas, filtrado por alpha_y
static int16_t g_i     = 0;     // corriente, LSBs del ADC alrededor de SENSE_ZERO, filtrada
static int16_t g_e     = 0;     // error, unidades del target, recortado para la telemetría
static int16_t g_u     = 0;     // comando al actuador, U_MIN..U_MAX

// Contadores de salud. Todos los puede escribir la computadora, así que poner uno
// en cero reinicia esa cuenta.
static uint16_t g_maxlate = 0;  // peor retardo observado entre la ISR y su atención, us
static uint16_t g_missed  = 0;  // períodos de control que loop() nunca atendió
static uint16_t g_sovr    = 0;  // muestras del sensor que el bus I2C no llegó a seguir
static uint16_t g_serr    = 0;  // transferencias del sensor que fallaron
static uint8_t  g_mstat   = 0;  // registro STATUS del AS5600: imán presente, muy débil, muy fuerte

// Estado del integrador, en unidades de error sumadas a lo largo de los ticks.
// Guardar la suma cruda y aplicar ki*dt una sola vez al final es lo que permite
// que sobreviva una ganancia de 5e-5: la cuantización cae sobre la ganancia,
// donde es una fracción de un por ciento, en lugar de caer sobre la acumulación,
// donde se truncaría a cero en cada período.
static int32_t g_integral = 0;
static int32_t g_e_filt   = 0;
static int32_t g_e_prev   = 0;

// La única magnitud derivada que queda, recalculada por refresh_tuning().
static int32_t g_integral_max = INT32_MAX / 2;

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
    { "target",  CTRL_U8,  &g_target,   0           },
    { "mode",    CTRL_U8,  &g_mode,     0           },
    { "tickdiv", CTRL_U8,  &g_tickdiv,  0           },
    { "y",       CTRL_I16, &g_y,        0           },
    { "y_uw",    CTRL_I32, &g_y_uw,     0           },
    { "mstat",   CTRL_U8,  &g_mstat,    0           },
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
// El Timer2 deja en paz a millis() (Timer0) y a Servo (Timer1), pero choca con
// tone() y con analogWrite() en los pines 3 y 11.
static void startSampleTimer(void)
{
    TCCR2A = _BV(WGM21);                // CTC, TOP = OCR2A
    TCCR2B = _BV(CS21) | _BV(CS20);     // preescalador /32
    OCR2A = 99;
    TCNT2 = 0;
    TIMSK2 = _BV(OCIE2A);
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
    ADMUX  = _BV(REFS0) | (SENSE_CHANNEL & 0x07);   // referencia AVcc
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

static void drive(int16_t u)
{
#ifdef MOTOR_PWM_PIN
#ifdef MOTOR_DIR_PIN
    digitalWrite(MOTOR_DIR_PIN, (u >= 0) ? HIGH : LOW);
#endif
    analogWrite(MOTOR_PWM_PIN, (uint8_t)(u >= 0 ? u : -u));
#else
    (void)u;
#endif
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

    g_i = clamp16(g_i_filt[1].update(g_i_filt[0].update(SENSE_ZERO - adc)),
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
    g_u = clamp16(g_uff, U_MIN, U_MAX);
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

    g_u = clamp16(candidate, U_MIN, U_MAX);

    // Integración condicional: dejar de cargar el integrador en cuanto el
    // actuador satura en el sentido hacia el que el integrador está empujando.
    bool saturated = (candidate > U_MAX && e > 0)
                  || (candidate < U_MIN && e < 0);

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

    uint8_t status;
    if (Sensor::read_status(status))
    {
        g_mstat = status;
    }
}

// ------------------------------------------------------------------- Arduino

void setup()
{
#ifdef MOTOR_PWM_PIN
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    analogWrite(MOTOR_PWM_PIN, 0);
#endif
#ifdef MOTOR_DIR_PIN
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    digitalWrite(MOTOR_DIR_PIN, HIGH);
#endif

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
}
