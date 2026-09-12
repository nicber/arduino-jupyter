// Lazo de control de posición sobre CtrlLink, gobernado desde un notebook de
// Jupyter.
//
// Hardware:
// Arduino UNO
// Sensor de posición de efecto Hall: AS5600 (I2C)   SDA -> A4, SCL -> A5
// Medición de corriente (opcional): ACS712 en A0
// Actuador (opcional): puente L298N, ENA -> 9 (PWM, 1 kHz), IN1 -> 6, IN2 -> 7
//
// Este archivo no hace casi nada por sí mismo: arma los módulos, los cablea entre
// sí y publica sus parámetros. Cada cosa que se puede medir, accionar o ajustar
// tiene un dueño, y el dueño se lleva su estado adentro:
//
//   g_clock    el reloj del lazo          Sampler/SampleClock.h
//   g_adc      el conversor corriendo libre  Sense/FreeAdc.h
//   g_current  la corriente con sentido    Sense/CurrentSense.h
//   g_lut      la corrección del ángulo    Calibracion/AngleLut.h
//   g_angle    el ángulo desenrollado      AngleSensor/AngleTracker.h
//   g_health   qué se le puede creer al sensor  AngleSensor/SensorHealth.h
//   g_pid      la ley de control           Control/Pid.h
//   g_set      la referencia y el objetivo Control/Setpoint.h
//   g_motor    el puente                   Actuator/HBridge.h
//
// Lo que queda acá es lo que de verdad es de este sketch y de ningún módulo: qué
// pines, qué sensor, qué escalas, y el orden en el que las cosas arrancan.
//
// El Timer2 muestrea el AS5600 a 5 kHz; cada `lop_div` muestras se ejecuta la ley
// de control, así que la frecuencia del lazo es 5000/lop_div Hz y por omisión vale
// 500 Hz. De esa manera el muestreo mantiene un período rígido aun cuando el
// cálculo de control fluctúe. `lop_late` informa cuánta fluctuación hubo y
// `lop_missed` cuenta los períodos de control que se saltearon del todo.
//
// La ley de control es aritmética entera de punta a punta; ver ControlMath. Y
// también lo es cada parámetro que lee: cada uno se guarda en la forma de punto
// fijo que la aritmética necesita, y la tabla de parámetros declara la escala
// que lo convierte. La computadora multiplica a la ida y divide a la vuelta, así
// que el alumno sigue escribiendo `dev.pid_kp = 0.5` y este sketch no ejecuta una
// sola instrucción de punto flotante.
//
// Los nombres de los parámetros llevan un prefijo de módulo, y eso es lo que hace
// que una tabla de tres docenas de entradas planas diga quién es dueño de cada
// una: `pid_kp` contra `ang_offset` contra `mot_top`. La computadora los descubre
// en tiempo de ejecución, así que agregar una ganancia no cuesta nada del lado de
// Python.
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
// La tabla de canales por omisión son 45 bytes por fila, o el 23 % del enlace a
// 500 Hz, que es el "bastante por debajo de la mitad" que le gusta a este
// protocolo. Bajar `lop_div` a 5 devuelve el lazo a 1 kHz y lleva la fila al
// 45 %, que ya es demasiado: ahí conviene subir `dec` o sacar un canal.
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
#include <FixedPoint.h>
#include <FirstOrderFilter.h>

#include <AngleLut.h>
#include <AngleTracker.h>
#include <SensorHealth.h>
#include <CurrentSense.h>
#include <FreeAdc.h>
#include <HBridge.h>
#include <Pid.h>
#include <SampleClock.h>
#include <Setpoint.h>

// -------------------------------------------------------------------- el banco

static const uint32_t BAUD           = 1000000;
static const uint16_t SAMPLE_HZ      = 5000;
static const int16_t  COUNTS_PER_REV = 4096;

// Actuador: un puente en H L298N. ENA lleva la magnitud por PWM y el par IN1/IN2
// el sentido. Se puede correr el lazo sin nada conectado acá: los pines conmutan
// igual y todo lo demás, telemetría incluida, se comporta idéntico.
//
// ENA va al pin 9 porque es OC1A, y el Timer1 es el único que queda libre: el
// Timer0 (pines 5 y 6) lleva millis() y no se le puede tocar el preescalador, y el
// Timer2 (pines 3 y 11) es el muestreador de 5 kHz. IN1 e IN2 son salidas
// digitales comunes y pueden ir a cualquier pin. Ver HBridge.h.
static const uint8_t MOTOR_PWM_PIN = 9;     // ENA del L298N, OC1A
static const uint8_t MOTOR_IN1_PIN = 6;     // IN1
static const uint8_t MOTOR_IN2_PIN = 7;     // IN2

// El TOP del Timer1 con el que arranca el PWM del puente, que es la forma en que
// se guarda la frecuencia: f = 16 MHz / (2 * top). Se guarda el TOP y no los Hz
// por la misma razón por la que el divisor del lazo se guarda como divisor y no
// como frecuencia: es por lo que cuenta el hardware, es exacto, y la conversión la
// hace el lado que tiene la aritmética. `dev.pwm(20000)` del lado del notebook.
//
// En abstracto conviene modular rápido, pero este banco no lo tolera y el motivo
// es instructivo. El L298 es un puente de Darlington bipolares: cae del orden de
// 2 V entre sus dos lados y tarda unos 2 us en conmutar. Contra una alimentación
// de 5 V eso deja unos 2,5 V para el motor, y a 20 kHz --períodos de 50 us-- lo
// que se pierde en cada transición se lleva una fracción grande de un tiempo de
// encendido que ya venía escaso: medido en este banco, a 20 kHz el motor
// directamente no arranca, y a 1 kHz anda.
//
// 1 kHz sale exacto (TOP = 8000) y entran dos períodos suyos en cada período del
// lazo, así que los dos quedan enganchados en fase en lugar de batir: el
// muestreador toma siempre las mismas cinco fases de la ondulación, lo que da un
// sesgo fijo en la corriente en lugar de una oscilación lenta. Con un puente
// MOSFET --un TB6612FNG, un DRV8833-- nada de esto haría falta.
static const uint16_t PWM_TOP_DEFAULT = 8000;   // 1,0 kHz

// Medición de corriente en A0. Nada de este bloque entra en la ley de control
// salvo que alguien ponga el objetivo en corriente: sobre todo fija las unidades
// que se le informan a la computadora.
static const uint8_t SENSE_CHANNEL = 0;

// La sensibilidad del sensor, que es lo único que convierte cuentas en amperes.
// 185 mV/A es un ACS712-05B conectado directo, que es como está pensado el banco.
// OJO si no cierra con lo que mide un tester en serie con el motor: un reposo muy
// por debajo de los 2500 mV que da un ACS712 alimentado a 5 V delata un divisor en
// la salida, y un divisor divide las dos cosas a la vez --el cero y la
// sensibilidad--, así que ahí va 185 dividido por lo mismo. Ver README.
static const float SENSE_MV_PER_A = 185.0f;

// La referencia del ADC, que es la única perilla de ganancia que tiene el AVR de
// este lado, y es una elección entre techo y resolución.
//
// Por omisión va la referencia alta, que es Vcc, y no sólo por el techo. Un ACS712
// es un sensor bipolar y ratiométrico: reposa en la mitad de su alimentación
// --2,5 V con 5 V-- para poder bajar cuando la corriente cambia de sentido. Medir
// esa salida contra Vcc es medirla contra la misma tensión que la produce, así que
// el reposo cae en media escala por construcción, valga Vcc 4,8 o 5,1 y sea cual
// sea la placa.
//
// Contra la referencia interna de 1,1 V, en cambio, ese mismo sensor satura en
// reposo: no mide nada, y desde el ADC se ve igual que una entrada al aire.
//
// Vcc es REFS=01 en las dos placas del banco. Y elegirla en el clon no es escribir
// REFS: ver board::adc_select_reference() en BoardStart.h.
//
// OJO que en este banco el reposo no cae en media escala: cae en 3071 cuentas de
// 4096, o sea 3,75 V contra Vcc de 5 V, y no en los 2,5 que daría un ACS712
// alimentado a 5. El canal mide bien --se lo verificó accionando el motor-- así
// que lo más probable es que el sensor no esté alimentado con los mismos 5 V: uno
// que reposa en 3,75 está viendo 7,5 V, que es lo que suele tener la fuente del
// puente. Vale la pena confirmarlo con un tester, porque de ahí sale para qué lado
// hay margen: la corriente de este banco hace *bajar* la salida, y hacia abajo
// quedan las 3071 cuentas enteras.
//
// Y porque un sensor alimentado por encima de 5 V puede sacar la salida por encima
// de 5 V, que es más de lo que le gusta a una entrada del AVR.
//
// Lo que se paga con la interna es resolución: un LSB pasa de 1,07 mV a 4,9, y con
// 185 mV/A eso deja 200 mA en apenas siete u ocho cuentas. SENSE_REF_INTERNAL en
// true vuelve a la interna, y sirve para un sensor unipolar --el que va en la
// alimentación del puente-- que reposa cerca de cero y no necesita techo.
static const bool  SENSE_REF_INTERNAL = false;
static const float ADC_REF_MV         = SENSE_REF_INTERNAL ? 1093.0f : 5006.0f;

// Todo lo que sigue cuenta en cuentas de 12 bits, en las dos placas del banco.
//
// El UNO tiene un ADC de 10 bits y el clon con LGT8F328P uno de 12, así que la
// misma tensión mide cuatro veces más en una que en la otra. Se corre la lectura
// del UNO dos bits para arriba en lugar de tirar los dos de abajo del clon: eso
// conserva lo que el clon mide de verdad y le cuesta al UNO dos ceros al final de
// un número que igual no los tenía. El corrimiento vive adentro de FreeAdc.
static const uint16_t ADC_FULL         = 4096;
static const int16_t  SENSE_ZERO       = SENSE_REF_INTERNAL ? 0 : (ADC_FULL / 2);
static const float    ADC_MV_PER_LSB   = ADC_REF_MV / ADC_FULL;
static const float    SENSE_MA_PER_LSB = 1000.0f * ADC_MV_PER_LSB / SENSE_MV_PER_A;

// Y la referencia interna no vale lo mismo en las dos placas, así que la escala de
// arriba es la del ATmega y nada más. La tabla de canales viaja en flash con una
// constante compilada y no se puede corregir al arrancar; lo que sí se calcula al
// arrancar es `cur_malsb`, y con eso la computadora corrige la escala de `i`.
//
// En el ATmega la referencia interna es el bandgap, especificado entre 1,0 y
// 1,2 V: el número es de esta placa y no del modelo, y son los 1093 mV medidos en
// el UNO de este banco. En el clon es una referencia trimada de fábrica y vale
// 1,024 V. Son un 6 % de diferencia, no un factor de cuatro: el factor de cuatro
// era el ancho del conversor y ya está corregido más arriba.
//
// Esta constante es sólo para esa elección. La alta no la necesita: las dos placas
// corren a 5 V y la referencia alta es Vcc en las dos.
//
// OJO que entonces ADC_REF_MV --los 5006 mV medidos en el UNO de este banco-- se
// usa para las dos placas. El canal es ratiométrico contra el Vcc de cada una, así
// que el número correcto para el clon es el Vcc del clon; mientras no se lo mida
// con un tester, lo que se acepta es ese error, que en dos placas alimentadas por
// el mismo USB es del orden del uno por ciento.
static const float ADC_REF_MV_LGT8F = 1024.0f;

// Son dos elecciones independientes, y vale la pena mantenerlas separadas.
//
// `ctl_mode` elige el controlador: la ley que convierte un error en un comando.
// `ctl_target` elige la realimentación: sobre qué magnitud medida cierra esa ley.
// Un controlador es un caso en control_step(); una realimentación es una rama en
// measured_value(). Ninguno de los dos sabe del otro.
enum : uint8_t
{
    MODE_OPEN = 0,      // u = ctl_uff, controlador puenteado
    MODE_PID  = 1,      // PID sobre la magnitud seleccionada
    MODE_RAMP = 2,      // PID, con la referencia avanzando por período
};

// La referencia está siempre en las unidades crudas de la magnitud seleccionada:
// cuentas para POSITION, LSBs del ADC para CURRENT.
enum : uint8_t
{
    TARGET_POSITION = 0,
    TARGET_CURRENT  = 1,
};

// ------------------------------------------------------------------ los módulos

typedef AS5600<NI2CBus>                                        Sensor;
typedef AngleLut<COUNTS_PER_REV, 64>                           Lut;
typedef AngleTracker<COUNTS_PER_REV>                           Angle;
typedef HBridge<MOTOR_PWM_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN>   Motor;
typedef FreeAdc<SENSE_CHANNEL>                                 Adc;

// Los divisores por omisión: 10 muestras de 5 kHz por período de control son
// 500 Hz de lazo.
static SampleClock  g_clock(10);
static Adc          g_adc;
static CurrentSense g_current(SENSE_ZERO);
static Lut          g_lut;
static Angle        g_angle;
static SensorHealth g_health;
static Pid          g_pid;
static Setpoint     g_set(MODE_OPEN, TARGET_POSITION);
static Motor        g_motor(PWM_TOP_DEFAULT);

// ------------------------------------------------ lo que es de este sketch y de nadie

// El filtro lento interno del AS5600, y lo último que de verdad se le escribió.
//
// Son dos campos y no dos globales sueltas, y ninguno es un static escondido
// adentro de la función de ajuste: escribirle al sensor puede fallar --viaja por el
// bus-- y entonces el intento siguiente tiene que volver a probar, así que hace
// falta saber qué quedó puesto. Ver apply_sensor_filter().
struct SensorFilter
{
    uint8_t want;       // el parámetro que fija la computadora
    uint8_t applied;    // lo último que el sensor aceptó
};

// SF_2X es el más rápido: el retardo de respuesta al escalón son 0,286 ms contra
// los 2,2 ms de 16x, y para un lazo esos 1,9 ms son mucho más caros que el ruido.
static SensorFilter g_sfilt = { 3, 0xFF };

// Lo que la placa mide de sí misma al arrancar, publicado para que la computadora
// no tenga que adivinarlo. Ver BoardStart.h.
struct BoardFacts
{
    uint16_t adcfs;     // fondo de escala real del conversor de esta placa
    uint16_t bgadc;     // el canal interno contra Vcc, en cuentas
    uint16_t malsb;     // mA por cuenta de `i` en esta placa, en Q8
    uint8_t  bus;       // estado eléctrico de las líneas del bus al arrancar
};

static BoardFacts g_board = { ADC_FULL, 0, 0, 0 };

// La cuenta cruda del sensor, sin corregir, sin offset y sin el signo invertido.
// Es lo que indexa la tabla de calibración, así que es lo que la computadora
// necesita para calcularla: reconstruirla desde el ángulo desenrollado se puede,
// pero deshacer un signo y un offset a mano es justo el lugar donde una
// calibración sale espejada y nadie se da cuenta hasta el final.
static uint16_t g_y_raw = 0;

// Prende y apaga la corrección en caliente, que es lo que permite medir cuánto
// sirve en lugar de suponerlo.
static uint8_t g_cal = 0;

// Una entrada de la tabla de calibración por escritura. Ver AngleLut::apply().
static uint32_t g_lutw   = Lut::NOTHING;
static uint16_t g_lutsum = 0;

// Los polos de los tres filtros. El estado filtrado vive adentro de cada módulo,
// pero el polo es un parámetro y se publica desde acá: alpha = dt / (tau + dt), una
// fracción en [0, 1]. alpha = 1 deja pasar la señal tal cual, que es la manera de
// apagar un filtro. Una computadora que piense en constantes de tiempo hace la
// conversión, porque es el lado que conoce dt.
//
// Juntos y no cada uno al lado de su módulo, para que se vea que son tres del mismo
// tipo y que los tres se aplican en el mismo lugar.
static int32_t g_alpha_y = Angle::Alpha::from_int(1).raw();                  // posición
static int32_t g_alpha_i = CurrentSense::Alpha::from_float(0.1667f).raw();   // corriente
static int32_t g_alpha_e = Pid::Alpha::from_float(0.0909f).raw();            // error

// Si había flujo en la pasada anterior del lazo, y el contador de escrituras que se
// vio la última vez. Los dos son de este archivo y no static adentro de una
// función, por la misma razón que todo lo demás: estado escondido en un lugar donde
// nadie lo busca.
static bool     g_was_streaming = false;
static uint16_t g_last_writes   = 0;

// ------------------------------------------------------------ la calibración

// La tabla NO se guarda en la placa. El dispositivo arranca siempre sin calibrar,
// y quien tiene la tabla es la computadora, que la empuja al conectarse --ver
// python/calib.py--. Es una decisión, no una limitación de memoria:
//
//   - Una calibración es una propiedad del *banco* --este imán, en este eje, con
//     este sensor--, no de la placa. En un archivo se lee, se compara, se revisa
//     y entra en el repositorio; en la EEPROM es estado invisible que sobrevive
//     a la reprogramación y que nadie recuerda haber puesto.
//   - Un dispositivo que arranca sin corregir no puede mentirle a nadie. El caso
//     feo de la EEPROM no es la tabla que falta: es la tabla vieja, de otro
//     montaje, que se aplica en silencio.
//   - Y para el aula: la corrección se prende y se apaga con `ang_cal` mientras el
//     motor gira. Eso es lo que hace que se pueda mostrar.
//
// Para dejarla fija en un tablero que se enciende solo,
// `calib.escribir_header()` genera Calibracion.h y este sketch lo toma si está.
#if defined(__has_include)
#  if __has_include("Calibracion.h")
#    include "Calibracion.h"
#    define TIENE_CALIBRACION 1
#  endif
#endif

// --------------------------------------------------------------------- tablas

// Cada entrada se guarda exactamente como la quiere la aritmética; la columna de
// escala es lo que le permite a la computadora seguir hablando en unidades
// naturales. El prefijo dice de qué módulo es el parámetro.
static const CtrlParam PROGMEM g_params[] =
{
    { "pid_kp",      CTRL_I32, &g_pid.kp,            Pid::Kp::FRAC     },
    { "pid_ki",      CTRL_I32, &g_pid.ki,            Pid::Ki::FRAC     },
    { "pid_kd",      CTRL_I32, &g_pid.kd,            Pid::Kd::FRAC     },
    { "pid_alpha",   CTRL_I32, &g_alpha_e,           Pid::Alpha::FRAC  },

    { "ctl_ref",     CTRL_I32, &g_set.ref,           Setpoint::FRAC    },
    { "ctl_rate",    CTRL_I32, &g_set.rate,          Setpoint::FRAC    },
    { "ctl_uff",     CTRL_I16, &g_set.uff,           0                 },
    { "ctl_mode",    CTRL_U8,  &g_set.mode,          0                 },
    { "ctl_target",  CTRL_U8,  &g_set.target,        0                 },

    { "mot_top",     CTRL_U16, &g_motor.top,         0                 },
    { "mot_bidir",   CTRL_U8,  &g_motor.bidir,       0                 },
    { "mot_invert",  CTRL_U8,  &g_motor.invert,      0                 },

    { "ang_offset",  CTRL_I16, &g_angle.offset,      0                 },
    { "ang_alpha",   CTRL_I32, &g_alpha_y,           Angle::Alpha::FRAC },
    { "ang_cal",     CTRL_U8,  &g_cal,               0                 },
    { "ang_filt",    CTRL_U8,  &g_sfilt.want,        0                 },
    { "ang_lutw",    CTRL_U32, &g_lutw,              0                 },
    { "ang_lutsum",  CTRL_U16, &g_lutsum,            0                 },
    { "ang_y",       CTRL_I16, &g_angle.y,           0                 },
    { "ang_y_uw",    CTRL_I32, &g_angle.y_uw,        0                 },
    { "ang_status",  CTRL_U8,  &g_health.status,     0                 },
    { "ang_present", CTRL_U8,  &g_health.present,    0                 },
    { "ang_agc",     CTRL_U8,  &g_health.agc,        0                 },
    { "ang_mag",     CTRL_U16, &g_health.magnitude,  0                 },
    { "ang_ovr",     CTRL_U16, &g_health.overruns,   0                 },
    { "ang_err",     CTRL_U16, &g_health.errors,     0                 },

    { "cur_zero",    CTRL_I16, &g_current.zero,      0                 },
    { "cur_invert",  CTRL_U8,  &g_current.invert,    0                 },
    { "cur_alpha",   CTRL_I32, &g_alpha_i,           CurrentSense::Alpha::FRAC },
    { "cur_malsb",   CTRL_U16, &g_board.malsb,       8                 },

    { "brd_adcfs",   CTRL_U16, &g_board.adcfs,       0                 },
    { "brd_bgadc",   CTRL_U16, &g_board.bgadc,       0                 },
    { "brd_bus",     CTRL_U8,  &g_board.bus,         0                 },

    { "lop_div",     CTRL_U8,  &g_clock.divide,      0                 },
    { "lop_late",    CTRL_U16, &g_clock.late,        0                 },
    { "lop_missed",  CTRL_U16, &g_clock.missed,      0                 },
};

static const float COUNTS_TO_DEG = 360.0f / COUNTS_PER_REV;

// Los canales NO llevan prefijo: son columnas de una serie temporal y viajan
// hasta un DataFrame, donde el nombre lo escribe quien grafica. Un prefijo ahí
// sería ruido en cada llamada a plot().
//
// `ref` y `e` están en las unidades crudas de lo que seleccione `ctl_target`, así
// que no llevan escala de ingeniería: declarar grados sería mentir apenas el lazo
// pase a corriente. La computadora multiplica por la escala de `y_uw` o la de `i`
// según corresponda.
static const CtrlChannel PROGMEM g_channels[] =
{
    { "ref",   CTRL_I32, &g_set.ref,      1.0f / (1 << Setpoint::FRAC), "tgt" },
    { "y_raw", CTRL_U16, &g_y_raw,        COUNTS_TO_DEG,    "deg" },
    { "y_uw",  CTRL_I32, &g_angle.y_uw,   COUNTS_TO_DEG,    "deg" },
    { "y_uwf", CTRL_I32, &g_angle.y_uwf,  COUNTS_TO_DEG,    "deg" },
    { "e",     CTRL_I16, &g_set.e,        1.0f,             "tgt" },
    { "u",     CTRL_I16, &g_motor.u,      1.0f,             "pwm" },
    { "i",     CTRL_I16, &g_current.i,    SENSE_MA_PER_LSB, "mA"  },
};

// ----------------------------------------------------------------------- la ISR

// Tres líneas, una por cosa que hay que hacer cada 200 us. Ninguna de las tres
// guarda estado acá: cada módulo se lleva el suyo.
ISR(TIMER2_COMPA_vect)
{
    Sensor::do_transfer();
    g_adc.on_isr();
    g_clock.on_isr();
}

// ------------------------------------------------------------------- el lazo

// Escribe los bits SF del CONF del sensor.
//
// El muestreador se para para escribir: nI2C encola la escritura y la completa su
// propia ISR, pero encolar reserva memoria y el muestreador llama a nI2C desde una
// ISR de temporizador. Con el muestreador quieto no hay nadie más pidiendo el bus.
// Cuesta un par de milisegundos de lazo detenido, y pasa sólo cuando alguien mueve
// el parámetro.
//
// Lee-modifica-escribe en lugar de escribir la palabra entera: CONF también lleva
// la histéresis, el modo de potencia y la salida, y ninguno de ésos es asunto de
// este parámetro.
static bool apply_sensor_filter(void)
{
    uint8_t conf[2];

    // La lectura sí pasa por el lazo de muestreo, así que va con el muestreador
    // todavía corriendo.
    if (!Sensor::read_registers(Sensor::REG_CONF_H, conf, 2))
    {
        return false;
    }

    uint16_t value = (uint16_t)(((uint16_t)conf[0] << 8) | conf[1]);
    value = (uint16_t)((value & ~0x0300u) | ((uint16_t)(g_sfilt.want & 0x03) << 8));

    conf[0] = (uint8_t)(value >> 8);
    conf[1] = (uint8_t)value;

    g_clock.pause();

    // Dejar terminar la transferencia que ya estaba en el aire. Acotado: sin sensor
    // en el bus esto no puede quedarse esperando para siempre.
    const uint32_t deadline = millis() + 5;
    while (Sensor::busy() && (int32_t)(millis() - deadline) < 0)
    {
    }

    const bool queued = Sensor::write_registers(Sensor::REG_CONF_H, conf, 2);

    // Cuatro bytes a 400 kHz son unos 100 us; dos milisegundos es holgura, no
    // cálculo.
    delay(2);

    g_clock.resume();

    return queued;
}

// Lee los sensores y actualiza todas las variables medidas. Corre una vez por
// período de control cualquiera sea el modo, así que la telemetría sigue viva en
// lazo abierto.
//
// La corrección de la tabla se aplica acá y no adentro del seguimiento del ángulo:
// así el seguimiento no sabe que existe una calibración, y la decisión de corregir
// o no queda donde se toma. La tabla se indexa con la cuenta cruda, que es la
// única que conserva el ángulo de adentro de la vuelta.
static void measure(void)
{
    g_current.update(g_adc.read());

    const Lut::Counts raw = (Lut::Counts)Sensor::counts();

    g_y_raw = (uint16_t)raw;
    g_angle.update(g_cal ? g_lut.corrected(raw) : raw);
}

// El despacho de la realimentación: sobre qué magnitud medida cierra el lazo.
static Setpoint::Value measured_value(void)
{
    if (g_set.target == TARGET_CURRENT)
    {
        return (Setpoint::Value)g_current.i;
    }

    // Posición. Con ang_alpha = 1 el filtro deja pasar la señal tal cual, así que
    // ésta es la cuenta desenrollada cruda salvo que la computadora haya pedido
    // suavizado.
    return (Setpoint::Value)g_angle.y_uwf;
}

static void control_step(void)
{
    measure();

    if (g_set.mode == MODE_RAMP)
    {
        // La rampa es el PID con una referencia móvil, así que es el PID más una
        // línea y no un controlador aparte.
        g_set.advance();
    }

    const Setpoint::Value e = g_set.error(measured_value());

    // Avisarle al controlador con qué configuración se está por correr. Reinicia si
    // cambió el modo *o* el objetivo, porque el integrador acumula en las unidades
    // de la magnitud realimentada. Ver Pid::configure().
    g_pid.configure(g_set.mode, g_set.target, e);

    Motor::Command u;

    switch (g_set.mode)
    {
        case MODE_PID:
        case MODE_RAMP:
            u = g_pid.step(e, g_motor.floor(), Motor::MAX, g_set.uff);
            break;

        // Lazo abierto: la computadora pone `ctl_uff` directamente sobre el
        // actuador. Éste es el modo para identificar la planta. Y un modo
        // desconocido cae acá a propósito: un error de tipeo en la computadora no
        // puede dejar el actuador gobernado por un controlador que nadie eligió.
        case MODE_OPEN:
        default:
            u = Pid::clamp(g_set.uff, g_motor.floor(), Motor::MAX);
            break;
    }

    g_motor.write(u);
}

// ---------------------------------------------------------------------- ajuste

// Aplica lo que la computadora acaba de escribir. Los parámetros llegan ya en la
// forma que quiere la aritmética --esa conversión es trabajo de la computadora--
// así que esto no hace más que avisarle a cada dueño que su parámetro se movió.
// Sin punto flotante, que es la razón por la que es seguro correrlo inmediatamente
// después de un paso de control.
//
// Una línea por dueño. Cada módulo se hace cargo de su propia detección de
// cambios, así que un barrido de ganancia no sacude el puente ni reescribe la
// tabla de calibración: eso vive adentro del que sabe por qué importa.
//
// Lo dispara el contador de escrituras de CtrlLink: una comparación de 16 bits por
// pasada de loop(), en lugar de vigilar parámetro por parámetro.
static void refresh_tuning(void)
{
    CtrlLink::set_period_us(g_clock.apply(SAMPLE_HZ));

    g_motor.apply();

    g_pid.refresh(Motor::MAX);
    g_pid.set_alpha(Pid::Alpha::from_raw(g_alpha_e));

    g_angle.set_alpha(Angle::Alpha::from_raw(g_alpha_y));
    g_current.set_alpha(CurrentSense::Alpha::from_raw(g_alpha_i));

    g_lut.apply(g_lutw);

    // Se recalcula siempre y no sólo al escribir la tabla: así `ang_lutsum`
    // describe lo que hay, incluso si alguien lo escribió a mano, y la computadora
    // puede verificar 64 entradas con una sola lectura.
    g_lutsum = g_lut.checksum();

    // El filtro del sensor, sólo cuando cambió: escribirlo en cada `set pid_kp`
    // pararía el muestreador sin motivo. Y sólo con el muestreador corriendo,
    // porque la lectura del CONF que precede a la escritura viaja en un tick de
    // muestreo. Si la escritura no sale, no se marca como aplicada y el intento
    // siguiente vuelve a probar.
    if (g_sfilt.want > Sensor::SF_2X)
    {
        g_sfilt.want = Sensor::SF_2X;
    }

    if (g_clock.running() && g_sfilt.want != g_sfilt.applied && apply_sensor_filter())
    {
        g_sfilt.applied = g_sfilt.want;
    }
}

// La visión que el propio AS5600 tiene del imán: detectado, muy débil, muy fuerte.
// Leerla le cuesta al lazo de muestreo una muestra, así que sólo se hace entre
// capturas; es decir, durante una verificación de puesta en marcha, que es la
// única vez que a alguien le interesa.
//
// Un registro por vez, por turnos. AGC y MAGNITUDE son contiguos y saldrían en una
// sola lectura de tres bytes, pero una lectura de tres bytes a 400 kHz no entra en
// el período de muestreo de 200 us, así que el tick siguiente encuentra el bus
// ocupado y se cuenta un desborde. Medido en el banco: 4,5 desbordes por segundo.
// Lo caro no es la muestra perdida: es que la puesta en marcha informaba una falla
// de bus en un equipo sano, y una verificación que grita en falso enseña a
// ignorarla. Ver SensorHealth::turn().
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

// Los contadores de salud describen la ventana de emisión, así que se ponen en
// cero cuando se abre una. Arrancarla cuesta unos milisegundos de puerto serie
// --el encabezado son siete líneas, y escribir bloquea en cuanto se llena el
// buffer de transmisión-- y los períodos que se pierden ahí son el precio de
// arrancar la captura, no una falla del lazo.
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
    // El reloj antes que nada: si la placa no corre a 16 MHz, el UART de acá abajo
    // emite al ritmo equivocado y ni el mensaje de error llega. Ver BoardStart.h.
    board::clock_begin();

    // El ancho del conversor primero, porque es lo que dice qué placa es; esa sonda
    // no necesita una referencia correcta, porque cae al CLKPR de arranque.
    g_board.adcfs = board::adc_full_scale();

    // Y enseguida la referencia, antes de cualquier cosa que mida. En una de las
    // dos placas del banco los bits REFS no la eligen, así que todo lo que se mida
    // antes de esta línea corre contra una referencia de resabio: el bandgap, y
    // sobre todo el estado eléctrico de las líneas del bus, que se juzga con
    // umbrales que son fracciones del fondo de escala.
    board::adc_select_reference(g_board.adcfs >= ADC_FULL, SENSE_REF_INTERNAL);

    g_board.bgadc = board::adc_bandgap();

    // La escala del canal de corriente, que depende de la referencia y por lo tanto
    // de la placa. Con Vcc las dos miden lo mismo; con la referencia interna no.
    {
        const float ref = (SENSE_REF_INTERNAL && g_board.adcfs >= ADC_FULL)
                        ? ADC_REF_MV_LGT8F : ADC_REF_MV;
        g_board.malsb = (uint16_t)(256.0f * 1000.0f * (ref / ADC_FULL)
                                   / SENSE_MV_PER_A + 0.5f);
    }

    // El estado eléctrico del bus, antes de que el TWI tome las líneas. Desde el
    // protocolo, un cable al aire, un módulo sin alimentación y un corto contra
    // masa se ven los tres igual --el sensor no contesta-- y se arreglan en lugares
    // distintos. Medirlo cuesta cuatro conversiones y una sola vez.
    g_board.bus = board::bus_check(g_board.adcfs);

    // Y la falla que sobrevive a todo lo anterior: los dos cables cambiados entre
    // sí. El bus se ve impecable y no contesta nadie. Se pregunta antes de
    // destrabar, que es lo que después deja el bus en un estado conocido.
    if (board::responds_swapped(Sensor::DEVICE_ADDRESS))
    {
        g_board.bus |= board::BUS_SWAPPED;
    }

    board::bus_recover();

    // El puente: deja ENA abierto y las dos entradas de sentido en bajo, que es el
    // estado del que parte write().
    g_motor.begin();

    // Si el proyecto trae una calibración compilada, entra acá y queda activa desde
    // el arranque. Sin ella la tabla es toda ceros y `ang_cal` arranca en 0: un
    // dispositivo sin calibrar tiene que decir que no está calibrado, no corregir
    // con lo que haya quedado.
#ifdef TIENE_CALIBRACION
    memcpy_P(g_lut.entry, CAL_LUT, sizeof(g_lut.entry));
    g_cal = 1;
#endif

    CtrlLink::set_id(F("ControlDemo"));
    CtrlLink::begin(BAUD,
                    g_params,   sizeof(g_params)   / sizeof(g_params[0]),
                    g_channels, sizeof(g_channels) / sizeof(g_channels[0]),
                    (uint32_t)g_clock.divide * 1000000UL / SAMPLE_HZ);

    refresh_tuning();

    g_adc.begin(g_board.adcfs, ADC_FULL, SENSE_REF_INTERNAL);
    Sensor::begin();
    g_clock.begin(SAMPLE_HZ);

    // El filtro del sensor se escribe recién ahora: la lectura del CONF que precede
    // a la escritura viaja en un tick de muestreo, así que antes de esta línea no
    // hay quién la lleve. refresh_tuning() lo sabe por g_clock.running().
    refresh_tuning();

    CtrlLink::note(F("ControlDemo listo"));
}

void loop()
{
    if (g_clock.take())
    {
        control_step();

        g_health.accumulate(Sensor::overruns(), Sensor::errors(), Sensor::present());

        CtrlLink::emit();
    }

    // Después del paso de control, nunca antes: un `set` que caiga justo cuando se
    // dispara un tick quedaría de otro modo por delante de él.
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
