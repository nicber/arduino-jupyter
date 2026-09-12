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
// omisión vale 500 Hz. De esa manera el muestreo mantiene un período rígido aun
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
// La tabla de canales por omisión son 45 bytes por fila, o el 23 % del enlace a
// 500 Hz, que es el "bastante por debajo de la mitad" que le gusta a este
// protocolo. Bajar `tickdiv` a 5 devuelve el lazo a 1 kHz y lleva la fila al
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
// los dos salen positivos. Eso es una propiedad del banco, no de la aritmética, y
// `iinvert` es la perilla que la reconcilia: un comando positivo tiene que dar una
// corriente positiva.
static const uint8_t  SENSE_CHANNEL   = 0;

// La sensibilidad del sensor, que es lo único que convierte cuentas en amperes.
// 185 mV/A es un ACS712-05B conectado directo, que es como está pensado el banco.
// OJO si no cierra con lo que mide un tester en serie con el motor: un reposo muy
// por debajo de los 2500 mV que da un ACS712 alimentado a 5 V delata un divisor en
// la salida, y un divisor divide las dos cosas a la vez --el cero y la
// sensibilidad--, así que ahí va 185 dividido por lo mismo. Ver README.
static const float    SENSE_MV_PER_A  = 185.0f;

// La referencia del ADC, que es la única perilla de ganancia que tiene el AVR de
// este lado, y es una elección entre techo y resolución.
//
// Por omisión va la referencia alta, que es Vcc, y no sólo por el techo. Un ACS712
// es un sensor bipolar y ratiométrico: reposa en la mitad de su alimentación
// --2,5 V con 5 V-- para poder bajar cuando la corriente cambia de sentido. Medir
// esa salida contra Vcc es medirla contra la misma tensión que la produce, así que
// el reposo cae en media escala por construcción, valga Vcc 4,8 o 5,1 y sea cual
// sea la placa. No hay un cero por placa que averiguar: son 2048 cuentas y listo,
// que es lo que dice SENSE_ZERO acá abajo.
//
// Contra la referencia interna de 1,1 V, en cambio, ese mismo sensor satura en
// reposo: no mide nada, y desde el ADC se ve igual que una entrada al aire.
//
// Vcc es REFS=01 en las dos placas del banco. En el ATmega es AVcc, y en el
// LGT8F328P es lo que el core lgt8fx llama DEFAULT, que también vale 1. Las otras
// tres que ese core declara por nombre --INTERNAL1V024 = 3, INTERNAL2V048 = 2,
// INTERNAL4V096 = 4-- no cambian esto, y la de 4,096 ni siquiera entra en los dos
// bits de REFS.
//
// Y elegirla en el clon no es escribir REFS: ver startAdcReference().
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
// Lo que se paga es resolución: un LSB pasa de 1,07 mV a 4,9, y con 185 mV/A eso
// deja 200 mA en apenas siete u ocho cuentas. Es el precio de poder medir el
// sensor que está puesto.
//
// SENSE_REF_INTERNAL en true vuelve a la interna, y sirve para un sensor unipolar
// --el que va en la alimentación del puente-- que reposa cerca de cero y no
// necesita techo, o para uno bipolar con un divisor a la salida. Ahí sí conviene:
// son 4,5 veces más resolución sobre la misma señal.
//
// SENSE_ZERO es sólo el punto de partida de `izero` --media escala con la alta,
// cero con la interna--; el cero de verdad lo mide la computadora. Ver
// Bench.zero_current().
//
// La interna no vale 1,100 V: el bandgap está especificado entre 1,0 y 1,2 V, o
// sea +/-10 % de error de ganancia de chip a chip, y ese error va derecho a los mA
// que se informan. La placa se mide a sí misma y publica el resultado en `bgadc`:
// es el bandgap leído contra AVcc, en cuentas. Falta una sola tensión conocida
// para cerrar la cuenta, porque acá adentro no hay ninguna, así que AVcc se mide
// una vez con un tester y entonces la referencia interna vale
// AVcc * bgadc / adcfs. `bringup()` hace esa cuenta y dice qué número poner acá.
// ADC_REF_MV es por placa --acá son los 5006 mV de AVcc medidos en el UNO de este
// banco-- y en este banco hay dos placas.
static const bool     SENSE_REF_INTERNAL = false;
static const float    ADC_REF_MV      = SENSE_REF_INTERNAL ? 1093.0f : 5006.0f;

// Todo lo que sigue cuenta en cuentas de 12 bits, en las dos placas del banco.
//
// El UNO tiene un ADC de 10 bits y el clon con LGT8F328P uno de 12, así que la
// misma tensión mide cuatro veces más en una que en la otra. Hay dos maneras de
// emparejarlas y no son equivalentes: tirar los dos bits de abajo del clon, o
// correr la lectura del UNO dos bits para arriba y dejar esos dos LSBs en cero.
// Se elige la segunda. La primera deja las dos placas en 10 bits y tira
// resolución que en este canal escasea --un ACS712-05B da 185 mV/A, y 200 mA son
// 37 mV--; la segunda conserva lo que el clon mide de verdad y le cuesta al UNO
// dos ceros al final de un número que igual no los tenía. Las escalas de abajo
// son entonces una sola, y el corrimiento vive en un único lugar: g_adcshift.
static const uint16_t ADC_FULL        = 4096;
static const int16_t  SENSE_ZERO      = SENSE_REF_INTERNAL ? 0 : (ADC_FULL / 2);
static const float    ADC_MV_PER_LSB  = ADC_REF_MV / ADC_FULL;
static const float    SENSE_MA_PER_LSB = 1000.0f * ADC_MV_PER_LSB / SENSE_MV_PER_A;

// Y la referencia tampoco vale lo mismo en las dos placas, así que la escala de
// arriba es la del ATmega y nada más. La tabla de canales viaja en flash con una
// constante compilada y no se puede corregir al arrancar; lo que sí se calcula al
// arrancar es `imalsb`, y con eso la computadora corrige la escala de `i`. Ver
// Bench._read_channels().
//
// En el ATmega la referencia interna es el bandgap, especificado entre 1,0 y
// 1,2 V: el número es de esta placa y no del modelo, y son los 1093 mV medidos en
// el UNO de este banco.
//
// En el clon la interna es una referencia trimada de fábrica y vale 1,024 V, no
// los 1,1 del ATmega. Son un 6 % de diferencia, no un factor de cuatro: el factor
// de cuatro era el ancho del conversor y ya está corregido más arriba.
//
// Esta constante es sólo para esa elección. La alta no la necesita: las dos placas
// corren a 5 V y la referencia alta es Vcc en las dos, así que ADC_REF_MV sirve
// para ambas.
static const float    ADC_REF_MV_LGT8F = 1024.0f;

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
static uint8_t g_tickdiv = 10;  // muestras de 5 kHz por período de control: 10 -> 500 Hz
static uint8_t g_bidir   = 1;   // 1: el puente acciona en los dos sentidos
// 1: un comando positivo hace bajar el ángulo medido. Es una propiedad del
// cableado --de qué lado están los cables del motor en el puente, y de qué lado
// mira el imán al sensor-- así que este valor describe un banco y no una verdad
// general, y cambia cada vez que alguien da vuelta el imán o los cables del
// motor. `bringup()` lo verifica en cada corrida y dice cuál poner.
//
// Y verificarlo exige que el eje esté quieto antes de medir: con el puente
// abierto el motor no frena, sigue por inercia varios segundos, y midiendo
// enseguida lo que se mide es el giro anterior. Así este parámetro pareció
// roto -- dos veredictos opuestos en la misma tarde-- hasta que se lo midió con
// el eje realmente parado. Ver Bench.spin().
static uint8_t g_uinvert = 0;

// 1: la corriente medida sale negativa cuando un comando positivo la hace
// circular. Es el hermano de `uinvert` para el otro sensor, y hace falta que sean
// dos porque cada uno arregla algo que el otro no puede: `uinvert` da vuelta el
// puente, así que da vuelta el ángulo y la corriente a la vez. Si después de
// acertarle al ángulo la corriente sigue saliendo al revés, lo que está dado
// vuelta es por dónde entra el sensor de corriente, y eso sólo se arregla acá --o
// dándolo vuelta a mano.
//
// Importa porque `i` no es sólo telemetría: con target = CURRENT el lazo cierra
// sobre ella, y realimentar con el signo cambiado no se establece, se escapa.
//
// En un sensor unipolar --el que va en la alimentación del puente-- los dos
// sentidos de giro salen positivos y esto no arregla nada, porque no hay nada que
// arreglar. `bringup()` distingue los dos casos midiendo.
static uint8_t g_iinvert = 0;

static uint16_t g_pwmtop = PWM_TOP_DEFAULT;   // TOP del Timer1: f = 8 MHz / pwmtop

// Calibración del sensor. `cal` prende y apaga la corrección en caliente, que es
// lo que permite medir cuánto sirve en lugar de suponerlo.
static uint8_t  g_cal    = 0;
static uint8_t  g_sfilt  = 3;   // filtro lento del AS5600: 3 es 2x, el más rápido

// Una entrada de la tabla por escritura, empaquetada como (índice << 16) | valor.
// El índice viaja adentro del mismo valor para que dos escrituras seguidas nunca
// sean iguales por casualidad: el sketch aplica la escritura al ver que este
// parámetro cambió, y con índice y valor en parámetros separados una tabla con
// dos entradas iguales seguidas perdería la segunda. 0xFFFFFFFF es "nada que
// hacer", porque ese índice no existe; de ahí sale el valor de arranque.
//
// Son 32 bits y no 16 porque el valor pasó a ser int16: el índice ya no entra en
// el byte alto.
static uint32_t g_lutw   = 0xFFFFFFFFUL;
static uint16_t g_lutsum = 0;   // suma de Fletcher de la tabla; la computadora la verifica

static uint16_t g_y_raw = 0;    // cuenta cruda del sensor, 0..4095, sin corregir
static int16_t g_y     = 0;     // ángulo medido, cuentas, con el offset aplicado
static int32_t g_y_uw  = 0;     // ángulo desenrollado, cuentas, sin filtrar
static int32_t g_y_uwf = 0;     // ángulo desenrollado, cuentas, filtrado por alpha_y
static int16_t g_i     = 0;     // corriente, LSBs del ADC alrededor de izero, filtrada
static int16_t g_e     = 0;     // error, unidades del target, recortado para la telemetría
static int16_t g_u     = 0;     // comando al actuador, u_min..U_MAX

// Contadores de salud. Todos los puede escribir la computadora, así que poner uno
// en cero reinicia esa cuenta.
// Si el muestreador de 5 kHz ya está corriendo. Lo consulta refresh_tuning()
// para saber si puede hablarle al sensor: una lectura de mantenimiento viaja en
// un tick de muestreo, así que antes del primer tick no hay quién la lleve.
static bool g_sampling = false;

static uint16_t g_maxlate = 0;  // peor retardo observado entre la ISR y su atención, us
static uint16_t g_missed  = 0;  // períodos de control que loop() nunca atendió
static uint16_t g_sovr    = 0;  // muestras del sensor que el bus I2C no llegó a seguir
static uint16_t g_serr    = 0;  // transferencias del sensor que fallaron
static uint8_t  g_mstat   = 0;  // registro STATUS del AS5600: imán presente, muy débil, muy fuerte
static uint8_t  g_spres   = 1;  // el sensor contesta en el bus

// Diagnóstico de montaje. `agc` a media escala es la única evidencia barata de
// que el imán está a la distancia correcta: contra un extremo quiere decir
// demasiado lejos o demasiado cerca, y ahí ninguna tabla arregla nada. Se leen
// junto con STATUS, entre corridas.
static uint8_t  g_agc     = 0;  // registro AGC del AS5600, 0..255 a 5 V
static uint16_t g_mag     = 0;  // registro MAGNITUDE: módulo del vector de campo

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

// Propiedades de la placa, medidas al arrancar y publicadas como parámetros para
// que la computadora no tenga que adivinarlas. Ver BoardStart.h.
static uint16_t g_adcfs    = ADC_FULL;  // fondo de escala real del ADC de esta placa
static uint8_t  g_adcshift = 0;         // cuánto se corre cada lectura para llegar a 12 bits
static uint16_t g_bgadc    = 0;         // el bandgap contra AVcc, en cuentas
static uint8_t  g_busdiag  = 0;         // estado eléctrico del bus I2C al arrancar
static uint16_t g_imalsb   = 0;         // mA por cuenta de `i` en esta placa, en Q8
static volatile uint8_t  g_divider    = 10;

// --------------------------------------------------------- calibración del AS5600

// El AS5600 no mide el ángulo que uno cree. Un imán descentrado respecto del
// integrado --la hoja de datos pide ±0,25 mm-- corre la lectura en una cantidad
// que depende del ángulo y que se repite vuelta tras vuelta: al lazo se le
// presenta como ondulación de velocidad, y ninguna ganancia la saca. La forma del
// error son los primeros armónicos de la vuelta mecánica; ver
// Docs/CALIBRACION_AS5600.md, que además explica cómo se lo mide sin tener un
// encoder de referencia.
//
// Acá vive nada más que la corrección: una tabla indexada por el ángulo crudo.
// Quién la calcula y de dónde sale es asunto de la computadora.
static const uint8_t LUT_SIZE = 64;

// Entradas en octavos de cuenta. La resolución de un octavo existe porque el
// error puede ser de unas pocas cuentas: en cuentas enteras la tabla tendría tres
// o cuatro valores distintos y sería un escalón, no una corrección.
//
// El tipo es int16 y no int8, que era lo primero que hubo acá. Un int8 en octavos
// llega a ±15,9 cuentas --±1,4 grados-- que es de sobra para lo que promete la
// hoja de datos, y la idea era que un error más grande que eso fuera un imán mal
// puesto y no algo para corregir por tabla. El banco dijo otra cosa: con el AGC
// en 114 de 255 --media escala, la distancia correcta-- el segundo armónico mide
// 105 cuentas, 9,3 grados. El AGC informa la distancia, no el centrado, así que
// un imán puede estar a la distancia justa y de todos modos torcido, y ahí el
// error es real y grande. Un int8 no podía representarlo y la tabla recortaba el
// noventa por ciento.
//
// El costo son 64 bytes más de SRAM, de los 1400 que quedaban.
static const int16_t LUT_MAX = 4095;   // octavos: ±511 cuentas, ±45 grados

static int16_t g_lut[LUT_SIZE];

// La tabla NO se guarda en la placa. El dispositivo arranca siempre sin
// calibrar, y quien tiene la tabla es la computadora, que la empuja al conectarse
// --ver python/calib.py--. Es una decisión, no una limitación de memoria:
//
//   - Una calibración es una propiedad del *banco* --este imán, en este eje, con
//     este sensor--, no de la placa. En un archivo se lee, se compara, se revisa
//     y entra en el repositorio; en la EEPROM es estado invisible que sobrevive
//     a la reprogramación y que nadie recuerda haber puesto.
//   - Un dispositivo que arranca sin corregir no puede mentirle a nadie. El caso
//     feo de la EEPROM no es la tabla que falta: es la tabla vieja, de otro
//     montaje, que se aplica en silencio.
//   - Y para el aula: la corrección se prende y se apaga con `cal` mientras el
//     motor gira. Eso es lo que hace que se pueda mostrar.
//
// Para dejarla fija en un tablero que se enciende solo, `calib.escribir_header()`
// genera Calibracion.h y este sketch lo toma si está.
#if defined(__has_include)
#  if __has_include("Calibracion.h")
#    include "Calibracion.h"
#    define TIENE_CALIBRACION 1
#  endif
#endif

// Suma de Fletcher de 16 bits sobre la tabla. La computadora la calcula por su
// lado y la compara con `lutsum`, así que las 64 escrituras se verifican con una
// sola lectura. Fletcher y no una suma pelada porque una suma no distingue una
// tabla de otra con dos entradas intercambiadas, y una entrada en el índice
// equivocado es exactamente el error que se comete acá.
// Se recorre byte por byte, primero el bajo y después el alto de cada entrada,
// para que la computadora pueda reproducirla sin saber nada del orden de bytes
// del AVR.
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
// separación, ocho puntos por ciclo del octavo armónico; la interpolación se
// hace cargo del resto.
static int16_t lut_lookup(int16_t counts)
{
    uint8_t i    = (uint8_t)(counts >> 6) & (LUT_SIZE - 1);
    uint8_t frac = (uint8_t)counts & 0x3F;

    int32_t a = (int32_t)g_lut[i];
    int32_t b = (int32_t)g_lut[(uint8_t)(i + 1) & (LUT_SIZE - 1)];

    // Un int32 en el medio y no un int16: con entradas de hasta ±4095 octavos la
    // suma llega a 4095*64 = 262080, que no entra en 16 bits. Son unas decenas de
    // ciclos más, una vez por período de control.
    int32_t eighths = (a * (int32_t)(64 - frac) + b * (int32_t)frac) >> 6;

    // Redondeo al medio hacia arriba. Con corrimiento aritmético `(e + 4) >> 3`
    // sirve para los dos signos; el `e < 0 ? -4 : 4` que uno escribe de reflejo
    // redondea mal los negativos chicos --3/8 daría -1 en lugar de 0--.
    return (int16_t)((eighths + 4) >> 3);
}

// Escribe los bits SF del CONF del sensor.
//
// El muestreador se para para escribir: nI2C encola la escritura y la completa
// su propia ISR, pero encolar reserva memoria y el muestreador de 5 kHz llama a
// nI2C desde una ISR de temporizador. Con el muestreador quieto no hay nadie más
// pidiendo el bus. Cuesta un par de milisegundos de lazo detenido, y pasa sólo
// cuando alguien mueve el parámetro.
//
// Lee-modifica-escribe en lugar de escribir la palabra entera: CONF también
// lleva la histéresis, el modo de potencia y la salida, y ninguno de ésos es
// asunto de este parámetro.
static bool apply_sensor_filter(void)
{
    uint8_t conf[2];

    // La lectura sí pasa por el lazo de muestreo, así que va con el muestreador
    // todavía corriendo.
    if (!Sensor::read_registers(Sensor::REG_CONF_H, conf, 2))
    {
        return false;
    }

    uint16_t value = ((uint16_t)conf[0] << 8) | conf[1];
    value = (uint16_t)((value & ~0x0300u) | ((uint16_t)(g_sfilt & 0x03) << 8));

    conf[0] = (uint8_t)(value >> 8);
    conf[1] = (uint8_t)value;

    TIMSK2 &= ~_BV(OCIE2A);

    // Dejar terminar la transferencia que ya estaba en el aire. Acotado: sin
    // sensor en el bus esto no puede quedarse esperando para siempre.
    uint32_t deadline = millis() + 5;
    while (Sensor::busy() && (int32_t)(millis() - deadline) < 0)
    {
    }

    bool queued = Sensor::write_registers(Sensor::REG_CONF_H, conf, 2);

    // Cuatro bytes a 400 kHz son unos 100 us; dos milisegundos es holgura, no
    // cálculo.
    delay(2);

    TIMSK2 |= _BV(OCIE2A);

    return queued;
}

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
    { "iinvert", CTRL_U8,  &g_iinvert,  0           },
    { "pwmtop",  CTRL_U16, &g_pwmtop,   0           },
    { "cal",     CTRL_U8,  &g_cal,      0           },
    { "sfilt",   CTRL_U8,  &g_sfilt,    0           },
    { "lutw",    CTRL_U32, &g_lutw,     0           },
    { "lutsum",  CTRL_U16, &g_lutsum,   0           },
    { "y",       CTRL_I16, &g_y,        0           },
    { "y_uw",    CTRL_I32, &g_y_uw,     0           },
    { "mstat",   CTRL_U8,  &g_mstat,    0           },
    { "spres",   CTRL_U8,  &g_spres,    0           },
    { "agc",     CTRL_U8,  &g_agc,      0           },
    { "mag",     CTRL_U16, &g_mag,      0           },
    { "maxlate", CTRL_U16, &g_maxlate,  0           },
    { "missed",  CTRL_U16, &g_missed,   0           },
    { "sovr",    CTRL_U16, &g_sovr,     0           },
    { "serr",    CTRL_U16, &g_serr,     0           },
    { "adcfs",   CTRL_U16, &g_adcfs,    0           },
    { "bgadc",   CTRL_U16, &g_bgadc,    0           },
    { "busdiag", CTRL_U8,  &g_busdiag,  0           },
    { "imalsb",  CTRL_U16, &g_imalsb,   8           },
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
    // La cuenta cruda, sin `offset`, sin el signo invertido de `y` y sin
    // corregir. Es lo que indexa la tabla de calibración, así que es lo que la
    // computadora necesita para calcularla: reconstruirla desde `y_uw` se puede,
    // pero deshacer un signo y un offset a mano es justo el lugar donde una
    // calibración sale espejada y nadie se da cuenta hasta el final.
    { "y_raw", CTRL_U16, &g_y_raw, COUNTS_TO_DEG,      "deg" },
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
// 1 kHz sale exacto (TOP = 8000) y entran dos períodos suyos en cada período del
// lazo, así que los dos quedan enganchados en fase en lugar de batir: el
// muestreador de 5 kHz toma siempre las mismas cinco fases de la ondulación, lo
// que da un sesgo fijo en `i` en lugar de una oscilación lenta. Con un puente
// MOSFET --un TB6612FNG, un DRV8833-- nada de esto haría falta y
// `dev.pwm(20000)` sería lo correcto.
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
// En el LGT8F328P los bits REFS del ADMUX NO eligen la referencia. La eligen
// DACON, VCAL y el bit REFS2 de ADCSRD, y REFS queda de resabio porque el core
// lgt8fx lo escribe igual --`ADMUX = analog_reference << 6`-- después de haber
// configurado los otros tres. Ver analogReference() en su wiring_analog.c.
//
// Un sketch que escriba sólo REFS, como hacía éste, no elige nada en esa placa: la
// referencia queda en lo que haya quedado de antes. Eso explica por qué la misma
// lectura del canal interno daba números distintos en corridas distintas, que está
// anotado en BoardStart.h como una rareza y es en realidad esto.
//
// Las dos placas del banco corren el mismo binario y el core es el del ATmega, así
// que estos registros no existen por nombre y van por dirección. Sólo se los toca
// cuando la placa es la del ADC de 12 bits: en el UNO 0xA0 y 0xAD no son
// registros, y no hay por qué escribirles.
static const uint16_t LGT_DACON  = 0xA0;
static const uint16_t LGT_ADCSRD = 0xAD;
static const uint16_t LGT_VCAL   = 0xC8;
static const uint16_t LGT_VCAL1  = 0xCD;   // el valor de calibración de 1,024 V
static const uint8_t  LGT_REFS2  = 6;

static void startAdcReference(void)
{
    if (g_adcfs < ADC_FULL)
    {
        return;                 // un ATmega: los bits REFS alcanzan y son los suyos
    }

    _SFR_MEM8(LGT_ADCSRD) &= (uint8_t)~_BV(LGT_REFS2);

    if (SENSE_REF_INTERNAL)
    {
        // La referencia interna, con VCAL cargado con la calibración de 1,024 V.
        _SFR_MEM8(LGT_DACON) = (uint8_t)((_SFR_MEM8(LGT_DACON) & 0x0C) | 0x02);
        _SFR_MEM8(LGT_VCAL)  = _SFR_MEM8(LGT_VCAL1);
    }
    else
    {
        // DEFAULT del core: Vcc, que es la misma elección que REFS=01 en el UNO.
        _SFR_MEM8(LGT_DACON) &= 0x0C;
    }
}

static void startAdc(void)
{
    startAdcReference();

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
// La cuenta del sensor, corregida si hay calibración, en el dominio crudo y
// antes de desenrollar.
//
// Antes de desenrollar porque la tabla se indexa con el ángulo dentro de la
// vuelta, y una vez desenrollado ese ángulo ya no está. Y sobre la cuenta cruda
// y no sobre `y`, porque `y` lleva el signo invertido y el offset: dos
// oportunidades de equivocarse a cambio de nada.
//
// g_lut se toca acá y en refresh_tuning(), las dos desde loop(), así que no hay
// nada que sincronizar. Si alguna vez la corrección se mudara a la ISR de
// muestreo, dejaría de ser cierto.
static int16_t sensor_measurement(void)
{
    int16_t counts = (int16_t)Sensor::counts();

    g_y_raw = (uint16_t)counts;

    if (g_cal)
    {
        counts = (int16_t)((counts - lut_lookup(counts)) & (COUNTS_PER_REV - 1));
    }

    return wrapped_error(g_offset, counts);
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

    // El offset primero y el signo después: `izero` está en cuentas del ADC, que
    // es el dominio en el que la computadora lo mide, así que restarlo no puede
    // depender de hacia dónde se cuente después. Ver Bench.zero_current(), que
    // deshace esta misma composición para calcular el cero.
    int16_t sensed = (int16_t)(adc - g_izero);

    if (g_iinvert)
    {
        sensed = (int16_t)-sensed;
    }

    g_i = clamp16(g_i_filt[1].update(g_i_filt[0].update(sensed)),
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

    // Una entrada de la tabla de calibración por escritura de `lutw`. Aplicar al
    // ver que el parámetro cambió es lo que permite cargar la tabla sin
    // agregarle un comando al protocolo: son 64 `set` comunes, cada uno con la
    // misma respuesta verificada que cualquier otro. Ver g_lutw.
    static uint32_t lutw_applied = 0xFFFFFFFFUL;

    if (g_lutw != lutw_applied)
    {
        lutw_applied = g_lutw;

        uint16_t index = (uint16_t)(g_lutw >> 16);
        int16_t  value = (int16_t)(uint16_t)g_lutw;

        if (index < LUT_SIZE)
        {
            // Acotar acá y no confiar: una entrada fuera de rango desbordaría la
            // interpolación, y el enlace acepta cualquier entero que le manden.
            if (value >  LUT_MAX) value =  LUT_MAX;
            if (value < -LUT_MAX) value = -LUT_MAX;

            g_lut[index] = value;
        }
    }

    // Se recalcula siempre y no sólo al escribir la tabla: así `lutsum` describe
    // lo que hay, incluso si alguien lo escribió a mano, y la computadora puede
    // verificar 64 entradas con una sola lectura.
    g_lutsum = lut_checksum();

    // El filtro del sensor, sólo cuando cambió: escribirlo en cada `set kp`
    // pararía el muestreador sin motivo. Y sólo con el muestreador corriendo,
    // porque la lectura del CONF que precede a la escritura viaja en un tick de
    // muestreo. Si la escritura no sale, no se marca como aplicada y el intento
    // siguiente vuelve a probar.
    static uint8_t sfilt_applied = 0xFF;

    if (g_sfilt > Sensor::SF_2X)
    {
        g_sfilt = Sensor::SF_2X;
    }

    if (g_sampling && g_sfilt != sfilt_applied && apply_sensor_filter())
    {
        sfilt_applied = g_sfilt;
    }
}

// La visión que el propio AS5600 tiene del imán: detectado, muy débil, muy
// fuerte. Leerla le cuesta al lazo de muestreo una muestra y bloquea acá hasta
// que esa muestra llegue, así que sólo se hace entre capturas; es decir, durante
// una verificación de puesta en marcha, que es la única vez que a alguien le
// interesa.
static void refresh_magnet_status(void)
{
    static uint32_t last_ms = 0;
    static bool     completo = false;

    // Los tres registros se leen por turnos --ver abajo--, así que el diagnóstico
    // entero tarda tres refrescos en llenarse. A 500 ms eso son un segundo y
    // medio de arranque en los que `agc` todavía vale cero, y quien pregunte
    // enseguida --`bringup()` lo hace-- lee un cero y lo informa como un imán
    // contra el borde. Así que la primera vuelta va rápido y recién después se
    // afloja al ritmo de algo que se mira entre corridas.
    if (CtrlLink::streaming() || (millis() - last_ms) < (completo ? 500u : 50u))
    {
        return;
    }
    last_ms = millis();

    if (!Sensor::present())
    {
        // Sin sensor en el bus no hay nada que informar del imán, y dejar el
        // último valor sería peor que no decir nada.
        g_mstat = 0;
        g_agc   = 0;
        g_mag   = 0;
        return;
    }

    // Un registro por vez, por turnos.
    //
    // AGC y MAGNITUDE son contiguos y salen en una sola lectura de tres bytes,
    // que es lo que se hacía acá. Pero una lectura de tres bytes a 400 kHz no
    // entra en el período de muestreo de 200 us, así que el tick siguiente
    // encuentra el bus todavía ocupado y se cuenta un desborde. Medido en el
    // banco: 4,5 desbordes por segundo, o sea el 0,09 % de las muestras. Lo caro
    // no es la muestra perdida: es que `bringup` informaba una falla de bus en un
    // equipo sano, y una verificación que grita en falso enseña a ignorarla.
    //
    // De a un registro por refresco, cada lectura entra en su período y el
    // diagnóstico completo se renueva cada segundo y medio, que para algo que se
    // mira entre corridas sobra.
    static uint8_t turno = 0;

    uint8_t buf[2];

    switch (turno)
    {
        case 0:
            if (Sensor::read_registers(Sensor::REG_STATUS, buf, 1))
            {
                g_mstat = buf[0];
            }
            break;

        case 1:
            if (Sensor::read_registers(Sensor::REG_AGC, buf, 1))
            {
                g_agc = buf[0];
            }
            break;

        default:
            if (Sensor::read_registers(Sensor::REG_MAGNITUDE_H, buf, 2))
            {
                g_mag = (uint16_t)((((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF);
            }
            break;
    }

    turno = (turno + 1) % 3;

    if (turno == 0)
    {
        completo = true;
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
    // El reloj antes que nada: si la placa no corre a 16 MHz, el UART de acá
    // abajo emite al ritmo equivocado y ni el mensaje de error llega. Y el bus
    // enseguida después: grabar la placa la resetea, y un reset en medio de una
    // lectura deja al AS5600 sujetando SDA, con lo que el muestreador arranca
    // trabado para siempre. Ver BoardStart.h.
    boardClockBegin();

    // El ADC antes que nada de lo que sigue. Su fondo de escala decide con qué
    // umbrales se juzgan las líneas del bus, y las sondas mueven el multiplexor,
    // así que las tres cosas tienen que pasar antes de que startAdc() lo deje
    // donde el lazo lo necesita.
    g_adcfs    = boardAdcFullScale();
    g_adcshift = (g_adcfs >= ADC_FULL) ? 0 : 2;
    g_bgadc    = boardAdcBandgap();

    // La escala del canal de corriente, que depende de la referencia y por lo
    // tanto de la placa. Con AVcc las dos miden lo mismo; con la referencia
    // interna no.
    {
        const float ref = (SENSE_REF_INTERNAL && g_adcfs >= ADC_FULL)
                        ? ADC_REF_MV_LGT8F : ADC_REF_MV;
        g_imalsb = (uint16_t)(256.0f * 1000.0f * (ref / ADC_FULL)
                              / SENSE_MV_PER_A + 0.5f);
    }

    // El estado eléctrico del bus, antes de que el TWI tome las líneas. Desde el
    // protocolo, un cable al aire, un módulo sin alimentación y un corto contra
    // masa se ven los tres igual --el sensor no contesta-- y se arreglan en
    // lugares distintos. Medirlo cuesta cuatro conversiones y una sola vez.
    g_busdiag = i2cBusCheck(g_adcfs);

    // Y la falla que sobrevive a todo lo anterior: los dos cables cambiados entre
    // sí. El bus se ve impecable y no contesta nadie. Se pregunta antes de
    // destrabar, que es lo que después deja el bus en un estado conocido.
    if (i2cRespondeInvertido(Sensor::DEVICE_ADDRESS)) {
        g_busdiag |= I2C_BUS_INVERTIDO;
    }

    i2cBusRecover();

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

    // Si el proyecto trae una calibración compilada, entra acá y queda activa
    // desde el arranque. Sin ella la tabla es toda ceros y `cal` arranca en 0:
    // un dispositivo sin calibrar tiene que decir que no está calibrado, no
    // corregir con lo que haya quedado.
#ifdef TIENE_CALIBRACION
    memcpy_P(g_lut, CAL_LUT, sizeof(g_lut));
    g_cal = 1;
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

    // El filtro del sensor se escribe recién ahora: la lectura del CONF que
    // precede a la escritura viaja en un tick de muestreo, así que antes de esta
    // línea no hay quién la lleve. refresh_tuning() lo sabe por g_sampling.
    g_sampling = true;
    refresh_tuning();

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
