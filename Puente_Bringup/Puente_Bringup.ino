// Puesta en marcha del accionamiento, sin depender del sensor de ángulo.
//
// Existe porque cuando el motor no gira hay dos culpables muy distintos --la
// placa no está sacando el comando, o el puente no lo está siguiendo-- y
// `bringup()` no puede separarlos: su única evidencia es el ángulo, y si el imán
// está mal montado el ángulo es ruido y todo parece un motor muerto. Acá la
// evidencia no sale del banco: la placa lee sus propios pines de vuelta.
//
// Primero se verifica a sí misma. Configura el Timer1 igual que ControlDemo y
// mira el registro PIN --que refleja el estado real del pin aunque sea salida--
// para confirmar que ENA conmuta y que IN1 e IN2 obedecen. Si eso pasa, lo que
// falta está del puente para afuera: su alimentación, su cableado, o el motor.
//
// Después acciona en secuencia lenta, anunciando cada estado por el puerto
// serie, para poder medir con un tester en las entradas y en las salidas del
// L298N sin correr atrás de la pantalla.
//
// Salida: 115200 baudios.
//
// OJO, el motor se mueve. Revisar que el eje esté libre antes de grabarlo.

#include <BoardStart.h>

static const uint8_t  PWM_PIN = 9;      // ENA del L298N, OC1A
static const uint8_t  IN1_PIN = 6;
static const uint8_t  IN2_PIN = 7;
static const uint16_t PWM_TOP = 8000;   // 1 kHz phase-correct, como ControlDemo

static void startMotorPwm(void)
{
    TCCR1A = _BV(COM1A1) | _BV(WGM11);  // modo 10: phase-correct, TOP = ICR1
    TCCR1B = _BV(WGM13) | _BV(CS10);    // preescalador /1
    TCNT1  = 0;
    ICR1   = PWM_TOP;
    OCR1A  = 0;
}

static void setDuty(uint8_t u)
{
    OCR1A = ((uint32_t)PWM_TOP * u) / 255;
}

// Mira un pin durante unos milisegundos y dice si lo vio en alto, en bajo, o en
// los dos --que es lo único que hace falta saber--. No devuelve un ciclo de
// trabajo: contar muestras en un lazo cerrado da un número sesgado, porque el
// período del lazo no es independiente del período del PWM. El ciclo de trabajo
// exacto ya se conoce, está en OCR1A.
//
// Y no se usa digitalRead(): el core de Arduino apaga el PWM del pin que se lee
// --llama a turnOffPWM()--, así que la primera lectura desconectaría la salida
// del temporizador y el diagnóstico informaría un pin muerto que él mismo mató.
// Se leen los registros de puerto a mano.
// Devuelve bit 0 si lo vio en alto y bit 1 si lo vio en bajo. Un uint8_t y no
// un par de bool: el preprocesador de Arduino genera los prototipos de las
// funciones antes de las definiciones de tipo del sketch, así que un struct
// propio en un valor de retorno no compila.
static const uint8_t VISTO_ALTO = 1;
static const uint8_t VISTO_BAJO = 2;

static uint8_t observar(uint8_t pin)
{
    volatile uint8_t* reg = (pin == PWM_PIN) ? &PINB : &PIND;
    const uint8_t mask = (pin == PWM_PIN) ? _BV(1)
                       : (pin == IN1_PIN) ? _BV(6) : _BV(7);

    uint8_t visto = 0;

    // Diez milisegundos son diez períodos del PWM: de sobra para ver los dos
    // estados si el pin está conmutando.
    const uint32_t hasta = micros() + 10000UL;
    while ((int32_t)(micros() - hasta) < 0)
    {
        visto |= (*reg & mask) ? VISTO_ALTO : VISTO_BAJO;
    }
    return visto;
}

static void informar(const __FlashStringHelper* que, uint8_t pin,
                     const __FlashStringHelper* esperado)
{
    const uint8_t visto = observar(pin);

    Serial.print(que);
    Serial.print(F(": "));
    if (visto == (VISTO_ALTO | VISTO_BAJO)) { Serial.print(F("conmutando")); }
    else if (visto == VISTO_ALTO)           { Serial.print(F("siempre en alto")); }
    else                                    { Serial.print(F("siempre en bajo")); }
    Serial.print(F("  (se espera "));
    Serial.print(esperado);
    Serial.println(')');
}

void setup()
{
    boardClockBegin();

    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println(F("Puesta en marcha del puente"));
    Serial.println(F("--- los pines, leidos de vuelta por la propia placa ---"));

    pinMode(PWM_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    startMotorPwm();

    setDuty(0);
    informar(F("ENA con u = 0  "), PWM_PIN, F("siempre en bajo"));
    setDuty(128);
    Serial.print(F("ENA con u = 128, OCR1A = "));
    Serial.print(OCR1A);
    Serial.print(F(" de "));
    Serial.print(ICR1);
    Serial.print(F(" -> ciclo de trabajo "));
    Serial.print((uint16_t)((uint32_t)OCR1A * 100 / ICR1));
    Serial.println('%');
    informar(F("ENA con u = 128"), PWM_PIN, F("conmutando"));
    // A fondo se espera «conmutando» y no «siempre en alto», y no es una falla:
    // con OCR1A en el TOP el PWM phase-correct deja igual una muesca de un par
    // de ciclos en el pico, así que el pin baja una vez por período durante unos
    // 125 ns. Al motor le llega el 99,97 % de la tensión y no la nota; a este
    // muestreo, que mira el pin y no el promedio, se le aparece. Lo que sí sería
    // una falla es que a fondo no conmutara *ni* estuviera en alto.
    setDuty(255);
    informar(F("ENA con u = 255"), PWM_PIN, F("conmutando o siempre en alto"));
    setDuty(0);

    digitalWrite(IN1_PIN, HIGH);
    informar(F("IN1 en alto     "), IN1_PIN, F("siempre en alto"));
    digitalWrite(IN1_PIN, LOW);
    informar(F("IN1 en bajo     "), IN1_PIN, F("siempre en bajo"));
    digitalWrite(IN2_PIN, HIGH);
    informar(F("IN2 en alto     "), IN2_PIN, F("siempre en alto"));
    digitalWrite(IN2_PIN, LOW);
    informar(F("IN2 en bajo     "), IN2_PIN, F("siempre en bajo"));

    Serial.println();
    Serial.println(F("Si todo eso dio lo esperado, la placa esta sacando el comando"));
    Serial.println(F("y lo que falta esta del puente para afuera: su alimentacion de"));
    Serial.println(F("potencia, el cableado de ENA/IN1/IN2, o el motor."));
    Serial.println();
    Serial.println(F("--- secuencia lenta, 3 s por estado, para medir con tester ---"));
}

void loop()
{
    struct Paso
    {
        uint8_t u;
        uint8_t in1;
        uint8_t in2;
        const __FlashStringHelper* texto;
    };

    const Paso pasos[] =
    {
        { 0,   LOW,  LOW,  F("todo apagado") },
        { 255, HIGH, LOW,  F("ENA a fondo, IN1=1 IN2=0  (tiene que girar)") },
        { 0,   LOW,  LOW,  F("todo apagado") },
        { 255, LOW,  HIGH, F("ENA a fondo, IN1=0 IN2=1  (el otro sentido)") },
        { 128, HIGH, LOW,  F("ENA al 50%, IN1=1 IN2=0") },
    };

    for (uint8_t i = 0; i < sizeof(pasos) / sizeof(pasos[0]); i++)
    {
        digitalWrite(IN1_PIN, pasos[i].in1);
        digitalWrite(IN2_PIN, pasos[i].in2);
        setDuty(pasos[i].u);
        Serial.println(pasos[i].texto);
        delay(3000);
    }
}
