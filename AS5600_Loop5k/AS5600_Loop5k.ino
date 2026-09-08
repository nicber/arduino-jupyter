// AS5600 muestreado a 5 kHz desde una ISR de temporizador, sobre un bus I2C
// gobernado por interrupciones.
//
// Hardware:
// Arduino UNO
// Sensor de posición de efecto Hall: AS5600 (I2C)
//
// Conexionado: SDA -> A4, SCL -> A5, VDD -> 5V, GND -> GND
//
// Salida: 115200 baudios, dos líneas de resumen por segundo. Imprimir cada
// muestra no es posible a esta frecuencia: 5 kHz de texto necesitan del orden de
// 2 Mbaud.

#include <nI2C.h>

#include <AS5600.h>
#include <NI2CBus.h>

typedef AS5600<NI2CBus> Sensor;

// Timer2, CTC, preescalador 32: 16 MHz / 32 / 100 = exactamente 5,000 kHz.
// El Timer2 deja en paz a millis() (Timer0) y a Servo (Timer1), pero choca con tone().
static void startSampleTimer(void)
{
    TCCR2A = _BV(WGM21);                // CTC, TOP = OCR2A
    TCCR2B = _BV(CS21) | _BV(CS20);     // preescalador /32
    OCR2A = 99;
    TCNT2 = 0;
    TIMSK2 = _BV(OCIE2A);
}

ISR(TIMER2_COMPA_vect)
{
    Sensor::do_transfer();
}

// ------------------------------------------------------------------ telemetría

static void printAngle(void)
{
    static uint16_t last_samples = 0;
    static uint32_t last_ms = 0;

    uint32_t now_ms = millis();
    uint16_t samples = Sensor::samples();

    // La vuelta al cero de los enteros sin signo hace que la diferencia sea
    // correcta aunque el contador sea de 16 bits y dé la vuelta cada ~13 s a
    // 5 kHz.
    uint16_t delta = samples - last_samples;
    uint32_t elapsed_ms = now_ms - last_ms;

    last_samples = samples;
    last_ms = now_ms;

    if (elapsed_ms == 0)
    {
        return;
    }

    uint16_t counts = Sensor::counts();

    Serial.print(F("frecuencia="));
    Serial.print((uint32_t)delta * 1000UL / elapsed_ms);
    Serial.print(F(" Hz  cuentas="));
    Serial.print(counts);
    Serial.print(F("  grados="));
    Serial.println(counts * (360.0f / 4096.0f), 2);
}

// La salud del sensor más nuestros propios contadores de transferencias. La
// lectura de STATUS la hace el lazo de muestreo en lugar de una muestra, así que
// llamar a esto no perturba la temporización del lazo: cuesta 2 muestras de
// 5000.
static void printStatus(void)
{
    uint8_t status;

    Serial.print(F("STATUS="));

    if (!Sensor::read_status(status))
    {
        Serial.print(F("<se agoto la espera; esta corriendo el temporizador de muestreo?>"));
    }
    else
    {
        Serial.print(F("0x"));
        if (status < 0x10)
        {
            Serial.print('0');
        }
        Serial.print(status, HEX);

        Serial.print(F(" ["));
        Serial.print((status & Sensor::STATUS_MD) ? F("MD") : F("--"));
        Serial.print((status & Sensor::STATUS_ML) ? F(" ML") : F(" --"));
        Serial.print((status & Sensor::STATUS_MH) ? F(" MH") : F(" --"));
        Serial.print(']');

        if (!(status & Sensor::STATUS_MD))
        {
            Serial.print(F("  ERROR: no se detecta el iman"));
        }
        if (status & Sensor::STATUS_ML)
        {
            Serial.print(F("  AVISO: iman muy debil / entrehierro muy grande"));
        }
        if (status & Sensor::STATUS_MH)
        {
            Serial.print(F("  AVISO: iman muy fuerte / entrehierro muy chico"));
        }
    }

    Serial.print(F("  desbordes="));
    Serial.print(Sensor::overruns());
    Serial.print(F("  errores="));
    Serial.println(Sensor::errors());
}

// ------------------------------------------------------------------- Arduino

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ;  // inofensivo en el UNO, necesario en placas con USB nativo
    }

    Serial.println();
    Serial.println(F("Lazo de muestreo del AS5600 a 5 kHz"));

    Sensor::begin();
    startSampleTimer();
}

void loop()
{
    printAngle();
    printStatus();
    delay(1000);
}
