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
#include <BoardStart.h>
#include <SampleClock.h>

typedef AS5600<NI2CBus> Sensor;

static const uint16_t SAMPLE_HZ = 5000;

// El mismo reloj que usa ControlDemo, en lugar de otra copia del Timer2 que se le
// vaya separando. Acá no hay ley de control, así que el divisor es 1: cada muestra
// es su propio período y nadie pregunta si venció uno.
//
// El Timer2 deja en paz a millis() (Timer0) y a Servo (Timer1), pero choca con
// tone() y con analogWrite() en los pines 3 y 11.
static SampleClock g_clock(1);

ISR(TIMER2_COMPA_vect)
{
    Sensor::do_transfer();
    g_clock.on_isr();
}

// ------------------------------------------------------------------ telemetría

// La muestra y el instante de la última impresión, para sacar la tasa por
// diferencia. Con nombre y a nivel de archivo, no escondidas adentro de la función:
// es estado, y el estado que no se ve es el que sorprende.
static uint16_t g_last_samples = 0;
static uint32_t g_last_ms      = 0;

static void printAngle(void)
{
    uint32_t now_ms = millis();
    uint16_t samples = Sensor::samples();

    // La vuelta al cero de los enteros sin signo hace que la diferencia sea
    // correcta aunque el contador sea de 16 bits y dé la vuelta cada ~13 s a
    // 5 kHz.
    uint16_t delta = samples - g_last_samples;
    uint32_t elapsed_ms = now_ms - g_last_ms;

    g_last_samples = samples;
    g_last_ms      = now_ms;

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
    // Antes del Serial, y antes del muestreador: los 5 kHz son un TOP del
    // Timer2 y valen lo que valga el reloj. El rescate del bus, por el reset
    // que dejó al sensor a medio hablar. Ver BoardStart.h.
    board::clock_begin();
    board::bus_recover();

    Serial.begin(115200);
    while (!Serial)
    {
        ;  // inofensivo en el UNO, necesario en placas con USB nativo
    }

    Serial.println();
    Serial.println(F("Lazo de muestreo del AS5600 a 5 kHz"));

    Sensor::begin();
    g_clock.begin(SAMPLE_HZ);
}

void loop()
{
    printAngle();
    printStatus();
    delay(1000);
}
