// AS5600 sampled at 5 kHz from a timer ISR over an interrupt-driven I2C bus.
//
// Hardware:
// Arduino UNO
// Hall Position Sensor: AS5600 (I2C)
//
// Wiring: SDA -> A4, SCL -> A5, VDD -> 5V, GND -> GND
//
// Output: 115200 baud, two summary lines per second. Printing every sample is
// not possible at this rate -- 5 kHz of text needs roughly 2 Mbaud.

#include <nI2C.h>

#include <AS5600.h>
#include <NI2CBus.h>

typedef AS5600<NI2CBus> Sensor;

// Timer2, CTC, prescaler 32: 16 MHz / 32 / 100 = exactly 5.000 kHz.
// Timer2 leaves millis() (Timer0) and Servo (Timer1) alone, but collides with tone().
static void startSampleTimer(void)
{
    TCCR2A = _BV(WGM21);                // CTC, TOP = OCR2A
    TCCR2B = _BV(CS21) | _BV(CS20);     // prescaler /32
    OCR2A = 99;
    TCNT2 = 0;
    TIMSK2 = _BV(OCIE2A);
}

ISR(TIMER2_COMPA_vect)
{
    Sensor::do_transfer();
}

// ------------------------------------------------------------------ telemetry

static void printAngle(void)
{
    static uint16_t last_samples = 0;
    static uint32_t last_ms = 0;

    uint32_t now_ms = millis();
    uint16_t samples = Sensor::samples();

    // Unsigned wraparound makes the delta correct even though the counter is
    // 16 bits and rolls over every ~13 s at 5 kHz.
    uint16_t delta = samples - last_samples;
    uint32_t elapsed_ms = now_ms - last_ms;

    last_samples = samples;
    last_ms = now_ms;

    if (elapsed_ms == 0)
    {
        return;
    }

    uint16_t counts = Sensor::counts();

    Serial.print(F("rate="));
    Serial.print((uint32_t)delta * 1000UL / elapsed_ms);
    Serial.print(F(" Hz  counts="));
    Serial.print(counts);
    Serial.print(F("  deg="));
    Serial.println(counts * (360.0f / 4096.0f), 2);
}

// Sensor health plus our own transfer counters. The STATUS read is performed by
// the sample loop in place of one sample, so calling this does not disturb the
// loop's timing -- it costs 2 samples out of 5000.
static void printStatus(void)
{
    uint8_t status;

    Serial.print(F("STATUS="));

    if (!Sensor::read_status(status))
    {
        Serial.print(F("<timed out; is the sample timer running?>"));
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
            Serial.print(F("  ERROR: no magnet detected"));
        }
        if (status & Sensor::STATUS_ML)
        {
            Serial.print(F("  WARN: magnet too weak / airgap too large"));
        }
        if (status & Sensor::STATUS_MH)
        {
            Serial.print(F("  WARN: magnet too strong / airgap too small"));
        }
    }

    Serial.print(F("  overruns="));
    Serial.print(Sensor::overruns());
    Serial.print(F("  errors="));
    Serial.println(Sensor::errors());
}

// ------------------------------------------------------------------- Arduino

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ;  // harmless on the UNO, needed on native-USB boards
    }

    Serial.println();
    Serial.println(F("AS5600 5 kHz sample loop"));

    Sensor::begin();
    startSampleTimer();
}

void loop()
{
    printAngle();
    printStatus();
    delay(1000);
}
