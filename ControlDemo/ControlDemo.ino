// Position control loop over CtrlLink, driven from a Jupyter notebook.
//
// Hardware:
// Arduino UNO
// Hall Position Sensor: AS5600 (I2C)   SDA -> A4, SCL -> A5
// Actuator (optional):  PWM on pin 9, direction on pin 8
//
// Timer2 samples the AS5600 at 5 kHz; every `tickdiv`-th sample runs the
// control law, so the loop rate is 5000/tickdiv Hz and defaults to 1 kHz.
// Sampling therefore keeps a rigid period even when the control computation
// jitters, and `maxlate` reports how much jitter there actually was.
//
// Serial is 1 Mbaud. On a 16 MHz AVR that is an exact divisor (UBRR=1), unlike
// 115200, which lands 2.1% off. Telemetry sustains that rate comfortably, but
// incoming bytes arrive every 10 us and the USART holds only two, so the 5 kHz
// sampler and nI2C's TWI interrupt together drop a few percent of the bytes of a
// command sent back to back. The host paces command bytes to compensate; see
// PROTOCOL.md. Commands are rare and tiny, so this costs nothing.
//
// Timer2 belongs to the sampler, so analogWrite() on pins 3 and 11 no longer
// works. Pins 9 and 10 (Timer1) and 5 and 6 (Timer0) are unaffected.

#include <nI2C.h>

#include <AS5600.h>
#include <NI2CBus.h>
#include <CtrlLink.h>

typedef AS5600<NI2CBus> Sensor;

static const uint32_t BAUD          = 1000000;
static const uint16_t SAMPLE_HZ     = 5000;
static const uint16_t COUNTS_PER_REV = 4096;

// Leave undefined to run the loop with no actuator attached: everything else,
// including the telemetry, behaves identically.
#define MOTOR_PWM_PIN 5
//#define MOTOR_DIR_PIN 8

// ------------------------------------------------------------------ variables
// Everything the host can read or write lives here. Channels are read by
// CtrlLink::emit() through their addresses, so they must be written by the same
// context that calls emit() -- loop(), not the ISR.

static float   g_kp   = 0.5f;
static float   g_ki   = 0.0f;
static float   g_kd   = 0.0f;
static int16_t g_ref  = 0;      // setpoint, counts (0..4095)
static int16_t g_uff  = 0;      // feedforward / open-loop command, -255..255
static uint8_t g_mode = 0;      // 0 = open loop (u = uff), 1 = position PID
static uint8_t g_tickdiv = 5;   // control periods per 5 kHz sample: 5 -> 1 kHz
static uint16_t g_maxlate = 0;  // worst observed ISR-to-service delay, us

static int16_t g_y = 0;         // measured angle, counts
static int16_t g_e = 0;         // error, counts, shortest way round
static int16_t g_u = 0;         // actuator command, -255..255

static float g_integral = 0.0f;
static float g_prev_e   = 0.0f;
static float g_dt       = 0.001f;

// Set by the ISR, cleared by loop(). `g_tick_us` is when the tick fired, so the
// service delay is visible to the loop that picks it up.
static volatile bool     g_tick      = false;
static volatile uint32_t g_tick_us   = 0;
static volatile uint16_t g_missed    = 0;   // ticks loop() did not service in time
static volatile uint8_t  g_divider   = 5;

// -------------------------------------------------------------------- tables

static const CtrlParam PROGMEM g_params[] =
{
    { "kp",      CTRL_F32, &g_kp      },
    { "ki",      CTRL_F32, &g_ki      },
    { "kd",      CTRL_F32, &g_kd      },
    { "ref",     CTRL_I16, &g_ref     },
    { "uff",     CTRL_I16, &g_uff     },
    { "mode",    CTRL_U8,  &g_mode    },
    { "tickdiv", CTRL_U8,  &g_tickdiv },
    { "maxlate", CTRL_U16, &g_maxlate },
};

static const float COUNTS_TO_DEG = 360.0f / COUNTS_PER_REV;

static const CtrlChannel PROGMEM g_channels[] =
{
    { "ref", CTRL_I16, &g_ref, COUNTS_TO_DEG, "deg" },
    { "y",   CTRL_I16, &g_y,   COUNTS_TO_DEG, "deg" },
    { "e",   CTRL_I16, &g_e,   COUNTS_TO_DEG, "deg" },
    { "u",   CTRL_I16, &g_u,   1.0f,          "pwm" },
};

// ---------------------------------------------------------------------- timer

// Timer2, CTC, prescaler 32: 16 MHz / 32 / 100 = exactly 5.000 kHz.
// Timer2 leaves millis() (Timer0) and Servo (Timer1) alone, but collides with
// tone() and with analogWrite() on pins 3 and 11.
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
    static uint8_t count = 0;

    Sensor::do_transfer();

    if (++count < g_divider)
    {
        return;
    }
    count = 0;

    if (g_tick)
    {
        // loop() has not serviced the previous tick: the control period is
        // being missed outright, which is worse than mere jitter.
        g_missed++;
    }

    g_tick_us = micros();
    g_tick    = true;
}

// -------------------------------------------------------------------- control

// Shortest way from y to ref on a circle of 4096 counts, so a setpoint just
// past the wrap point does not command a full turn the wrong way.
static int16_t wrapped_error(int16_t ref, int16_t y)
{
    return (int16_t)(((ref - y + 2048) & 0x0FFF) - 2048);
}

static int16_t saturate(float v, int16_t limit)
{
    if (v >  (float)limit) return  limit;
    if (v < -(float)limit) return -limit;
    return (int16_t)v;
}

static void drive(int16_t u)
{
#ifdef MOTOR_PWM_PIN
    //digitalWrite(MOTOR_DIR_PIN, (u >= 0) ? HIGH : LOW);
    analogWrite(MOTOR_PWM_PIN, (uint8_t)(u >= 0 ? u : -u));
#else
    (void)u;
#endif
}

static void control_step(void)
{
    static int16_t g_reference = 0.0f;
    
    if (g_mode == 0 || g_mode == 1)
    {
        g_reference = g_ref;
    }
    else if (g_mode == 2)
    {
        g_reference += g_ref;
        g_reference %= COUNTS_PER_REV;
    }

    g_y = (int16_t)Sensor::counts();
    g_e = wrapped_error(g_reference, g_y);

    if (g_mode == 0)
    {
        // Open loop: the host drives `uff` directly. This is the mode to use
        // for plant identification -- step uff and watch y.
        g_integral = 0.0f;
        g_prev_e   = (float)g_e;
        g_u        = saturate((float)g_uff, 255);
    }
    else
    {
        float e = (float)g_e;

        float derivative = (e - g_prev_e) / g_dt;
        g_prev_e = e;

        float candidate = g_kp * e + g_ki * (g_integral + e * g_dt) + g_kd * derivative
                        + (float)g_uff;

        g_u = saturate(candidate, 255);
        if (g_u < 0) {
            g_u = 0;
        }

        // Conditional integration: stop winding up the integrator once the
        // actuator is saturated in the direction the integrator is pushing.
        bool saturated = (candidate > 255.0f && e > 0.0f)
                      || (candidate < -255.0f && e < 0.0f);

        if (!saturated)
        {
            g_integral += e * g_dt;
        }
    }

    drive(g_u);
}

// ------------------------------------------------------------------- Arduino

void setup()
{
#ifdef MOTOR_PWM_PIN
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    //pinMode(MOTOR_DIR_PIN, OUTPUT);
    analogWrite(MOTOR_PWM_PIN, 0);
#endif

    CtrlLink::set_id(F("ControlDemo"));
    CtrlLink::begin(BAUD,
                    g_params,   sizeof(g_params)   / sizeof(g_params[0]),
                    g_channels, sizeof(g_channels) / sizeof(g_channels[0]),
                    1000000UL / (SAMPLE_HZ / g_tickdiv));

    Sensor::begin();
    startSampleTimer();

    CtrlLink::note(F("ControlDemo ready"));
}

void loop()
{
    // Track a change to the loop rate before servicing the next tick, so the
    // derivative and integral terms and the period reported to the host all
    // agree with the divider the ISR is actually using.
    if (g_tickdiv != g_divider)
    {
        if (g_tickdiv == 0)
        {
            g_tickdiv = 1;
        }
        g_divider = g_tickdiv;
        g_dt      = (float)g_tickdiv / (float)SAMPLE_HZ;
        CtrlLink::set_period_us((uint32_t)g_tickdiv * 1000000UL / SAMPLE_HZ);
    }

    if (g_tick)
    {
        uint32_t fired;

        // The ISR can land between the two halves of a 32-bit load.
        noInterrupts();
        fired  = g_tick_us;
        g_tick = false;
        interrupts();

        uint16_t late = (uint16_t)(micros() - fired);
        if (late > g_maxlate)
        {
            g_maxlate = late;
        }

        control_step();
        CtrlLink::emit();
    }

    CtrlLink::poll();
}
