// Position control loop over CtrlLink, driven from a Jupyter notebook.
//
// Hardware:
// Arduino UNO
// Hall Position Sensor: AS5600 (I2C)   SDA -> A4, SCL -> A5
// Current sense (optional): ACS712 on A0
// Actuator (optional):  PWM on pin 5, direction on pin 8
//
// Timer2 samples the AS5600 at 5 kHz; every `tickdiv`-th sample runs the
// control law, so the loop rate is 5000/tickdiv Hz and defaults to 1 kHz.
// Sampling therefore keeps a rigid period even when the control computation
// jitters. `maxlate` reports how much jitter there was and `missed` counts the
// control periods that were skipped outright.
//
// The control law is integer arithmetic end to end -- see ControlMath. So is
// every parameter it reads: each one is stored in the fixed-point form the
// arithmetic wants, and the parameter table declares the scale that converts
// it. The host multiplies on the way in and divides on the way out, so a
// student still writes `dev.kp = 0.5` and this sketch never executes a single
// floating-point instruction.
//
// Serial is 1 Mbaud. On a 16 MHz AVR that is an exact divisor (UBRR=1), unlike
// 115200, which lands 2.1% off. Telemetry sustains that rate comfortably, but
// incoming bytes arrive every 10 us and the USART holds only two, so the 5 kHz
// sampler and nI2C's TWI interrupt together drop a few percent of the bytes of a
// command sent back to back. The host paces command bytes to compensate; see
// PROTOCOL.md. Commands are rare and tiny, so this costs nothing.
//
// The default channel table is 41 bytes per row, or 41% of the link at 1 kHz.
// That is more than the "well under half" this protocol likes; raise `dec` for
// long runs, or drop a channel.
//
// Peripherals this sketch takes over: Timer2, so analogWrite() on pins 3 and 11
// and tone() no longer work; and the ADC, which is driven directly here, so
// analogRead() must not be called. Pins 9 and 10 (Timer1) and 5 and 6 (Timer0)
// are unaffected.

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

// Leave MOTOR_PWM_PIN undefined to run the loop with no actuator attached:
// everything else, including the telemetry, behaves identically.
//
// Defining MOTOR_DIR_PIN as well gives bidirectional drive, -255..255. With it
// undefined the bridge is single-quadrant and the command is clamped at zero,
// which is what the anti-windup logic is told through U_MIN.
#define MOTOR_PWM_PIN 5
//#define MOTOR_DIR_PIN 8

#ifdef MOTOR_DIR_PIN
static const int16_t U_MIN = -255;
#else
static const int16_t U_MIN = 0;
#endif
static const int16_t U_MAX = 255;

// Current sense on A0. ACS712-05B: 185 mV/A about a 2.5 V zero, so half scale
// on a 5 V reference. Change SENSE_MV_PER_A for a different part -- it only
// affects the units the host is told about, never the loop.
static const uint8_t  SENSE_CHANNEL   = 0;
static const int16_t  SENSE_ZERO      = 512;
static const float    SENSE_MV_PER_A  = 185.0f;
static const float    ADC_MV_PER_LSB  = 5000.0f / 1024.0f;
static const float    SENSE_MA_PER_LSB = 1000.0f * ADC_MV_PER_LSB / SENSE_MV_PER_A;

// `ref` and `refrate` carry 8 fractional bits, so a ramp can advance by less
// than one count per period without quantising to nothing.
static const uint8_t  REF_FRAC = 8;

// Two independent choices, and it is worth keeping them apart.
//
// `mode` picks the controller: the law that turns an error into a command.
// `target` picks the feedback: which measured quantity that law closes on.
// A controller is a function and a case in control_step(); a feedback is a
// branch in target_error(). Neither knows about the other.
enum : uint8_t
{
    MODE_OPEN = 0,      // u = uff, controller bypassed
    MODE_PID  = 1,      // PID on the selected target
    MODE_RAMP = 2,      // PID, with ref advancing by refrate every period
};

// `ref >> REF_FRAC` is always in the raw units of the selected target: counts
// for TARGET_POSITION, ADC LSBs for TARGET_CURRENT.
enum : uint8_t
{
    TARGET_POSITION = 0,
    TARGET_CURRENT  = 1,
};

// Fixed-point scales, chosen for the range each quantity actually needs.
// See FixedPoint.h; the trade is magnitude against resolution.
//
// The gains are per sample, not per second: u = kp*e + ki*sum(e) + kd*diff(e),
// with no dt anywhere. That is what the arithmetic does, so it is what the
// parameter means, and it keeps every scale a compile-time constant the host
// can be told about. A host that prefers continuous-time gains multiplies by
// dt on its own side -- and when `tickdiv` changes, the effect of the same
// three numbers changing with it is the lesson, not a bug.
//
// The parameter table publishes FRAC straight off these types, so the host is
// told each format by the declaration that defines it rather than by a constant
// that has to be kept in step with it.
typedef Fixed<int32_t, 22> Kp;      // +/-511,   resolution 2.4e-7
typedef Fixed<int32_t, 30> Ki;      // +/-1.99,  resolution 9.3e-10
typedef Fixed<int32_t, 16> Kd;      // +/-32767, resolution 1.5e-5
typedef FirstOrderFilter<4>::Alpha Alpha;   // Q16, a fraction in [0, 1]

// ------------------------------------------------------------------ variables
// Everything the host can read or write lives here. Channels are read by
// CtrlLink::emit() through their addresses, so they must be written by the same
// context that calls emit() -- loop(), not the ISR.

// Gains, in the fixed-point form the controllers use. The host sets them in
// natural units and the parameter table's declared format does the conversion.
// The default is a mild proportional loop: the error is in counts, so a gain
// that looks small is not, and 4096 of them make a revolution.
static int32_t g_kp = Kp::from_float(0.002f).raw();
static int32_t g_ki = 0;
static int32_t g_kd = 0;

// Filter poles: alpha = dt / (tau + dt), a fraction in [0, 1]. alpha = 1 is a
// pass-through, which is how a filter is switched off -- so alpha_y = 1 feeds
// the position loop the raw count. A host that thinks in time constants
// converts, because it is the side that knows dt and has the arithmetic for it.
static int32_t g_alpha_y = Alpha::from_int(1).raw();            // position, two poles
static int32_t g_alpha_i = Alpha::from_float(0.1667f).raw();    // current, two poles
static int32_t g_alpha_e = Alpha::from_float(0.0909f).raw();    // error, one pole

static int32_t g_ref     = 0;   // setpoint, target units << REF_FRAC
static int32_t g_refrate = 0;   // ramp rate, same units per control period
static int16_t g_uff     = 0;   // feedforward / open-loop command
static int16_t g_offset  = 0;   // sensor zero, counts
static uint8_t g_target  = TARGET_POSITION;
static uint8_t g_mode    = MODE_OPEN;
static uint8_t g_tickdiv = 5;   // 5 kHz samples per control period: 5 -> 1 kHz

static int16_t g_y     = 0;     // measured angle, counts, offset applied
static int32_t g_y_uw  = 0;     // unwrapped angle, counts, unfiltered
static int32_t g_y_uwf = 0;     // unwrapped angle, counts, filtered by alpha_y
static int16_t g_i     = 0;     // current, ADC LSBs about SENSE_ZERO, filtered
static int16_t g_e     = 0;     // error, target units, clamped for telemetry
static int16_t g_u     = 0;     // actuator command, U_MIN..U_MAX

// Health counters. All are host-writable, so zeroing one restarts the count.
static uint16_t g_maxlate = 0;  // worst observed ISR-to-service delay, us
static uint16_t g_missed  = 0;  // control periods loop() never serviced
static uint16_t g_sovr    = 0;  // sensor samples the I2C bus could not keep up with
static uint16_t g_serr    = 0;  // sensor transfers that failed
static uint8_t  g_mstat   = 0;  // AS5600 STATUS register: magnet present, too weak, too strong

// Integrator state, in error units summed over ticks. Keeping the sum raw and
// applying ki*dt once at the end is what lets a gain of 5e-5 survive: the
// quantisation lands on the gain, where it is a fraction of a percent, instead
// of on the accumulation, where it would truncate to nothing every period.
static int32_t g_integral = 0;
static int32_t g_e_filt   = 0;
static int32_t g_e_prev   = 0;

// The one derived quantity left, recomputed by refresh_tuning().
static int32_t g_integral_max = INT32_MAX / 2;

static FirstOrderFilter<4> g_y_filt[2];   // position: free-running, needs the headroom
static FirstOrderFilter<8> g_i_filt[2];   // current: small signal, wants the resolution
static FirstOrderFilter<8> g_err_filt;

// Set by the ISR, cleared by loop(). `g_tick_us` is when the tick fired, so the
// service delay is visible to the loop that picks it up.
static volatile bool     g_tick       = false;
static volatile uint32_t g_tick_us    = 0;
static volatile uint16_t g_missed_isr = 0;
static volatile int16_t  g_adc        = 0;   // last completed A0 conversion
static volatile uint8_t  g_divider    = 5;

// -------------------------------------------------------------------- tables

// Every entry is stored exactly as the arithmetic wants it; the scale column is
// what lets the host go on speaking in natural units.
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

// `ref` and `e` are in the raw units of whatever `target` selects, so they get
// no engineering scale here -- claiming degrees would be a lie the moment the
// loop is switched to current. They come out in target units and the host
// multiplies by the scale of `y_uw` or of `i` to suit. Everything else has a
// fixed meaning and carries its own.
static const CtrlChannel PROGMEM g_channels[] =
{
    { "ref",   CTRL_I32, &g_ref,   1.0f / (1 << REF_FRAC), "tgt" },
    { "y_uw",  CTRL_I32, &g_y_uw,  COUNTS_TO_DEG,      "deg" },
    { "y_uwf", CTRL_I32, &g_y_uwf, COUNTS_TO_DEG,      "deg" },
    { "e",     CTRL_I16, &g_e,     1.0f,               "tgt" },
    { "u",     CTRL_I16, &g_u,     1.0f,               "pwm" },
    { "i",     CTRL_I16, &g_i,     SENSE_MA_PER_LSB,   "mA"  },
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

// ------------------------------------------------------------------------ adc

// The ADC is driven straight from the sampler instead of through analogRead(),
// which busy-waits for the conversion. A conversion at /128 takes 104 us, so
// one fits inside a 200 us sample period: the ISR collects the result the
// previous tick started and immediately starts the next. The cost is one
// sample period of delay on `i`; the saving is 112 us of blocking out of a
// 1000 us control period.
static void startAdc(void)
{
    ADMUX  = _BV(REFS0) | (SENSE_CHANNEL & 0x07);   // AVcc reference
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADSC);
}

ISR(TIMER2_COMPA_vect)
{
    static uint8_t count = 0;

    Sensor::do_transfer();

    if (ADCSRA & _BV(ADIF))
    {
        g_adc = (int16_t)ADC;
        // Writing 1 to ADIF clears it; the same store starts the next
        // conversion, so the ADC free-runs one result behind the sampler.
        ADCSRA |= _BV(ADIF) | _BV(ADSC);
    }

    if (++count < g_divider)
    {
        return;
    }
    count = 0;

    if (g_tick)
    {
        // loop() has not serviced the previous tick: the control period is
        // being missed outright, which is worse than mere jitter.
        g_missed_isr++;
    }

    g_tick_us = micros();
    g_tick    = true;
}

// -------------------------------------------------------------------- control

// Shortest way from a to b on a circle of 4096 counts, so a setpoint just past
// the wrap point does not command a full turn the wrong way.
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

// The magnet turns the opposite way to the shaft, hence the negation; `offset`
// is then the count that reads as zero.
static int16_t sensor_measurement(void)
{
    return wrapped_error(g_offset, (int16_t)Sensor::counts());
}

// Reads the sensors and updates every measured variable. Runs once per control
// period whatever the mode, so the telemetry stays live in open loop.
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

// The feedback dispatch: which measured quantity the loop closes on. Publishes
// the clamped error for telemetry on the way past, so a controller that ignores
// the value still leaves `e` live for the host to watch.
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
        // Position. alpha_y = 1 makes the filter a pass-through, so this is
        // the raw unwrapped count unless the host asked for smoothing.
        e = ref - g_y_uwf;
    }

    g_e = clamp16(e, INT16_MIN, INT16_MAX);
    return e;
}

// ---------------------------------------------------------------- controllers
// One function per mode, all with the same signature: read the measurements and
// the parameters, leave a command in `g_u`. Adding a controller means adding a
// function here and a case to the switch in control_step().

// Open loop: the host drives `uff` straight onto the actuator. This is the mode
// to use for plant identification -- step uff and watch what comes back.
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

    // Conditional integration: stop winding up the integrator once the
    // actuator is saturated in the direction the integrator is pushing.
    bool saturated = (candidate > U_MAX && e > 0)
                  || (candidate < U_MIN && e < 0);

    if (!saturated)
    {
        // Second line of defence, and the one that matters when ki is changed
        // mid-run: cap the sum at the point where its term alone would saturate
        // the actuator, so the integrator can always unwind within a period or
        // two. The sum is formed wide because the cap is only applied
        // afterwards, and a single large error would otherwise be able to
        // overflow the accumulator on the way there.
        int64_t sum = (int64_t)g_integral + e;

        if (sum >  g_integral_max) sum =  g_integral_max;
        if (sum < -g_integral_max) sum = -g_integral_max;

        g_integral = (int32_t)sum;
    }
}

// The ramp is the PID with a moving setpoint, so it is the PID plus one line
// rather than a controller of its own.
static void controller_ramp(void)
{
    g_ref += g_refrate;
    controller_pid();
}

// Called when the host switches controllers. Without it a controller inherits
// the integrator and the derivative history of the one before it and kicks on
// its first period.
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

        // An unknown mode is the safe one: a typo on the host must not leave
        // the actuator being driven by a controller nobody chose.
        case MODE_OPEN:
        default:        controller_open(); break;
    }

    drive(g_u);
}

// The sensor's own counters are free-running and cannot be cleared, so what is
// published is the running total of their increments. That is what makes `sovr`
// and `serr` host-writable like the rest: zeroing one restarts the count from
// here rather than being overwritten on the next period.
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

// ------------------------------------------------------------------- tuning

// Applies whatever the host has just written. The parameters arrive already in
// the form the arithmetic wants -- that conversion is the host's job -- so all
// this does is propagate the two that other state depends on. No floats, which
// is why it is safe to run it straight after a control step.
//
// It is driven by CtrlLink's write counter: one 16-bit comparison per pass of
// loop(), rather than watching each parameter for a change.
static void refresh_tuning(void)
{
    if (g_tickdiv == 0)
    {
        g_tickdiv = 1;
    }

    // The divider the ISR uses and the period reported to the host change
    // together, so neither can be left describing a rate the loop is not
    // running at. The gains are per sample and do not depend on either.
    g_divider = g_tickdiv;
    CtrlLink::set_period_us((uint32_t)g_tickdiv * 1000000UL / SAMPLE_HZ);

    // Bound the integrator at the sum whose term alone saturates the actuator,
    // so it can always unwind within a period or two. A ki small enough to put
    // that past what an int32_t holds leaves the type's own limit standing --
    // the accumulation has to stay in range whether or not ki cares.
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

// The AS5600's own view of the magnet: detected, too weak, too strong. Reading
// it costs the sample loop one sample and blocks here until that sample lands,
// so it is only done between captures -- during a bringup check, in other
// words, which is the only time anyone wants it.
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

    CtrlLink::note(F("ControlDemo ready"));
}

void loop()
{
    static uint16_t last_writes = 0;

    if (g_tick)
    {
        uint32_t fired;
        uint16_t missed;

        // The ISR can land between the two halves of a 32-bit load.
        noInterrupts();
        fired        = g_tick_us;
        missed       = g_missed_isr;
        g_missed_isr = 0;
        g_tick       = false;
        interrupts();

        // Accumulated into a plain copy rather than read straight out of the
        // ISR's counter, so that the host can zero it without racing the ISR.
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

    // After the control step, never before one: a `set` that lands just as a
    // tick fires would otherwise put this in front of it.
    uint16_t writes = CtrlLink::writes();
    if (writes != last_writes)
    {
        last_writes = writes;
        refresh_tuning();
    }

    refresh_magnet_status();

    CtrlLink::poll();
}
