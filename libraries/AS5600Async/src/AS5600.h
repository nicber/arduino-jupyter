// AS5600 magnetic angle sensor, read asynchronously for a fast poll loop.
//
// The AS5600 suppresses auto-increment of its address pointer on reads of the
// ANGLE, RAW ANGLE and MAGNITUDE registers (datasheet [v1-06] 2018-Jun-20,
// page 13), so while the pointer is parked on RAW ANGLE every sample is a bare
// two-byte read with no register write:
//   START + SLA+R + high + low + STOP  ~= 90 us at 400 kHz.
// That fits a 200 us (5 kHz) budget with room to spare; reloading the pointer
// on every sample would cost roughly half again as much.
//
// The pointer is armed lazily: whenever it is known to be somewhere else (at
// startup, after a STATUS read, after a bus error) the next sample uses the
// register-addressed form instead, which costs ~140 us once and re-parks it.
//
// Bus is a static-dispatch policy (no virtuals). It must provide:
//   static void begin(uint8_t address);
//   static bool read(uint8_t* buffer, uint8_t length, void (*callback)(uint8_t));
//   static bool read_register(uint8_t reg, uint8_t* buffer, uint8_t length,
//                             void (*callback)(uint8_t));
//   static bool ok(uint8_t status);

#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <stdint.h>
#include <util/atomic.h>

template <class Bus>
class AS5600
{
    public:

    static const uint8_t DEVICE_ADDRESS = 0x36;  // 7-bit, 0110110b
    static const uint8_t REG_STATUS     = 0x0B;
    static const uint8_t REG_RAWANGLE_H = 0x0C;

    // STATUS register bits (datasheet Figure 23).
    static const uint8_t STATUS_MH = _BV(3);  // AGC minimum gain overflow, magnet too strong
    static const uint8_t STATUS_ML = _BV(4);  // AGC maximum gain overflow, magnet too weak
    static const uint8_t STATUS_MD = _BV(5);  // magnet was detected

    // Call once, before the sample loop starts. The address pointer is armed by
    // the first do_transfer(), so this does not touch the bus and cannot fail.
    static void begin(void)
    {
        Bus::begin(DEVICE_ADDRESS);
        m_armed = false;
    }

    // Start one sample. Non-blocking; meant to be called from a timer ISR.
    static void do_transfer(void)
    {
        if (m_inflight)
        {
            // Previous transfer has not completed: the bus is not keeping up.
            m_overruns++;
            return;
        }

        m_inflight = true;

        bool started;

        if (m_status_request)
        {
            // Housekeeping, in place of one sample. Reading STATUS moves the
            // address pointer, so the next sample has to re-arm it.
            m_status_request = false;
            m_armed = false;
            started = Bus::read_register(REG_STATUS, m_rx, 1, &process_status_data);
        }
        else if (m_armed)
        {
            started = Bus::read(m_rx, sizeof(m_rx), &process_read_data);
        }
        else
        {
            started = Bus::read_register(REG_RAWANGLE_H, m_rx, sizeof(m_rx), &process_read_data);
        }

        if (!started)
        {
            m_inflight = false;
            m_errors++;
        }
    }

    // Latest completed sample, 0..4095.
    // A 16-bit load is two instructions on AVR and the TWI ISR can land between
    // them, so the read is made atomic rather than merely volatile.
    static uint16_t counts(void) { return snapshot(m_counts); }

    static uint16_t samples(void)  { return snapshot(m_samples); }
    static uint16_t overruns(void) { return snapshot(m_overruns); }
    static uint16_t errors(void)   { return snapshot(m_errors); }

    // Fetch the STATUS register. Hands the work to the sample loop, which does
    // it on its next tick in place of one sample, then waits for the result.
    // The caller blocks for up to one sample period; the loop itself is never
    // held up and never reads a stale pointer. Costs 2 samples out of 5000.
    // Returns false if the sample loop is not running.
    static bool read_status(uint8_t& out, uint16_t timeout_ms = 5)
    {
        m_status_ready = false;
        m_status_request = true;

        uint32_t deadline = millis() + timeout_ms;

        while (!m_status_ready)
        {
            if ((int32_t)(millis() - deadline) >= 0)
            {
                return false;
            }
        }

        out = m_status;
        return true;
    }

    private:

    // Runs in the TWI ISR.
    static void process_read_data(uint8_t status)
    {
        if (Bus::ok(status))
        {
            // 12-bit value, high byte first, upper nibble unused.
            m_counts = (((uint16_t)m_rx[0] << 8) | m_rx[1]) & 0x0FFF;
            m_samples++;
            m_armed = true;
        }
        else
        {
            // A failed transfer may have left the pointer anywhere.
            m_errors++;
            m_armed = false;
        }

        m_inflight = false;
    }

    // Runs in the TWI ISR.
    static void process_status_data(uint8_t status)
    {
        if (Bus::ok(status))
        {
            m_status = m_rx[0];
            m_status_ready = true;
        }
        else
        {
            m_errors++;
        }

        // The pointer auto-incremented onto RAW ANGLE, but suppression is only
        // documented when the pointer was *written* to the high byte, so treat
        // it as unarmed and let the next sample set it explicitly.
        m_armed = false;
        m_inflight = false;
    }

    static uint16_t snapshot(const volatile uint16_t& counter)
    {
        uint16_t value;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = counter; }
        return value;
    }

    // Touched only in ISR context, so plain.
    static uint8_t m_rx[2];

    static volatile bool     m_inflight;
    static volatile bool     m_armed;
    static volatile bool     m_status_request;
    static volatile bool     m_status_ready;
    static volatile uint8_t  m_status;
    static volatile uint16_t m_counts;
    static volatile uint16_t m_samples;
    static volatile uint16_t m_overruns;
    static volatile uint16_t m_errors;
};

template <class Bus> uint8_t           AS5600<Bus>::m_rx[2];
template <class Bus> volatile bool     AS5600<Bus>::m_inflight       = false;
template <class Bus> volatile bool     AS5600<Bus>::m_armed          = false;
template <class Bus> volatile bool     AS5600<Bus>::m_status_request = false;
template <class Bus> volatile bool     AS5600<Bus>::m_status_ready   = false;
template <class Bus> volatile uint8_t  AS5600<Bus>::m_status         = 0;
template <class Bus> volatile uint16_t AS5600<Bus>::m_counts         = 0;
template <class Bus> volatile uint16_t AS5600<Bus>::m_samples        = 0;
template <class Bus> volatile uint16_t AS5600<Bus>::m_overruns       = 0;
template <class Bus> volatile uint16_t AS5600<Bus>::m_errors         = 0;

#endif  // AS5600_H
