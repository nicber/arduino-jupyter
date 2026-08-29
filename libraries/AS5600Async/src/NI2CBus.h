// Binds AS5600<> to nI2C. The only file in this library that knows about nI2C.
//
// nI2C is interrupt-driven (ISR(TWI_vect) in nTWI.cpp) and fires its completion
// callback from that ISR, which is what makes it usable under a timer-driven
// sample loop. Its read path points the packet straight at the caller's buffer;
// only writes allocate, so the steady-state read loop touches no heap.

#ifndef NI2CBUS_H
#define NI2CBUS_H

#include <stdint.h>
#include <nI2C.h>

struct NI2CBus
{
    // address_size must be non-zero or nI2C rejects the handle outright
    // (PrepareForTransfer in nI2C.cpp). AS5600 register addresses are one byte.
    static void begin(uint8_t address)
    {
        m_handle = nI2C->RegisterDevice(address, 1, CI2C::Speed::FAST);  // 400 kHz
    }

    // nI2C's no-register-address overload: START + SLA+R directly, no pointer
    // reload. This is the overload the AS5600 fast path depends on.
    static bool read(uint8_t* buffer, uint8_t length, void (*callback)(uint8_t status))
    {
        return nI2C->Read(m_handle, buffer, length, callback) == CI2C::STATUS_OK;
    }

    // Register-addressed read: nI2C queues a pointer write and the read together,
    // so the pointer is left on `reg` (or wherever the device increments it to).
    static bool read_register(uint8_t reg, uint8_t* buffer, uint8_t length,
                              void (*callback)(uint8_t status))
    {
        return nI2C->Read(m_handle, reg, buffer, length, callback) == CI2C::STATUS_OK;
    }

    static bool ok(uint8_t status) { return status == CI2C::STATUS_OK; }

    static CI2C::Handle m_handle;
};

#endif  // NI2CBUS_H
