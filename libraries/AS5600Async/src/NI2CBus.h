// Vincula AS5600<> con nI2C. El único archivo de esta biblioteca que sabe de
// nI2C.
//
// nI2C está gobernada por interrupciones (ISR(TWI_vect) en nTWI.cpp) y dispara su
// callback de finalización desde esa ISR, que es lo que la hace utilizable bajo
// un lazo de muestreo disparado por temporizador. Su camino de lectura apunta el
// paquete directamente al buffer de quien llama; sólo las escrituras reservan
// memoria, así que el lazo de lectura en régimen no toca el heap.

#ifndef NI2CBUS_H
#define NI2CBUS_H

#include <stdint.h>
#include <nI2C.h>

struct NI2CBus
{
    // address_size tiene que ser distinto de cero o nI2C rechaza el handle de
    // plano (PrepareForTransfer en nI2C.cpp). Las direcciones de registro del
    // AS5600 son de un byte.
    static void begin(uint8_t address)
    {
        m_handle = nI2C->RegisterDevice(address, 1, CI2C::Speed::FAST);  // 400 kHz
    }

    // La sobrecarga de nI2C sin dirección de registro: START + SLA+R
    // directamente, sin recargar el puntero. Ésta es la sobrecarga de la que
    // depende el camino rápido del AS5600.
    static bool read(uint8_t* buffer, uint8_t length, void (*callback)(uint8_t status))
    {
        return nI2C->Read(m_handle, buffer, length, callback) == CI2C::STATUS_OK;
    }

    // Lectura con dirección de registro: nI2C encola juntas la escritura del
    // puntero y la lectura, así que el puntero queda en `reg` (o donde el
    // dispositivo lo haya incrementado).
    static bool read_register(uint8_t reg, uint8_t* buffer, uint8_t length,
                              void (*callback)(uint8_t status))
    {
        return nI2C->Read(m_handle, reg, buffer, length, callback) == CI2C::STATUS_OK;
    }

    static bool ok(uint8_t status) { return status == CI2C::STATUS_OK; }

    static CI2C::Handle m_handle;
};

#endif  // NI2CBUS_H
