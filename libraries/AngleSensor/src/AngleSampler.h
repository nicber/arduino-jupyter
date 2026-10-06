// El ángulo del sensor tal como lo maneja la ISR del muestreador: una muestra por
// tick, desenrollada en el tick mismo, y congelada en el tick que cierra una fila.
//
// Desenrollar en cada tick y no una vez por fila: desenrollar sólo vale mientras el
// eje gire menos de media vuelta entre dos lecturas (ver AngleTracker.h), y a 5 kHz
// eso son 15 700 rad/s contra 628 con filas de 200 Hz.
//
// Congelar en el tick y no cuando loop() llega a atender la fila: si no, una fila
// atendida tarde llevaría la marca de su tick con una medición de hasta `loop_late`
// después. Congelado, el ángulo tiene un retardo fijo de un tick: es la transferencia
// que lanzó el tick anterior.
//
// `Sensor` es el driver: samples(), counts() y do_transfer(), como en AS5600.

#ifndef ANGLESENSOR_ANGLESAMPLER_H
#define ANGLESENSOR_ANGLESAMPLER_H

#include <stdint.h>

#include "AngleTracker.h"

#if defined(__AVR__)
#include <util/atomic.h>
#define ANGLESAMPLER_ATOMIC ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#else
#define ANGLESAMPLER_ATOMIC     // en la máquina de escritorio no hay interrupciones
#endif

template <class Sensor, uint16_t PerRev = 4096>
class AngleSampler
{
    public:

    typedef AngleTracker<PerRev> Tracker;

    Tracker turns;      // la cuenta cruda, desenrollada en cada tick

    constexpr AngleSampler()
        : turns()
        , m_samples(0)
        , m_fresh_now(0)
        , m_counts(0)
        , m_raw(0)
        , m_raw_uw(0)
        , m_fresh(0)
    {
    }

    // En cada tick, desde la ISR. Con `transfer` en false no lanza la lectura del
    // tick, que es lo que hace la perilla `dbg_i2c` de Banco.
    void on_tick(bool transfer = true)
    {
        // Antes de lanzar la transferencia de este tick: si el contador no avanzó
        // desde el tick anterior, la que se lanzó entonces no terminó, y la cuenta
        // que hay es la de antes.
        const uint16_t samples = Sensor::samples();
        m_fresh_now = (samples != m_samples);
        m_samples   = samples;

        if (transfer)
        {
            Sensor::do_transfer();
        }

        // Una muestra repetida no avanza nada.
        m_counts = Sensor::counts();
        turns.update((typename Tracker::Counts)m_counts);
    }

    // En el tick que cierra una fila, desde la ISR y después de on_tick().
    void freeze(void)
    {
        m_raw    = m_counts;
        m_raw_uw = turns.y_uw;
        m_fresh  = m_fresh_now;
    }

    // Desde loop(): lo que congeló el último freeze(), de una sola vez. `raw` es la
    // cuenta cruda de adentro de la vuelta, `raw_uw` la misma desenrollada, y
    // `fresh` dice si es una muestra nueva o repite la de la fila anterior porque la
    // transferencia no terminó a tiempo.
    void take(uint16_t& raw, int32_t& raw_uw, bool& fresh) const
    {
        ANGLESAMPLER_ATOMIC
        {
            raw    = m_raw;
            raw_uw = m_raw_uw;
            fresh  = m_fresh;
        }
    }

    private:

    // Sólo de la ISR.
    uint16_t m_samples;     // el contador de muestras del sensor en el tick anterior
    uint8_t  m_fresh_now;
    uint16_t m_counts;

    // Lo congelado, que lee loop().
    volatile uint16_t m_raw;
    volatile int32_t  m_raw_uw;
    volatile uint8_t  m_fresh;
};

#undef ANGLESAMPLER_ATOMIC

#endif  // ANGLESENSOR_ANGLESAMPLER_H
