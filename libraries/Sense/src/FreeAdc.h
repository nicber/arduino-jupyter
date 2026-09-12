// El ADC corriendo libre sobre un solo canal, para que un lazo de control pueda
// leerlo sin bloquearse.
//
// Se maneja directamente en lugar de a través de analogRead(), que espera
// activamente a que la conversión termine. Con preescalador /128 una conversión
// tarda 104 us, así que entra en un período de muestreo de 200 us: la ISR recoge el
// resultado que arrancó el tick anterior e inmediatamente lanza el siguiente. El
// costo es un período de muestreo de retardo en la lectura; el ahorro son 112 us de
// bloqueo adentro de cada período de control.
//
// Publica la cuenta ya normalizada al ancho que se le pida, para que dos placas con
// conversores de distinto ancho den el mismo número. Se corre la lectura hacia
// arriba en lugar de tirar los bits de abajo de la más ancha: eso conserva lo que la
// placa de 12 bits mide de verdad y le cuesta a la de 10 dos ceros al final de un
// número que igual no los tenía.

#ifndef SENSE_FREEADC_H
#define SENSE_FREEADC_H

#include <Arduino.h>
#include <stdint.h>

template <uint8_t Channel>
class FreeAdc
{
    public:

    typedef int16_t Counts;

    constexpr FreeAdc()
        : m_latest(0)
        , m_shift(0)
    {
    }

    // `full_scale` es lo que el conversor de esta placa da a fondo de escala, y
    // `normalized` el ancho al que se quiere publicar.
    //
    // Siempre contra Vcc. Un sensor de corriente bipolar y ratiométrico --un ACS712--
    // reposa en la mitad de su alimentación, y medirlo contra la misma tensión que lo
    // alimenta lo deja en media escala por construcción. En una de las dos placas
    // del banco los bits REFS no eligen nada: ver board::adc_select_vcc().
    void begin(uint16_t full_scale, uint16_t normalized)
    {
        m_shift = 0;
        for (uint16_t width = full_scale; width < normalized; width <<= 1)
        {
            m_shift++;
        }

        ADMUX  = (uint8_t)(_BV(REFS0) | (Channel & 0x07));
        ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADSC);
    }

    // Llamar desde la ISR de muestreo. Recoge la conversión que ya terminó y lanza
    // la siguiente en la misma escritura.
    void on_isr(void)
    {
        if (ADCSRA & _BV(ADIF))
        {
            m_latest = (Counts)(ADC << m_shift);

            // Escribir un 1 en ADIF lo borra; ese mismo almacenamiento lanza la
            // conversión siguiente, así que el conversor corre libre un resultado
            // por detrás del muestreador.
            ADCSRA |= _BV(ADIF) | _BV(ADSC);
        }
    }

    // La última conversión, leída sin que la ISR pueda caer en el medio de los dos
    // bytes.
    Counts read(void) const
    {
        Counts value;

        noInterrupts();
        value = m_latest;
        interrupts();

        return value;
    }

    private:

    volatile Counts m_latest;
    uint8_t         m_shift;
};

#endif  // SENSE_FREEADC_H
