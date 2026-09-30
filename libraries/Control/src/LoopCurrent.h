// La corriente que lee un lazo: con el sentido del motor y filtrada.
//
// CurrentSense resta el cero y nada más, que es lo que quiere un banco de
// identificación. El signo y el filtro viven acá, del lado del control, para que
// quien no cierre un lazo no cargue con ellos.

#ifndef CONTROL_LOOPCURRENT_H
#define CONTROL_LOOPCURRENT_H

#include <stdint.h>

#include <CurrentSense.h>
#include <FirstOrderFilter.h>

class LoopCurrent
{
    public:

    typedef CurrentSense::Counts Counts;

    // Dos polos. La señal es chica, así que el filtro quiere resolución: ocho bits
    // de guarda debajo de la cuenta.
    typedef FirstOrderFilter<8>        Filter;
    typedef typename Filter::Alpha     Alpha;

    // --------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección.

    CurrentSense sense; // el cero del sensor, en cuentas crudas del conversor
    uint8_t invert;     // 1: la corriente sale negativa cuando un comando positivo la hace circular
    Counts  i;          // la corriente medida, con signo y filtrada

    constexpr explicit LoopCurrent(Counts initial_zero)
        : sense(initial_zero)
        , invert(0)
        , i(0)
        , m_filt()
    {
    }

    // El cero primero y el signo después: `zero` está en cuentas del conversor, que
    // es el dominio en el que la computadora lo mide, así que restarlo no puede
    // depender de hacia dónde se cuente después.
    void update(Counts raw)
    {
        sense.update(raw);

        // La negación va en 32 bits y vuelve por clamp(): -INT16_MIN no entra en un
        // int16 y se queda en INT16_MIN, o sea que el signo no se aplicaría. Y se
        // alcanza sin ninguna perilla rara: una fila sin conversiones de A0 publica
        // 0 cuentas equivalentes, que con el cero en 2048 da exactamente INT16_MIN.
        const Counts sensed = invert ? clamp(-(int32_t)sense.i) : sense.i;

        i = clamp(m_filt[1].update(m_filt[0].update(sensed)));
    }

    void set_alpha(Alpha alpha)
    {
        m_filt[0].set_alpha(alpha);
        m_filt[1].set_alpha(alpha);
    }

    private:

    static Counts clamp(int32_t v)
    {
        if (v < INT16_MIN) return INT16_MIN;
        if (v > INT16_MAX) return INT16_MAX;
        return (Counts)v;
    }

    Filter m_filt[2];
};

#endif  // CONTROL_LOOPCURRENT_H
