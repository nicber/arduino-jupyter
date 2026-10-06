// La corriente que lee un lazo: en cuentas enteras y filtrada.
//
// CurrentSense entrega la corriente con el cero restado y el signo puesto, en
// dieciseisavos de cuenta, que es lo que quiere un banco de identificación. El
// filtro vive acá, del lado del control, para que quien no cierre un lazo no cargue
// con él; y las cuentas enteras también, porque son la unidad en la que están
// afinadas las ganancias del PID cuando el lazo cierra sobre la corriente.

#ifndef CONTROL_LOOPCURRENT_H
#define CONTROL_LOOPCURRENT_H

#include <stdint.h>

#include <CurrentSense.h>
#include <FirstOrderFilter.h>

class LoopCurrent
{
    public:

    typedef int16_t Counts;

    // Dos polos. La señal es chica, así que el filtro quiere resolución: ocho bits
    // de guarda debajo de la cuenta.
    typedef FirstOrderFilter<8>        Filter;
    typedef typename Filter::Alpha     Alpha;

    Counts i;           // la corriente, en cuentas alrededor del cero, filtrada

    constexpr LoopCurrent()
        : i(0)
        , m_filt()
    {
    }

    // `i_q4` es CurrentSense::i. Se redondea una sola vez, acá: en 32 bits, porque
    // INT16_MAX más el medio dieciseisavo no entra en un int16.
    void update(int16_t i_q4)
    {
        static const uint8_t F = CurrentSense::FRAC_BITS;
        const Counts c = (Counts)(((int32_t)i_q4 + (1 << (F - 1))) >> F);

        i = clamp(m_filt[1].update(m_filt[0].update(c)));
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
