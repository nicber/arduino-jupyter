// La posición que lee un lazo: el ángulo desenrollado, referido a un cero elegido,
// con el signo del eje y filtrado.
//
// AngleTracker desenrolla y nada más, que es lo que quiere un banco de
// identificación. Las tres cosas que un lazo necesita encima viven acá, del lado
// del control, para que quien no cierre un lazo no cargue con ellas.

#ifndef CONTROL_LOOPANGLE_H
#define CONTROL_LOOPANGLE_H

#include <stdint.h>

#include <AngleTracker.h>
#include <FirstOrderFilter.h>

template <uint16_t PerRev = 4096>
class LoopAngle
{
    public:

    typedef AngleTracker<PerRev>            Tracker;
    typedef typename Tracker::Counts        Counts;
    typedef typename Tracker::Unwrapped     Unwrapped;

    // Dos polos. La posición cuenta libre y necesita el margen, así que le alcanzan
    // cuatro bits de guarda.
    typedef FirstOrderFilter<4>             Filter;
    typedef typename Filter::Alpha          Alpha;

    // --------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección.

    Counts    offset;   // la cuenta que se lee como cero
    Tracker   track;    // y, referido a `offset`; y_uw, desenrollado sin filtrar
    Unwrapped y_uwf;    // desenrollado y filtrado

    constexpr LoopAngle()
        : offset(0)
        , track()
        , y_uwf(0)
        , m_filt()
    {
    }

    // Un ángulo crudo de la vuelta --ya corregido, si alguien lo corrige-- se
    // convierte en la posición que el lazo lee.
    //
    // El imán gira en sentido contrario al eje, de ahí que `offset` entre como el
    // primer término: es la cuenta que se lee como cero.
    void update(Counts corrected)
    {
        track.update(Tracker::wrapped_error(offset, corrected));
        y_uwf = m_filt[1].update(m_filt[0].update(track.y_uw));
    }

    void set_alpha(Alpha alpha)
    {
        m_filt[0].set_alpha(alpha);
        m_filt[1].set_alpha(alpha);
    }

    private:

    Filter m_filt[2];
};

#endif  // CONTROL_LOOPANGLE_H
