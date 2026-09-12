// Un ángulo de una vuelta convertido en una posición que el lazo puede usar:
// referido a un cero elegido, desenrollado para que no salte al dar la vuelta, y
// filtrado.
//
// No sabe de qué sensor viene la cuenta. Recibe el ángulo ya corregido por
// update(), y de la calibración tampoco sabe nada: eso deja a este módulo entero
// del lado de la aritmética, comprobable en la máquina de escritorio, y deja la
// decisión de corregir o no en manos de quien la toma.

#ifndef ANGLESENSOR_ANGLETRACKER_H
#define ANGLESENSOR_ANGLETRACKER_H

#include <stdint.h>

#include <FirstOrderFilter.h>

template <uint16_t PerRev = 4096>
class AngleTracker
{
    public:

    // Adentro de la vuelta y a lo largo de las vueltas, cada uno con nombre: el
    // primero da la vuelta a las PerRev cuentas y el segundo no, y mezclarlos es
    // justo el error que hace que un lazo ordene una vuelta entera para corregir un
    // grado.
    typedef int16_t Counts;
    typedef int32_t Unwrapped;

    // Dos polos. La posición cuenta libre y necesita el margen, así que le alcanzan
    // cuatro bits de guarda.
    typedef FirstOrderFilter<4>    Filter;
    typedef typename Filter::Alpha Alpha;

    static const uint16_t PER_REV = PerRev;

    // --------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección.

    Counts    offset;   // la cuenta que se lee como cero
    Counts    y;        // el ángulo medido adentro de la vuelta, referido a `offset`
    Unwrapped y_uw;     // el mismo ángulo desenrollado, sin filtrar
    Unwrapped y_uwf;    // desenrollado y filtrado

    constexpr AngleTracker()
        : offset(0)
        , y(0)
        , y_uw(0)
        , y_uwf(0)
        , m_filt()
    {
    }

    // El camino más corto de `a` hasta `b` en la circunferencia, para que una
    // referencia apenas pasado el punto de vuelta no ordene una vuelta entera en el
    // sentido equivocado.
    static Counts wrapped_error(Counts a, Counts b)
    {
        const Counts half = (Counts)(PerRev / 2);
        return (Counts)((((Counts)(a - b) + half) & (Counts)(PerRev - 1)) - half);
    }

    // Un ángulo crudo de la vuelta --ya corregido, si alguien lo corrige-- se
    // convierte en la posición que el lazo lee.
    //
    // El imán gira en sentido contrario al eje, de ahí que `offset` entre como el
    // primer término: es la cuenta que se lee como cero.
    void update(Counts corrected)
    {
        const Counts next = wrapped_error(offset, corrected);

        y_uw += wrapped_error(next, y);
        y_uwf = m_filt[1].update(m_filt[0].update(y_uw));
        y     = next;
    }

    void set_alpha(Alpha alpha)
    {
        m_filt[0].set_alpha(alpha);
        m_filt[1].set_alpha(alpha);
    }

    // Vuelve a arrancar el desenrollado desde cero sin que el filtro arrastre el
    // valor viejo. Sin esto, tomar la posición actual como cero deja la salida
    // filtrada bajando hacia el cero nuevo durante varias constantes de tiempo, y
    // eso se ve en la telemetría como un transitorio que nadie ordenó.
    void rezero(void)
    {
        y_uw  = 0;
        y_uwf = 0;
        m_filt[0].reset(0);
        m_filt[1].reset(0);
    }

    private:

    Filter m_filt[2];
};

#endif  // ANGLESENSOR_ANGLETRACKER_H
