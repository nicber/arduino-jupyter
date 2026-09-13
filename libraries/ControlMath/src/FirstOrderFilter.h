// Pasabajos de un polo, hecho íntegramente con enteros.
//
//   y[n] = y[n-1] + alpha * (x[n] - y[n-1]),   alpha = dt / (tau + dt)
//
// que es el polo exacto de una sección RC muestreada con período `dt`. Poner dos
// en cascada da una respuesta de dos polos; al hacerlo la frecuencia de corte
// baja en un factor de alrededor de 1,55, que en general es lo que se quería de
// todos modos.
//
// Lo que rompe un IIR entero ingenuo es que `alpha * (x - y)` redondea a cero
// mucho antes de que y llegue a x: con una constante de tiempo larga, la salida
// se frena varias cuentas antes y se queda ahí. Empujar el estado una cuenta cada
// vez que la diferencia es distinta de cero —el parche habitual— sólo cambia el
// estancamiento por un ciclo límite y un piso de velocidad de cambio.
//
// Éste, en cambio, se guarda el resto, mediante Fixed::scale_carry(). La parte de
// `alpha * (x - y)` que no entra en el estado se arrastra a la muestra siguiente,
// así que un escalón demasiado chico para mover el estado ahora lo mueve unas
// muestras después y la salida converge de manera exacta, para cualquier alpha y
// cualquier entrada. No se descarta nada, así que no hay sesgo ni zona muerta.
//
// El estado es un Fixed con `Guard` bits fraccionarios; los bits de guarda son
// una escala, así que se escriben como tal. Cómo dimensionarlo: el estado es un
// int32_t, así que la entrada tiene que quedar dentro de +/-2^(31 - Guard). Guard
// compra suavidad durante un transitorio, no exactitud en régimen permanente —de
// eso ya se ocupa el arrastre—, así que un contador que corre libre puede
// arreglarse con Guard = 4 (+/-2^27 cuentas) y un canal de ADC acotado puede
// gastar 8.

#ifndef CONTROLMATH_FIRSTORDERFILTER_H
#define CONTROLMATH_FIRSTORDERFILTER_H

#include <stdint.h>

#include "FixedPoint.h"

template <unsigned Guard = 8>
class FirstOrderFilter
{
    public:

    // alpha es una fracción en [0, 1]; Q16 la resuelve con 1.5e-5, que con
    // muestreo de 1 ms da una constante de tiempo de hasta alrededor de un
    // minuto.
    typedef Fixed<int32_t, 16> Alpha;

    // El estado propio del filtro: la entrada con `Guard` bits fraccionarios
    // debajo.
    typedef Fixed<int32_t, Guard> State;

    // tau y dt en segundos. tau <= 0 da alpha = 1: deja pasar la señal tal cual,
    // que es la manera natural de que la computadora apague el filtro.
    static constexpr Alpha alpha_for(float tau, float dt)
    {
        return tau > 0.0f ? Alpha::from_float(dt / (tau + dt)) : Alpha::from_int(1);
    }

    // constexpr para que las instancias de filtro a nivel de archivo las
    // inicialice el cargador y no un constructor global que corra antes de
    // setup().
    constexpr explicit FirstOrderFilter(Alpha alpha = Alpha::from_int(1),
                                        int32_t initial = 0)
        : m_alpha(alpha)
        , m_state(State::from_int(initial))
        , m_carry(0)
    {
    }

    // Es seguro llamarlo con el filtro corriendo: el estado está en unidades de
    // salida, así que la salida no salta cuando se mueve la frecuencia de corte.
    void set_alpha(Alpha alpha) { m_alpha = alpha; }

    void reset(int32_t x)
    {
        m_state = State::from_int(x);
        m_carry = 0;
    }

    int32_t update(int32_t x)
    {
        int32_t step = m_alpha.scale_carry(State::from_int(x).raw() - m_state.raw(),
                                           m_carry);

        m_state = m_state + State::from_raw(step);
        return value();
    }

    int32_t value(void) const { return m_state.to_int(); }

    private:

    Alpha   m_alpha;
    State   m_state;
    int32_t m_carry;    // resto de alpha * diferencia, en la fracción de Alpha
};

#endif  // CONTROLMATH_FIRSTORDERFILTER_H
