// Punto fijo con escala definida en tiempo de compilación, para lazos de control
// que no pueden tocar el punto flotante.
//
// Un `Fixed<Raw, Frac>` es un entero de tipo `Raw` que guarda un valor escalado
// por 2^Frac. La escala es un parámetro de plantilla, así que elegirla es gratis:
// todo desplazamiento se resuelve en tiempo de compilación y una conversión entre
// escalas que una versión en punto flotante habría hecho en tiempo de ejecución
// directamente no existe.
//
// Por qué no una biblioteca existente: fpm y CNL son las buenas, y las dos
// necesitan <type_traits> y <limits>, que avr-g++ no provee. El único rasgo que
// hace falta acá son cuatro líneas, así que está escrito más abajo.
//
// Cómo elegir una escala. Frac compra resolución a costa de rango: un valor
// necesita `Frac` bits debajo de la coma y los suficientes arriba para su mayor
// magnitud, y los dos tienen que entrar en `Raw`. Una ganancia de 0,5 y una de
// 0,00005 quieren escalas muy distintas, que es justamente por qué esto es un
// parámetro y no 16.
//
//   Fixed<int32_t, 22> kp;   // +/-511,     resolución 2.4e-7
//   Fixed<int32_t, 30> ki;   // +/-1.99,    resolución 9.3e-10
//
// Costo en un AVR. Los productos de dos `int32_t` pasan por un intermedio de
// 64 bits (__muldi3, del orden de 200 ciclos); los productos de dos `int16_t`
// compilan a un puñado de instrucciones MUL. Conviene el `Raw` más angosto que
// contenga el rango, y mantener las multiplicaciones fuera del bucle interno
// donde se sabe que los operandos son chicos.
//
// Toda operación que descarta bits redondea al más cercano, con los empates
// hacia más infinito. Un desplazamiento aritmético a la derecha por sí solo
// redondea hacia menos infinito, lo que le pone medio LSB de continua a un
// término proporcional: un tirón permanente hacia un lado que después el lazo de
// control tiene que integrar para sacárselo de encima.

#ifndef CONTROLMATH_FIXEDPOINT_H
#define CONTROLMATH_FIXEDPOINT_H

#include <stdint.h>

// El único rasgo que hace falta: el tipo en el que se calcula el producto de dos
// `T`.
template <typename T> struct FixedWider;
template <> struct FixedWider<int8_t>  { typedef int16_t type; };
template <> struct FixedWider<int16_t> { typedef int32_t type; };
template <> struct FixedWider<int32_t> { typedef int64_t type; };

template <typename Raw, unsigned Frac>
class Fixed
{
    public:

    typedef Raw                            raw_type;
    typedef typename FixedWider<Raw>::type wide_type;

    static const unsigned FRAC = Frac;
    static const Raw      ONE  = (Raw)1 << Frac;

    // Medio LSB, para redondear en lugar de truncar. Frac == 0 es un entero
    // común y no tiene nada que redondear, y el desplazamiento de abajo igual
    // tiene que ser legal para que el compilador acepte la rama muerta.
    static const Raw      HALF = Frac ? ((Raw)1 << (Frac ? Frac - 1 : 0)) : 0;

    constexpr Fixed(void) : m_raw(0) {}

    // Etiquetado para que un entero pelado no pueda convertirse en silencio en un
    // Fixed con una escala en la que nunca estuvo.
    struct FromRaw {};
    constexpr Fixed(Raw value, FromRaw) : m_raw(value) {}

    static constexpr Fixed from_raw(Raw value)
    {
        return Fixed(value, FromRaw());
    }

    // El único lugar donde se permite un float. Llamarlo sobre un literal y se
    // reduce a una constante; llamarlo cuando cambia una ganancia enviada por la
    // computadora y cuesta una multiplicación en punto flotante, fuera del paso
    // de control.
    static constexpr Fixed from_float(float value)
    {
        return from_raw((Raw)(value * (float)ONE + (value >= 0.0f ? 0.5f : -0.5f)));
    }

    static constexpr Fixed from_int(Raw value)
    {
        return from_raw((Raw)(value << Frac));
    }

    constexpr Raw   raw(void)      const { return m_raw; }
    constexpr float to_float(void) const { return (float)m_raw / (float)ONE; }
    constexpr Raw   to_int(void)   const { return (Raw)((m_raw + HALF) >> Frac); }

    // Multiplicar un entero común por este valor y recuperar un entero común: el
    // caballito de batalla de una ley de control, donde una ganancia en unidades
    // de ingeniería se encuentra con una señal en cuentas crudas. El producto se
    // forma en el tipo ancho, así que el único desborde del que preocuparse es el
    // del resultado.
    template <typename Int>
    constexpr Raw scale(Int x) const
    {
        return round_shift((wide_type)m_raw * (wide_type)x);
    }

    // scale(), pero exacto a lo largo de una secuencia de llamadas. Los bits por
    // debajo del LSB del resultado se devuelven en `carry` en lugar de perderse
    // por redondeo, así que pasarle el mismo carry a la llamada siguiente los
    // recupera. Una suma corriente de estos es exacta, mientras que una suma de
    // scale() se desvía hasta medio LSB por término y, peor todavía, se frena del
    // todo apenas un término redondea a cero.
    //
    // `carry` tiene que arrancar en cero y pertenece a la secuencia, no al valor:
    // un acumulador por cada suma corriente.
    template <typename Int>
    Raw scale_carry(Int x, Raw& carry) const
    {
        wide_type product = (wide_type)m_raw * (wide_type)x + carry;

        // El desplazamiento trunca hacia abajo, así que el resto nunca es
        // negativo y la separación es exacta en lugar de apenas aproximada.
        carry = (Raw)product & (ONE - 1);
        return (Raw)(product >> Frac);
    }

    // Misma escala a la entrada, misma escala a la salida.
    constexpr Fixed operator+(Fixed o) const { return from_raw((Raw)(m_raw + o.m_raw)); }
    constexpr Fixed operator-(Fixed o) const { return from_raw((Raw)(m_raw - o.m_raw)); }
    constexpr Fixed operator-(void)    const { return from_raw((Raw)(-m_raw)); }

    constexpr Fixed operator*(Fixed o) const
    {
        return from_raw(round_shift((wide_type)m_raw * (wide_type)o.m_raw));
    }

    // Reescalar a otra cantidad de bits fraccionarios. Las dos cuentas de
    // desplazamiento se acotan porque las dos ramas del ternario se compilan,
    // aunque para un par de escalas dado sólo una pueda tomarse.
    template <unsigned Frac2>
    constexpr Fixed<Raw, Frac2> rescale(void) const
    {
        return Fixed<Raw, Frac2>::from_raw(
            Frac2 >= Frac ? (Raw)(m_raw << (Frac2 >= Frac ? Frac2 - Frac : 0))
                          : (Raw)(m_raw >> (Frac >= Frac2 ? Frac - Frac2 : 0)));
    }

    constexpr bool operator==(Fixed o) const { return m_raw == o.m_raw; }
    constexpr bool operator!=(Fixed o) const { return m_raw != o.m_raw; }
    constexpr bool operator< (Fixed o) const { return m_raw <  o.m_raw; }
    constexpr bool operator> (Fixed o) const { return m_raw >  o.m_raw; }
    constexpr bool operator<=(Fixed o) const { return m_raw <= o.m_raw; }
    constexpr bool operator>=(Fixed o) const { return m_raw >= o.m_raw; }

    private:

    // Baja `product` de vuelta Frac bits, redondeando al más cercano. El
    // desplazamiento en sí trunca hacia abajo, así que sumarle medio LSB antes lo
    // convierte en redondeo al más cercano con los empates para arriba, que es
    // simétrico respecto del cero, a diferencia del truncamiento.
    static constexpr Raw round_shift(wide_type product)
    {
        return (Raw)((product + (wide_type)HALF) >> Frac);
    }

    Raw m_raw;
};

#endif  // CONTROLMATH_FIXEDPOINT_H
