// Una corriente medida por un sensor de salida analógica, a partir de cuentas
// crudas de un conversor.
//
// No toca el conversor ni sabe cuál es: recibe la cuenta por update(). Eso es lo
// que lo deja entero del lado de la aritmética, así que se compila y se prueba en
// la máquina de escritorio, que es donde un error de signo se encuentra en un
// segundo en lugar de en un banco.
//
// Lo que sí sabe son las dos cosas que convierten una cuenta en una corriente con
// sentido: dónde está el cero del sensor, y si el sensor está insertado al revés.

#ifndef SENSE_CURRENTSENSE_H
#define SENSE_CURRENTSENSE_H

#include <stdint.h>

#include <FirstOrderFilter.h>

class CurrentSense
{
    public:

    // Cuentas del conversor alrededor del cero del sensor. Con nombre porque no son
    // miliamperes: la escala que las convierte vive en la tabla de canales y la
    // aplica la computadora, que tiene punto flotante y no tiene plazo que cumplir.
    typedef int16_t Counts;

    // Dos polos. La señal es chica, así que el filtro quiere resolución: ocho bits
    // de guarda debajo de la cuenta.
    typedef FirstOrderFilter<8>        Filter;
    typedef typename Filter::Alpha     Alpha;

    // --------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección.

    Counts  zero;       // el cero del sensor, en cuentas crudas del conversor
    uint8_t invert;     // 1: la corriente sale negativa cuando un comando positivo la hace circular
    Counts  i;          // la corriente medida, filtrada, alrededor de `zero`

    constexpr explicit CurrentSense(Counts initial_zero)
        : zero(initial_zero)
        , invert(0)
        , i(0)
        , m_filt()
    {
    }

    // Una cuenta cruda del conversor -> la corriente publicada.
    //
    // El cero primero y el signo después: `zero` está en cuentas del conversor, que
    // es el dominio en el que la computadora lo mide, así que restarlo no puede
    // depender de hacia dónde se cuente después. Quien calcule el cero deshace esta
    // misma composición.
    void update(Counts raw)
    {
        Counts sensed = (Counts)(raw - zero);

        if (invert)
        {
            sensed = (Counts)-sensed;
        }

        i = clamp(m_filt[1].update(m_filt[0].update(sensed)));
    }

    void set_alpha(Alpha alpha)
    {
        m_filt[0].set_alpha(alpha);
        m_filt[1].set_alpha(alpha);
    }

    // La cuenta cruda que daría una corriente dada, que es el camino de vuelta de
    // update(). Existe para que quien mida el cero desde afuera no tenga que
    // reconstruir la composición a mano, que es justo donde un signo se espeja.
    Counts raw_for(Counts current) const
    {
        const Counts signed_current = invert ? (Counts)-current : current;
        return (Counts)(signed_current + zero);
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

#endif  // SENSE_CURRENTSENSE_H
