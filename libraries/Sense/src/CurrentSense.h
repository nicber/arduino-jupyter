// Una corriente medida por un sensor de salida analógica, a partir de cuentas
// crudas de un conversor.
//
// No toca el conversor ni sabe cuál es: recibe la cuenta por update(). Eso es lo
// que lo deja entero del lado de la aritmética, así que se compila y se prueba en
// la máquina de escritorio.
//
// Lo único que sabe es dónde está el cero del sensor. No filtra --un filtro acá se
// confunde con la planta que se mide-- ni invierte el signo, que lo aplica quien
// lo usa igual que el del ángulo (Banco con `cur_inv`).

#ifndef SENSE_CURRENTSENSE_H
#define SENSE_CURRENTSENSE_H

#include <stdint.h>

class CurrentSense
{
    public:

    // Cuentas del conversor alrededor del cero del sensor. Con nombre porque no son
    // miliamperes: la escala que las convierte vive en la tabla de canales y la
    // aplica la computadora.
    typedef int16_t Counts;

    Counts zero;        // el cero del sensor, en cuentas crudas del conversor
    Counts i;           // la corriente medida, alrededor de `zero`, en la unidad de update()

    constexpr explicit CurrentSense(Counts initial_zero)
        : zero(initial_zero)
        , i(0)
    {
    }

    void update(Counts raw)
    {
        i = (Counts)(raw - zero);
    }

    // Lo mismo con la cuenta en dieciseisavos (ver Sense/SupplyRatio.h). El cero se
    // queda en cuentas enteras a propósito: es la unidad en la que la computadora lo
    // mide y la que informa `cur_zero`, y en dieciseisavos un reposo de 2000 cuentas
    // son 32000, que no le deja al int16 margen para la tolerancia del divisor. La
    // resta va en 32 bits por lo mismo, y satura.
    void update_q4(int32_t raw_q4)
    {
        const int32_t d = raw_q4 - ((int32_t)zero << 4);
        i = (Counts)(d > INT16_MAX ? INT16_MAX : (d < INT16_MIN ? INT16_MIN : d));
    }
};

#endif  // SENSE_CURRENTSENSE_H
