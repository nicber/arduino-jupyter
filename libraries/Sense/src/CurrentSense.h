// Una corriente medida por un sensor de salida analógica, a partir de cuentas
// crudas de un conversor.
//
// No toca el conversor ni sabe cuál es: recibe la cuenta por update(). Eso es lo
// que lo deja entero del lado de la aritmética, así que se compila y se prueba en
// la máquina de escritorio.
//
// Lo único que sabe es dónde está el cero del sensor. No filtra --un filtro acá se
// confunde con la planta que se mide-- ni da vuelta el signo, que se resuelve del
// lado de la computadora igual que el del ángulo.

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
    Counts i;           // la corriente medida, alrededor de `zero`

    constexpr explicit CurrentSense(Counts initial_zero)
        : zero(initial_zero)
        , i(0)
    {
    }

    void update(Counts raw)
    {
        i = (Counts)(raw - zero);
    }
};

#endif  // SENSE_CURRENTSENSE_H
