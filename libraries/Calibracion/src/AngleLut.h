// La corrección del error de ángulo de un sensor de una vuelta, como una tabla
// indexada por el ángulo crudo.
//
// Un imán descentrado respecto del integrado corre la lectura en una cantidad que
// depende del ángulo y que se repite vuelta tras vuelta: al lazo se le presenta
// como ondulación de velocidad, y ninguna ganancia la saca. La forma del error son
// los primeros armónicos de la vuelta mecánica.
//
// Acá vive nada más que la corrección. Quién la calcula y de dónde sale es asunto
// de la computadora, y esta tabla no se guarda en la placa: un dispositivo que
// arranca sin corregir no puede mentirle a nadie, y el caso feo no es la tabla que
// falta sino la tabla vieja, de otro montaje, que se aplica en silencio.
//
// Es aritmética entera y nada más, así que se compila y se prueba en la máquina de
// escritorio.

#ifndef CALIBRACION_ANGLELUT_H
#define CALIBRACION_ANGLELUT_H

#include <stdint.h>

template <uint16_t PerRev = 4096, uint8_t Size = 64>
class AngleLut
{
    public:

    // Los dos dominios que conviven acá, cada uno con nombre. Una entrada está en
    // octavos de cuenta y una corrección en cuentas, y sumar uno donde va el otro es
    // un error de ocho veces que no se nota hasta que el eje gira.
    typedef int16_t Counts;
    typedef int16_t Eighths;

    static const uint8_t  SIZE    = Size;
    static const uint16_t PER_REV = PerRev;

    // Cuántas cuentas cubre cada entrada. Con 64 entradas sobre 4096 cuentas son 64
    // cuentas --5,6 grados-- de separación, ocho puntos por ciclo del octavo
    // armónico; la interpolación se hace cargo del resto.
    static const uint16_t SPAN = PerRev / Size;

    // El mismo número como corrimiento. Hace falta porque la interpolación divide un
    // valor que puede ser negativo, y ahí `/ SPAN` y `>> SPAN_BITS` no son lo mismo:
    // la división redondea hacia cero y el corrimiento hacia abajo. La computadora
    // reproduce esta tabla con la aritmética de numpy, que corre hacia abajo, así
    // que el corrimiento es el que mantiene las dos en acuerdo exacto.
    static constexpr uint8_t bits_of(uint16_t n)
    {
        return n <= 1 ? 0 : (uint8_t)(1 + bits_of((uint16_t)(n >> 1)));
    }
    static const uint8_t SPAN_BITS = bits_of(SPAN);

    // El recorte de una entrada, en octavos. Son +/-511 cuentas, o +/-45 grados.
    //
    // La resolución de un octavo existe porque el error puede ser de unas pocas
    // cuentas: en cuentas enteras la tabla tendría tres o cuatro valores distintos y
    // sería un escalón, no una corrección. Y el rango es de int16 y no de int8
    // porque el banco lo pidió: con el AGC en media escala --la distancia correcta--
    // el segundo armónico midió 105 cuentas, 9,3 grados. El AGC informa la distancia
    // y no el centrado, así que un imán puede estar a la distancia justa y de todos
    // modos torcido, y ahí el error es real y grande.
    static const Eighths MAX = 4095;

    // El valor de `packed` que quiere decir «nada que hacer»: un índice que no
    // existe. De ahí sale el valor de arranque del parámetro.
    static const uint32_t NOTHING = 0xFFFFFFFFUL;

    // Las entradas, públicas porque quien las cargue de PROGMEM copia sobre ellas.
    Eighths entry[Size];

    constexpr AngleLut() : entry(), m_applied(NOTHING) {}

    // La corrección en cuentas para un ángulo crudo, interpolada linealmente entre
    // las dos entradas que lo rodean.
    Counts correction(Counts raw) const
    {
        // Sin signo antes de partir el ángulo en índice y fracción: el índice sale de
        // un corrimiento y la fracción de una máscara, y las dos cosas valen para
        // todo el rango de la vuelta sólo si el valor no se lee como negativo.
        const uint16_t r    = (uint16_t)raw;
        const uint8_t index = (uint8_t)(r >> SPAN_BITS) & (uint8_t)(Size - 1);
        const uint16_t frac = r & (uint16_t)(SPAN - 1);

        const int32_t a = (int32_t)entry[index];
        const int32_t b = (int32_t)entry[(uint8_t)(index + 1) & (uint8_t)(Size - 1)];

        // Un int32 en el medio y no un int16: con entradas de hasta +/-4095 octavos
        // la suma llega a 4095 * 64 = 262080, que no entra en 16 bits. Son unas
        // decenas de ciclos más, una vez por período de control.
        const int32_t eighths = (a * (int32_t)(SPAN - frac) + b * (int32_t)frac)
                              >> SPAN_BITS;

        // Redondeo al medio hacia arriba. Con corrimiento aritmético `(e + 4) >> 3`
        // sirve para los dos signos; el `e < 0 ? -4 : 4` que uno escribe de reflejo
        // redondea mal los negativos chicos: 3/8 daría -1 en lugar de 0.
        return (Counts)((eighths + 4) >> 3);
    }

    // El ángulo corregido, que es lo que el lazo quiere. Se aplica sobre la cuenta
    // cruda y antes de desenrollar, porque la tabla se indexa con el ángulo de
    // adentro de la vuelta y una vez desenrollado ese ángulo ya no está.
    //
    // Se llama corrected() y no apply() para no quedar sobrecargado contra la
    // escritura de una entrada: los dos tomarían un entero y el compilador elegiría
    // por el ancho del tipo, que es la clase de resolución que nadie quiere leer.
    Counts corrected(Counts raw) const
    {
        return (Counts)((raw - correction(raw)) & (Counts)(PerRev - 1));
    }

    // Suma de Fletcher de 16 bits sobre la tabla, para que las Size escrituras se
    // verifiquen con una sola lectura.
    //
    // Fletcher y no una suma pelada porque una suma no distingue una tabla de otra
    // con dos entradas intercambiadas, y una entrada en el índice equivocado es
    // exactamente el error que se comete acá. Se recorre byte por byte, primero el
    // bajo y después el alto de cada entrada, para que la computadora pueda
    // reproducirla sin saber nada del orden de bytes del procesador.
    uint16_t checksum(void) const
    {
        uint8_t a = 0;
        uint8_t b = 0;

        for (uint8_t i = 0; i < Size; i++)
        {
            const uint16_t v = (uint16_t)entry[i];

            a += (uint8_t)v;
            b += a;
            a += (uint8_t)(v >> 8);
            b += a;
        }

        return (uint16_t)(((uint16_t)b << 8) | a);
    }

    // Una entrada por escritura, empaquetada como (índice << 16) | valor.
    //
    // El índice viaja adentro del mismo valor para que dos escrituras seguidas nunca
    // sean iguales por casualidad: quien llama aplica al ver que el parámetro
    // cambió, y con índice y valor en parámetros separados una tabla con dos
    // entradas iguales seguidas perdería la segunda. Así se carga la tabla entera
    // sin agregarle un comando al protocolo: son Size escrituras comunes, cada una
    // con la misma respuesta verificada que cualquier otra.
    //
    // Devuelve false para un índice que no existe, que es de dónde sale el valor de
    // «nada que hacer».
    bool write_packed(uint32_t packed)
    {
        const uint16_t index = (uint16_t)(packed >> 16);

        if (index >= Size)
        {
            return false;
        }

        Eighths value = (Eighths)(uint16_t)packed;

        // Acotar acá y no confiar: una entrada fuera de rango desbordaría la
        // interpolación, y el enlace acepta cualquier entero que le manden.
        if (value >  MAX) value =  MAX;
        if (value < -MAX) value = -MAX;

        entry[index] = value;
        return true;
    }

    // Lo mismo, pero sólo cuando el parámetro de verdad cambió desde la última vez.
    //
    // Quien llama corre después de cada escritura de cualquier parámetro, así que sin
    // esto un barrido de ganancia reescribiría la misma entrada de la tabla una vez
    // por comando. El valor ya aplicado es un miembro de la tabla y no una variable
    // escondida adentro de la función de ajuste: es la tabla la que sabe qué tiene
    // puesto.
    bool apply(uint32_t packed)
    {
        if (packed == m_applied)
        {
            return false;
        }

        m_applied = packed;
        return write_packed(packed);
    }

    private:

    uint32_t m_applied;
};

#endif  // CALIBRACION_ANGLELUT_H
