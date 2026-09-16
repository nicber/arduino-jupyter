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
// arranca sin corregir no aplica una corrección equivocada, y el caso crítico no es
// la tabla que falta sino la tabla desactualizada, de otro montaje, que se aplica
// en silencio.
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
    // porque las mediciones lo exigen: con el AGC en media escala --la distancia
    // correcta-- el segundo armónico midió entre 105 y 108 cuentas, 9,2 a 9,5 grados. El AGC informa la distancia
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
        // sirve para los dos signos; el `e < 0 ? -4 : 4`, que parece la alternativa
        // natural, redondea mal los negativos chicos: -3/8 daría -1 en lugar de 0.
        return (Counts)((eighths + 4) >> 3);
    }

    // El ángulo corregido, adentro de la vuelta. La tabla se indexa con la cuenta
    // cruda, porque una vez desenrollado ese ángulo ya no está. Quien desenrolla
    // antes de corregir, como Banco, suma al ángulo desenrollado la diferencia
    // entre el corregido y el crudo (AngleTracker::wrapped_error()), que son unas
    // pocas cuentas.
    //
    // Se llama corrected() y no apply() para no quedar sobrecargado contra la
    // escritura de una entrada: los dos tomarían un entero y el compilador elegiría
    // por el ancho del tipo, una resolución de sobrecarga difícil de seguir al leer.
    Counts corrected(Counts raw) const
    {
        return (Counts)((raw - correction(raw)) & (Counts)(PerRev - 1));
    }

    // Suma de Fletcher sobre la tabla, para que las Size escrituras se verifiquen
    // con una sola lectura.
    //
    // Fletcher y no una suma simple porque una suma no distingue una tabla de otra
    // con dos entradas intercambiadas, y una entrada en el índice equivocado es
    // exactamente el error que se comete acá. El segundo acumulador pesa cada
    // entrada por su posición, así que un intercambio lo mueve en (j-i)(x_j - x_i).
    //
    // Sobre palabras de 16 bits y módulo 65535, no sobre bytes y módulo 256. Con
    // bytes módulo 256 esa diferencia se anula cada vez que (j-i)(x_j - x_i) es
    // múltiplo de 256, o sea en el 2,3 % de los intercambios y en el 25 % de los que
    // están a distancia 32 --justo el error que esta función existe para encontrar--.
    // Módulo 255 lo baja sólo a 1,4 %: 255 = 3·5·17 tiene demasiados divisores. Sobre
    // palabras módulo 65535 quedan 2 de cada 100 000, que es el piso de cualquier
    // suma de 16 bits (1/65536) y no una debilidad del método.
    //
    // La suma módulo 65535 es la del complemento a uno --el acarreo vuelve al bit
    // 0--, que en un AVR son unas pocas instrucciones y no una división.
    //
    // Los dos acumuladores se pliegan en un u16 porque es lo que viaja, y se pliegan
    // sumando y no con un o-exclusivo: una entrada corrida en delta mueve `a` en
    // delta y `b` en (Size - i)·delta, así que en la última entrada los dos se mueven
    // igual y un o-exclusivo los cancelaría. Medido sobre 60 tablas: plegando con
    // o-exclusivo se escapa el 0,79 % de las entradas corridas, y sumando, el
    // 0,05 %. Para un intercambio los dos dan lo mismo, 0,0025 %, que es el piso de
    // cualquier suma de 16 bits (1/65536 = 0,0015 %).
    uint16_t checksum(void) const
    {
        uint16_t a = 0;
        uint16_t b = 0;

        for (uint8_t i = 0; i < Size; i++)
        {
            a = ones_add(a, (uint16_t)entry[i]);
            b = ones_add(b, a);
        }

        return (uint16_t)(a + b);
    }

    // Suma módulo 65535: la del complemento a uno, con el acarreo de vuelta al bit
    // 0. No puede dar la vuelta al sumar el acarreo: haría falta x + y = 131071, y
    // con los dos en 16 bits el máximo es 131070.
    static uint16_t ones_add(uint16_t x, uint16_t y)
    {
        const uint16_t s = (uint16_t)(x + y);
        return (uint16_t)(s < x ? (uint16_t)(s + 1) : s);
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

    // Lo mismo, pero sólo cuando el parámetro efectivamente cambió desde la última vez.
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
