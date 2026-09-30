// Media móvil de N muestras, con un buffer circular.
//
// Dos operaciones por muestra: entra la nueva y sale la de hace N. El buffer es
// circular, así que nada se mueve de lugar --correr una ventana de cuatro a mano
// son seis movimientos por muestra, y a 5 kHz eso se nota-- y el índice se envuelve
// con una comparación en lugar de un módulo, que en un AVR es una llamada a
// biblioteca.
//
// N es un parámetro de plantilla, así que la división que cierra la media es por
// una constante y se elige en tiempo de compilación:
//
//   N potencia de dos    un corrimiento, y nada más.
//   cualquier otra N     una división entera, que en un AVR es una llamada.
//
// Vale la pena anotar por qué lo segundo no es una multiplicación por el recíproco
// precalculado, que es lo que uno esperaría y lo que hace un x86. Dos razones, las
// dos medidas:
//
// 1. **avr-gcc no la hace solo.** Con -O2 y -mmcu=atmega328p, `total / 10` emite
//    `call __divmodhi4` con `Sum` de 16 bits y `call __divmodsi4` con la de 32. No
//    hay reducción a multiplicación en ningún ancho.
//
// 2. **Escrita a mano sería más rápida pero no exacta, y para N par no puede
//    serlo.** Multiplicar por el recíproco en 16 bits cuesta `__usmulhisi3`, unos
//    30 ciclos contra los ~160 de `__divmodhi4`: cinco veces menos. Pero con N par
//    el empate del redondeo cae sobre un entero exacto --total/N + 1/2 es entero
//    cuando total = Nk - N/2, que con N par es un entero-- y cualquier error del
//    recíproco lo rompe hacia el lado equivocado. Barrido exhaustivo sobre los
//    65536 valores de un int16, buscando los bits fraccionarios que hacen falta:
//
//      N = 3   17 bits, exacto        N = 6     ningún ancho lo hace exacto
//      N = 5   18 bits, exacto        N = 10    ningún ancho lo hace exacto
//      N = 45  19 bits, exacto        N = 100   ningún ancho lo hace exacto
//
//    Así que el recíproco sólo serviría para N impar, y esta clase no va a tener
//    dos semánticas de redondeo según la paridad de la ventana. Quien necesite una
//    ventana grande en un lazo apretado que la elija potencia de dos, que es lo que
//    hace `RowAdc`, y no pague nada.
//
// Con N potencia de dos hay además un regalo: la suma corrida **es** la media con
// log2(N) bits fraccionarios debajo, así que mean_fixed<log2(N)>() devuelve la suma
// tal cual, sin una sola instrucción. Elegir una escala es gratis y todo
// corrimiento se resuelve en compilación; ver FixedPoint.h.
//
//   MovingAverage<int16_t, 4> ma;
//   ma.push(10); ma.push(11); ma.push(12); ma.push(13);
//   ma.mean()              -> 12          // redondeada al más cercano
//   ma.total()             -> 46          // la suma corrida
//   ma.mean_fixed<2>()     -> 46 en Q2    // o sea 11,5 exacto, y sale gratis
//
// Cómo dimensionar los dos tipos. `Sample` tiene que contener una muestra y `Sum`
// la suma de N de ellas; los dos son con signo porque es lo que Fixed sabe escalar,
// y conviene el más angosto que entre, que en un AVR es la diferencia entre una
// suma de dos bytes y una de cuatro.
//
// Todo redondeo es al más cercano con los empates hacia más infinito, que es la
// convención de FixedPoint.h: un corrimiento a secas trunca hacia abajo y le pone
// medio LSB de continua a una señal centrada en cero.
//
// Aritmética pura: se compila y se prueba en la máquina de escritorio.

#ifndef CONTROLMATH_MOVINGAVERAGE_H
#define CONTROLMATH_MOVINGAVERAGE_H

#include <stdint.h>

#include "FixedPoint.h"

// log2 de una potencia de dos, en tiempo de compilación.
template <unsigned N> struct MaLog2 { static const unsigned VALUE = 1 + MaLog2<N / 2>::VALUE; };
template <> struct MaLog2<1> { static const unsigned VALUE = 0; };

// total / N redondeando al más cercano, empates hacia más infinito. Se elige por si
// N es potencia de dos, en tiempo de compilación.
template <typename Sum, unsigned N, bool Pot2 = ((N & (N - 1)) == 0)>
struct MaDividir;

template <typename Sum, unsigned N>
struct MaDividir<Sum, N, true>
{
    // El corrimiento aritmético trunca hacia abajo; lo que falta para redondear al
    // más cercano es el último bit que se cae, que es justo el que dice si la parte
    // que se descarta pasa de la mitad.
    //
    // Escrito así y no como `(total + N/2) >> log2(N)` porque esa suma puede
    // desbordar cuando `total` está pegado al tope de `Sum` --no en esta clase,
    // donde es la suma de N muestras, pero sí en general--, y evitarlo ensanchando
    // el intermedio costaría una cuenta de 64 bits con `Sum` de 32. Así no desborda
    // nunca y no hace falta ensanchar nada. La prueba lo recorre exhaustivamente.
    static const unsigned L = MaLog2<N>::VALUE;

    static constexpr Sum de(Sum total)
    {
        return L == 0 ? total
                      : (Sum)((total >> L) + (Sum)((total >> (L ? L - 1 : 0)) & 1));
    }
};

template <typename Sum, unsigned N>
struct MaDividir<Sum, N, false>
{
    // La división del lenguaje trunca hacia cero, así que el lado negativo se
    // arregla a mano para que el redondeo sea el mismo que arriba.
    typedef typename Fixed<Sum, 0>::wide_type Wide;

    static constexpr Sum de(Sum total)
    {
        return divide((Wide)total + (Wide)(N / 2));
    }

    static constexpr Sum divide(Wide x)
    {
        return (Sum)(x >= 0 ? x / (Wide)N
                            : -(Wide)((-x + (Wide)N - 1) / (Wide)N));
    }
};

template <typename Sample, unsigned N, typename Sum = int32_t>
class MovingAverage
{
    public:

    static const unsigned SIZE = N;

    // constexpr para que una instancia a nivel de archivo la inicialice el cargador
    // y no un constructor global que corra antes de setup().
    constexpr MovingAverage(void)
        : m_buf()
        , m_total(0)
        , m_next(0)
    {
    }

    // Mete una muestra y devuelve la media de las últimas N.
    //
    // Antes de que hayan entrado N, la ventana todavía tiene ceros adentro y la
    // media los cuenta: eso es lo que hace que arranque desde cero en lugar de
    // saltar, y es lo que quiere quien promedia una señal que arranca en reposo.
    // Quien no lo quiera llama a prime() primero.
    Sum push(Sample x)
    {
        add(x);
        return mean();
    }

    // Lo mismo sin calcular la media. Para quien use total() y no quiera pagar la
    // división --en un lazo apretado, con `Sum` de 32 bits, la media que se tira
    // igual ocupa lugar--.
    void add(Sample x)
    {
        // La resta da la vuelta cuando el que sale es mayor que el que entra, y está
        // bien: el resultado verdadero entra en `Sum`, así que la cuenta módulo 2^n
        // es la correcta.
        m_total += (Sum)x - (Sum)m_buf[m_next];
        m_buf[m_next] = x;
        m_next = (uint8_t)(m_next + 1 == N ? 0 : m_next + 1);
    }

    // La media de ahora, sin meter nada.
    constexpr Sum mean(void) const { return MaDividir<Sum, N>::de(m_total); }

    // La misma media con `Frac` bits fraccionarios, para quien no quiera perderla.
    // Con N potencia de dos y Frac = log2(N) es la suma corrida sin tocar.
    template <unsigned Frac>
    constexpr Fixed<Sum, Frac> mean_fixed(void) const
    {
        return Fixed<Sum, Frac>::from_raw(
            MaDividir<Sum, N>::de((Sum)(m_total * (Sum)(1u << Frac))));
    }

    // La suma corrida tal cual. Para quien la necesite junto a otra cosa --una
    // cuenta de muestras, por ejemplo-- y prefiera dividir él.
    constexpr Sum total(void) const { return m_total; }

    // Llena la ventana entera con un valor, para que la media arranque ahí en lugar
    // de subir desde cero.
    void prime(Sample x)
    {
        for (uint8_t k = 0; k < N; k++)
        {
            m_buf[k] = x;
        }
        m_total = (Sum)x * (Sum)N;
        m_next  = 0;
    }

    void reset(void) { prime((Sample)0); }

    private:

    Sample  m_buf[N];
    Sum     m_total;
    uint8_t m_next;
};

#endif  // CONTROLMATH_MOVINGAVERAGE_H
