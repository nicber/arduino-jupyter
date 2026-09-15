// El promedio de todas las conversiones de las últimas `rows` filas.
//
// Es el filtro de la corriente, y vive en la placa a propósito: la computadora
// recibe una fila cada 2 ms, y lo que hay que sacar --el rizado de ~1 kHz del PWM y
// los 50 y 100 Hz que se meten desde la red, de decenas de mA en reposo (ver
// MainsNotch.h)-- no se puede sacar de filas que ya vienen con eso plegado. Un
// promedio de caja de 20 ms tiene ceros exactos en 50, 100, 150 Hz... y en cada armónico del PWM, y
// medido en el banco deja 5 a 10 mA de ruido en lugar de 15 a 70 por fila.
//
// Lo que se paga es retardo: una caja de N filas atrasa (N - 1)/2 filas más la
// media fila de la ventana propia, o sea 10 ms para 10 filas a 500 Hz. Ese retardo
// es conocido y fijo, y se puede meter en el modelo; con `rows = 1` no hay filtro y
// cada fila es el promedio de sus propias conversiones.
//
// Promedia conversiones y no promedios de fila: una fila con menos conversiones
// pesa menos. Aritmética pura, así que se prueba en la máquina de escritorio.

#ifndef SENSE_WINDOWMEAN_H
#define SENSE_WINDOWMEAN_H

#include <stdint.h>

class WindowMean
{
    public:

    static const uint8_t MAX_ROWS = 32;

    uint8_t rows;       // filas en la ventana, 1..MAX_ROWS; mover y llamar a apply()
    int16_t mean;       // el último promedio, en cuentas, redondeado

    constexpr explicit WindowMean(uint8_t initial_rows)
        : rows(initial_rows)
        , mean(0)
        , m_sums()
        , m_counts()
        , m_total(0)
        , m_count(0)
        , m_next(0)
        , m_size(initial_rows)
    {
    }

    // Aplica un `rows` que se haya movido: lo recorta y empieza la ventana de
    // nuevo, porque las filas viejas ya no suman lo mismo.
    void apply(void)
    {
        if (rows < 1)        rows = 1;
        if (rows > MAX_ROWS) rows = MAX_ROWS;

        if (rows != m_size)
        {
            m_size = rows;
            for (uint8_t k = 0; k < MAX_ROWS; k++)
            {
                m_sums[k]   = 0;
                m_counts[k] = 0;
            }
            m_total = 0;
            m_count = 0;
            m_next  = 0;
        }
    }

    // Una fila: la suma de sus conversiones y cuántas fueron. Devuelve el promedio.
    int16_t push(uint32_t sum, uint16_t n)
    {
        m_total += sum - m_sums[m_next];
        m_count += (uint32_t)n - m_counts[m_next];
        m_sums[m_next]   = sum;
        m_counts[m_next] = n;
        m_next = (uint8_t)((m_next + 1) % m_size);

        if (m_count)
        {
            mean = (int16_t)((m_total + m_count / 2) / m_count);
        }
        return mean;
    }

    // La suma de las conversiones de la ventana y cuántas son: para quien necesite
    // más resolución que el promedio redondeado (SupplyRatio).
    uint32_t total(void) const { return m_total; }
    uint32_t count(void) const { return m_count; }

    private:

    uint32_t m_sums[MAX_ROWS];
    uint16_t m_counts[MAX_ROWS];
    uint32_t m_total;
    uint32_t m_count;
    uint8_t  m_next;
    uint8_t  m_size;
};

#endif  // SENSE_WINDOWMEAN_H
