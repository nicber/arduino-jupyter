// Notch para la red: 50 Hz y sus primeros armónicos, sobre la corriente fila por
// fila.
//
// Medido en el banco, en reposo y sin motor: la corriente trae tonos de 67 mA a
// 50 Hz, 44 mA a 100 Hz y 7,5 mA a 150 Hz, que se meten por la alimentación y la
// masa del USB. La ventana de 20 ms de WindowMean los anula, pero atrasa ~10 ms y
// redondea el pico de un arranque. Con una ventana corta --para mirar un
// transitorio-- los tonos quedan, y esto los saca.
//
// Una sección por armónico, cada una un biquad
//
//   H(z) = g · (1 - 2 cos w0 z^-1 + z^-2) / (1 - 2 r cos w0 z^-1 + r^2 z^-2)
//
// con ceros sobre la circunferencia en w0 y polos a radio r: cuanto más cerca de 1,
// más angosto el notch y más largo el transitorio. g deja la ganancia en continua en
// 1. La red se ve desde la placa en ~49,7 Hz y no en 50: el cristal del clon adelanta
// un 0,6 % --las filas salen a 503 Hz y no a 500--, y la red además se mueve unas
// decenas de mHz. La calibra la computadora con una captura sin carga: ver
// Bench.calibrar_red(). Medido en el banco con r = 0,95 y la red calibrada: los tonos
// bajan de 83/48/10 mA a 0,5/0,1/0,1 mA (con el centro 0,1 Hz corrido quedan 2 mA),
// un escalón asienta en ~80 ms con 15 % de sobrepico, y el ruido blanco no baja --eso
// lo hace la ventana--.
//
// Aritmética: coeficientes en Q12 y señal en cuartos de cuenta, los dos en 16 bits,
// así que cada producto es de 16 x 16 bits, que es lo barato en un AVR. Lo que la
// división por 2^12 no guarda se arrastra a la muestra siguiente, así que la salida
// no se queda trabada a unas cuentas del valor. Los coeficientes se calculan con
// punto flotante en apply(), que corre sólo cuando la computadora mueve algo.
//
// Aritmética pura, salvo apply(), así que se prueba en la máquina de escritorio.

#ifndef SENSE_MAINSNOTCH_H
#define SENSE_MAINSNOTCH_H

#include <math.h>
#include <stdint.h>

class MainsNotch
{
    public:

    static const uint8_t MAX_HARMONICS = 3;

    // --------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección. Mover y llamar a apply().

    uint16_t mains_chz;     // la red vista desde la placa, en centésimas de Hz
    uint8_t  harmonics;     // cuántas secciones: 0 apaga, 1 = 50, 2 = 50+100, 3 = +150
    uint16_t pole_milli;    // r, en milésimas

    constexpr MainsNotch()
        : mains_chz(4980)
        , harmonics(0)
        , pole_milli(950)
        , m_active(0)
        , m_fresh(true)
        , m_sec()
    {
    }

    // Recalcula los coeficientes para filas de `row_hz`. Una sección cuyo armónico
    // queda por encima de la mitad de la frecuencia de las filas no se usa.
    void apply(float row_hz)
    {
        if (harmonics > MAX_HARMONICS) harmonics = MAX_HARMONICS;
        if (pole_milli > 999)          pole_milli = 999;

        const float r = pole_milli / 1000.0f;
        m_active = 0;

        for (uint8_t k = 0; k < harmonics; k++)
        {
            const float f0 = mains_chz / 100.0f * (k + 1);
            if (f0 <= 0.0f || f0 >= row_hz / 2.0f)
            {
                break;
            }

            const float c  = cos(2.0f * M_PI * f0 / row_hz);
            const float a1 = -2.0f * r * c;
            const float a2 = r * r;
            const float g  = (1.0f + a1 + a2) / (2.0f - 2.0f * c);   // continua en 1

            Section& s = m_sec[m_active++];
            s.b0 = q12(g);
            s.b1 = q12(-2.0f * c * g);
            s.a1 = q12(a1);
            s.a2 = q12(a2);
        }

        m_fresh = true;
    }

    // Una fila, en cuentas. Sin secciones activas devuelve la entrada tal cual.
    int16_t step(int16_t counts)
    {
        if (!m_active)
        {
            return counts;
        }

        int16_t x = sat((int32_t)counts * 4);

        // Al prender o cambiar algo, el estado arranca en la entrada: sin eso cada
        // cambio de parámetro sería un escalón desde cero, con su transitorio.
        if (m_fresh)
        {
            for (uint8_t k = 0; k < m_active; k++)
            {
                Section& s = m_sec[k];
                s.x1 = s.x2 = s.y1 = s.y2 = x;
                s.carry = 0;
            }
            m_fresh = false;
        }

        for (uint8_t k = 0; k < m_active; k++)
        {
            Section& s = m_sec[k];

            const int32_t acc = (int32_t)s.b0 * x
                              + (int32_t)s.b1 * s.x1
                              + (int32_t)s.b0 * s.x2
                              - (int32_t)s.a1 * s.y1
                              - (int32_t)s.a2 * s.y2
                              + s.carry;

            // División hacia abajo por 2^12, con el resto guardado.
            int32_t y = acc >> 12;
            s.carry = (int16_t)(acc - (y << 12));

            const int16_t out = sat(y);
            s.x2 = s.x1;
            s.x1 = x;
            s.y2 = s.y1;
            s.y1 = out;
            x = out;
        }

        // De cuartos de cuenta a cuentas, redondeando.
        return (int16_t)((x + 2) >> 2);
    }

    private:

    struct Section
    {
        int16_t b0, b1, a1, a2;     // Q12; b2 = b0
        int16_t x1, x2, y1, y2;     // cuartos de cuenta
        int16_t carry;
    };

    static int16_t q12(float v)
    {
        return (int16_t)lround(v * 4096.0f);
    }

    static int16_t sat(int32_t v)
    {
        if (v > INT16_MAX) return INT16_MAX;
        if (v < INT16_MIN) return INT16_MIN;
        return (int16_t)v;
    }

    uint8_t m_active;
    bool    m_fresh;
    Section m_sec[MAX_HARMONICS];
};

#endif  // SENSE_MAINSNOTCH_H
