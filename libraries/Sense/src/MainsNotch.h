// Notch para la red: 50 Hz y sus primeros armónicos, sobre la corriente fila por
// fila, sin tener que medir la frecuencia de la red. Y otro en el Nyquist de las filas,
// para lo que el PWM deja ahí; ver `nyquist`.
//
// Medido en el banco, en reposo y sin motor: la corriente trae tonos de 67 mA a
// 50 Hz, 44 mA a 100 Hz y 7,5 mA a 150 Hz, que se meten por la alimentación y la
// masa del USB. La ventana de 20 ms de WindowMean los anula, pero atrasa ~10 ms y
// redondea el pico de un arranque. Con una ventana corta --para mirar un
// transitorio-- los tonos quedan, y esto los saca.
//
// La red no cae justo en 50 Hz vista desde la placa: el clon no tiene cristal y su
// reloj RC interno adelanta (ver BoardStart/BoardClock.h). Con las filas medidas a
// 503 Hz y no a 500, un 0,6 %, la red se ve en ~49,7 Hz, y además se mueve unas
// decenas de mHz. Un notch angosto en 50 Hz deja pasar buena parte de
// eso. En lugar de medir la red y sintonizar el notch, cada armónico k tiene dos
// notch en serie, en k · 49,5 y k · 50,5 Hz: entre los dos cubren la banda de
// ±0,5 Hz alrededor de 50 (±k · 0,5 alrededor de cada armónico) con una atenuación
// que varía poco adentro de ella.
//
// Cada notch es un biquad
//
//   H(z) = g · (1 - 2 cos w0 z^-1 + z^-2) / (1 - 2 r cos w0 z^-1 + r^2 z^-2)
//
// con ceros sobre la circunferencia en w0 y polos a radio r: cuanto más cerca de 1,
// más angosto y más largo el transitorio. g deja la ganancia en continua en 1.
//
// Aritmética: coeficientes en Q12 y señal en dieciseisavos de cuenta, los dos en 16
// bits, así que cada producto es de 16 x 16 bits, que es lo barato en un AVR. Lo que
// la división por 2^12 no guarda se arrastra a la muestra siguiente, así que la
// salida no se queda trabada a unas cuentas del valor. Los polos cerca de la
// circunferencia amplifican el redondeo de cada sección: en cuartos de cuenta eso le
// sumaba a la corriente unos 3,5 mA RMS de ruido, medido en el banco con la ventana de
// 10 filas; en dieciseisavos queda por debajo del redondeo a cuentas de la salida. El
// precio es el rango: la entrada va de -2047 a 2047 cuentas, que es la corriente
// alrededor de su cero (el fondo del ACS712 de 5 A son ~735). Los coeficientes se
// calculan con punto flotante en apply(), que corre sólo cuando la computadora mueve
// algo.
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
    static const uint8_t MAX_SECTIONS  = 2 * MAX_HARMONICS + 1;   // + el del Nyquist

    // Los dos notch de cada armónico, en centésimas de Hz.
    static const uint16_t LOW_CHZ  = 4950;
    static const uint16_t HIGH_CHZ = 5050;

    // La señal adentro del filtro, en fracciones de cuenta: 2^FRAC_BITS por cuenta.
    static const uint8_t FRAC_BITS = 4;

    // --------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección. Mover y llamar a apply().

    // Qué armónicos, como máscara: 1 = 50 Hz, 2 = 100 Hz, 4 = 150 Hz, cada uno con dos
    // notch; 7 los tres, 0 ninguno.
    uint8_t  harmonics;
    uint16_t pole_milli;    // r, en milésimas

    // 1 agrega un notch en el Nyquist de las filas: 250 Hz con filas a 500 Hz, donde cae
    // lo que queda del PWM de 1250 Hz. Medido con una fila y el motor en régimen, ese tono
    // baja de 1 a 2 mA a 0,1 a 0,2 mA. No es un notch como los de la red: con los polos
    // en 250 Hz y r = 0,95 el redondeo arrastrado a la muestra siguiente queda debajo de
    // los polos y se amplifica unas 4000 veces, y en el banco la banda de 235 a 250 Hz
    // subía siete veces. Acá los ceros son dobles en z = -1 y los polos van en 0,9 del
    // Nyquist (225 Hz a 500 Hz) con radio NYQ_POLE: sin pico, 0,98 en 150 Hz, 0,83 en
    // 200 Hz, -3 dB en 210 Hz, 0,36 ms de retardo en continua y el redondeo amplificado
    // unas 12 veces. Con otro `loop_div` sigue en el Nyquist, aunque el PWM ya no caiga
    // ahí.
    uint8_t nyquist;

    static constexpr float NYQ_POLE  = 0.70f;
    static constexpr float NYQ_ANGLE = 0.90f;   // fracción del Nyquist

    constexpr MainsNotch()
        : harmonics(0)
        , pole_milli(950)
        , nyquist(0)
        , m_active(0)
        , m_fresh(true)
        , m_applied_harmonics(0xFF)
        , m_applied_milli(0)
        , m_applied_nyquist(0)
        , m_applied_row_hz(0.0f)
        , m_sec()
    {
    }

    // Recalcula los coeficientes para filas de `row_hz`. Un notch que queda por
    // encima de la mitad de la frecuencia de las filas no se usa.
    //
    // Sólo si algo de lo suyo cambió: el sketch lo llama después de cada escritura
    // de parámetro, `ctl_uff` incluido, y recalcular reinicia el estado --que en
    // medio de un escalón deja una oscilación de la amplitud de la red-- y cuesta
    // varias operaciones de punto flotante.
    void apply(float row_hz)
    {
        harmonics &= (uint8_t)((1 << MAX_HARMONICS) - 1);
        if (pole_milli > 999)          pole_milli = 999;

        if (harmonics == m_applied_harmonics && pole_milli == m_applied_milli &&
            nyquist == m_applied_nyquist && row_hz == m_applied_row_hz)
        {
            return;
        }
        m_applied_harmonics = harmonics;
        m_applied_milli     = pole_milli;
        m_applied_nyquist   = nyquist;
        m_applied_row_hz    = row_hz;

        const float r = pole_milli / 1000.0f;
        m_active = 0;

        for (uint8_t k = 1; k <= MAX_HARMONICS; k++)
        {
            if (!(harmonics & (1 << (k - 1))))
            {
                continue;
            }
            for (uint8_t lado = 0; lado < 2; lado++)
            {
                const float f0 = (lado ? HIGH_CHZ : LOW_CHZ) / 100.0f * k;
                if (f0 >= row_hz / 2.0f)
                {
                    continue;
                }
                add(f0, row_hz, r);
            }
        }

        if (nyquist)
        {
            add_nyquist();
        }

        m_fresh = true;
    }

    // Cuántos notch quedaron activos: dos por armónico, menos los que no entran.
    uint8_t active(void) const { return m_active; }

    // Una fila, en cuentas alrededor del cero. Sin secciones activas devuelve la
    // entrada tal cual.
    int16_t step(int16_t counts)
    {
        if (!m_active)
        {
            return counts;
        }

        int16_t x = sat((int32_t)counts * (1L << FRAC_BITS));

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

        // De fracciones de cuenta a cuentas, redondeando.
        return (int16_t)((x + (1 << (FRAC_BITS - 1))) >> FRAC_BITS);
    }

    private:

    void add(float f0, float row_hz, float r)
    {
        const float c  = cos(2.0f * M_PI * f0 / row_hz);
        const float a1 = -2.0f * r * c;
        const float a2 = r * r;
        const float g  = (1.0f + a1 + a2) / (2.0f - 2.0f * c);   // continua en 1

        // b1 sale de los otros ya redondeados y no de -2 cos w0 g: así la continua
        // pasa con ganancia exactamente 1 también en Q12, y el cero se corre del
        // orden de 0,02 Hz.
        Section& s = m_sec[m_active++];
        s.b0 = q12(g);
        s.a1 = q12(a1);
        s.a2 = q12(a2);
        s.b1 = (int16_t)(4096 + s.a1 + s.a2 - 2 * s.b0);
    }

    // Ceros dobles exactos en z = -1: b = b0 (1, 2, 1). a2 se corre a lo sumo 2 LSB
    // para que 4096 + a1 + a2 sea múltiplo de 4, y así b0 = (4096 + a1 + a2) / 4 deja la
    // continua exactamente en 1.
    void add_nyquist(void)
    {
        const float c = cos(M_PI * NYQ_ANGLE);
        Section& s = m_sec[m_active++];
        s.a1 = q12(-2.0f * NYQ_POLE * c);
        s.a2 = q12(NYQ_POLE * NYQ_POLE);
        int16_t resto = (int16_t)((4096 + s.a1 + s.a2) % 4);
        s.a2 = (int16_t)(s.a2 - (resto > 2 ? resto - 4 : resto));
        s.b0 = (int16_t)((4096 + s.a1 + s.a2) / 4);
        s.b1 = (int16_t)(2 * s.b0);
    }

    struct Section
    {
        int16_t b0, b1, a1, a2;     // Q12; b2 = b0
        int16_t x1, x2, y1, y2;     // fracciones de cuenta, ver FRAC_BITS
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

    uint8_t  m_active;
    bool     m_fresh;
    uint8_t  m_applied_harmonics;   // con qué se calcularon los coeficientes de ahora
    uint16_t m_applied_milli;
    uint8_t  m_applied_nyquist;
    float    m_applied_row_hz;
    Section  m_sec[MAX_SECTIONS];
};

#endif  // SENSE_MAINSNOTCH_H
