// La corriente medida contra la alimentación del sensor y no contra la del micro.
//
// El ACS712 es ratiométrico: reposa en la mitad de su alimentación y su sensibilidad
// es proporcional a ella. Medido contra AVCC, eso sólo se cancela si el sensor y el
// micro comparten alimentación. En el clon del banco no: el micro funciona a 3,3 V y
// el sensor está en los 5 V del USB. Medido en el banco, contra AVCC la lectura se
// corre con cualquier consumo de la placa --la corriente de base del transistor,
// 1,35 % a fondo; el LED del pin 13, +45 cuentas--, porque lo que se mueve es la
// referencia del ADC y no el sensor.
//
// Así que A1 lee los 5 V del sensor por un divisor resistivo, `div_e4` es la relación
// de ese divisor en diezmilésimas, y la corriente sale del cociente A0/A1, en el que
// las dos alimentaciones se cancelan. Medido con la fuente del motor apagada: la
// caída con el PWM pasa de +0,4..+1,35 % a -0,01 % en promedio, sin tendencia con el
// ciclo de trabajo, y el efecto del LED baja un 88 %.
//
// El resultado sale en "cuentas equivalentes": las que daría un ADC de 1,25 mV por
// cuenta contra exactamente 5,000 V de alimentación del sensor,
//
//   c = (A0 / A1) · 4000 · k,        k = div_e4 / 10000
//
// que es la misma escala que sin divisor (RowAdc::UV_PER_COUNT), con el reposo del sensor en
// ~2000 cuentas cualquiera sea el divisor. Así la escala del canal, el cero y todo lo
// que viene después no cambian. Con `div_e4 = 0` no hay divisor y sale A0 tal cual,
// contra AVCC, que es lo correcto en una placa a 5 V como el UNO.
//
// Aritmética pura, así que se prueba en la máquina de escritorio.

#ifndef SENSE_SUPPLYRATIO_H
#define SENSE_SUPPLYRATIO_H

#include <stdint.h>

class SupplyRatio
{
    public:

    uint16_t div_e4;    // relación del divisor de A1, en diezmilésimas; 0 = sin divisor
    uint16_t supply;    // la última media de A1, en cuentas: una lectura, para verificar

    constexpr SupplyRatio()
        : div_e4(0)
        , supply(0)
        , m_scale_q4(0)
    {
    }

    // Aplicar un `div_e4` que se haya movido. 4000 · k en Q4 es div_e4 · 6,4.
    void apply(void)
    {
        if (div_e4 > 10000) div_e4 = 10000;
        m_scale_q4 = (uint16_t)(((uint32_t)div_e4 * 32UL + 2UL) / 5UL);
    }

    bool active(void) const { return div_e4 != 0; }

    // Cuentas equivalentes a partir de las sumas de A0 y A1 y de cuántas conversiones
    // entraron en cada una. Sin divisor, o sin conversiones de A1, la media de A0.
    int16_t counts(uint32_t sum0, uint32_t n0, uint32_t sum1, uint32_t n1)
    {
        if (!n0)
        {
            return 0;
        }

        // Medias en dieciseisavos de cuenta: una suma de 32 filas de 91 conversiones
        // de 4095 por 16 todavía entra en 32 bits.
        const uint32_t mean0_q4 = (sum0 * 16UL + n0 / 2) / n0;

        if (!active() || !n1)
        {
            supply = 0;
            return (int16_t)((mean0_q4 + 8) >> 4);
        }

        const uint32_t mean1_q4 = (sum1 * 16UL + n1 / 2) / n1;
        supply = (uint16_t)((mean1_q4 + 8) >> 4);
        if (!mean1_q4)
        {
            return INT16_MAX;
        }

        // A0/A1 en Q12, y de ahí a cuentas equivalentes. Un A1 muy chico --el divisor
        // desconectado-- da un cociente enorme: se satura en lugar de desbordar.
        const uint32_t ratio_q12 = ((mean0_q4 << 12) + mean1_q4 / 2) / mean1_q4;
        if (ratio_q12 > 60000UL)
        {
            return INT16_MAX;
        }

        const uint32_t c = (ratio_q12 * m_scale_q4 + 32768UL) >> 16;
        return (int16_t)(c > 32767UL ? 32767UL : c);
    }

    private:

    uint16_t m_scale_q4;
};

#endif  // SENSE_SUPPLYRATIO_H
