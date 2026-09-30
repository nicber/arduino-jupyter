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
// con el reposo del sensor en ~2000 cuentas cualquiera sea el divisor. Así la escala
// del canal, el cero y todo lo que viene después no cambian. Acá vive la definición de
// esa cuenta (`UV_PER_COUNT`): es esta clase la que la produce.
//
// **En dieciseisavos de cuenta.** `counts_q4()` no calcula el cociente y lo escala
// después, sino que hace las dos cosas en una división:
//
//   c_q4 = mean0_q4 · scale_q4 / mean1_q4
//
// con las dos medias y la escala en la misma unidad fraccionaria, que se cancela
// sola. No hace falta ningún cociente intermedio, y lo que sale son dieciseisavos de
// cuenta, la misma unidad que MainsNotch lleva adentro. Por qué importa: medido sobre
// capturas del banco, el redondeo a cuenta entera de la salida ponía 1,7 a 2,1 mA RMS
// --el que más pone de toda la cadena, muy por encima de los 0,5 mA de un cociente en
// Q12-- y con la salida en dieciseisavos la cadena entera baja de 1,9..2,9 a
// 0,24..0,65 mA RMS. Un cociente en Q16 no arregla nada y roza el desborde; en Q18
// desborda. `counts()` devuelve lo mismo redondeado a cuentas, para quien no necesite
// la resolución.
//
// Con `div_e4 = 0` no hay divisor y sale A0 contra AVCC, que es lo correcto en una
// placa a 5 V como el UNO. Ése es el único camino en el que hay que saber cuánto vale
// AVCC, porque es el único en el que no se cancela: `AVCC_MV`.
//
// Aritmética pura, así que se prueba en la máquina de escritorio.

#ifndef SENSE_SUPPLYRATIO_H
#define SENSE_SUPPLYRATIO_H

#include <stdint.h>

class SupplyRatio
{
    public:

    // Una cuenta equivalente, en uV en la entrada: 5 * 1024 mV / 4096.
    static const uint16_t UV_PER_COUNT = 1250;

    // Bits fraccionarios de lo que devuelve counts_q4(): dieciseisavos de cuenta. Es
    // la unidad de MainsNotch (FRAC_BITS), así que la cadena no cambia de unidad entre
    // una clase y la otra.
    static const uint8_t FRAC_BITS = 4;

    // El fondo de escala en esa unidad. Un A1 muy chico --el divisor desconectado--
    // daría un cociente enorme: se satura acá en lugar de desbordar, y el canal queda
    // contra el riel, que es lo que bringup() sabe leer.
    static const uint16_t FULL_Q4 = 4095u << FRAC_BITS;

    // AVCC en mV. Sólo interviene sin divisor: con divisor se cancela en el cociente.
    // 5006 es lo que se midió una vez en el UNO del banco, y es una hipótesis sobre esa
    // placa: en el clon a 3,3 V no vale, y por eso el clon necesita el divisor.
    static const uint16_t AVCC_MV = 5006;

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

    // Cuentas equivalentes en dieciseisavos, a partir de las sumas de A0 y A1 y de
    // cuántas conversiones entraron en cada una. Sin divisor, o sin conversiones de
    // A1, la media de A0 llevada a la escala de AVCC.
    //
    // No desborda: mean0_q4 <= 4095·16 = 65520 y m_scale_q4 <= 64000, y
    // 65520 · 64000 + 32760 = 4.193.312.760, por debajo de 2^32.
    uint16_t counts_q4(uint32_t sum0, uint32_t n0, uint32_t sum1, uint32_t n1)
    {
        if (!n0)
        {
            // Una fila sin conversiones de A0 es la cadena del ADC parada, y tiene
            // que verse desde la computadora: dejando `supply` con la lectura vieja,
            // `cur_a1` seguía informando 1760 con el ADC muerto y nada en el
            // diagnóstico lo delataba.
            supply = 0;
            return 0;
        }

        const uint32_t mean0_q4 = media_q4(sum0, n0);

        if (!active() || !n1)
        {
            supply = 0;
            // AVCC/5120 en Q12: lo único que dice cuánto vale una cuenta cuando no
            // hay divisor que lo cancele.
            return cap((mean0_q4 * (uint32_t)AVCC_Q12 + 2048UL) >> 12);
        }

        const uint32_t mean1_q4 = media_q4(sum1, n1);
        supply = (uint16_t)((mean1_q4 + 8) >> FRAC_BITS);
        if (!mean1_q4)
        {
            return FULL_Q4;
        }

        return cap((mean0_q4 * (uint32_t)m_scale_q4 + mean1_q4 / 2) / mean1_q4);
    }

    // Lo mismo redondeado a cuentas enteras, para quien no necesite la resolución.
    int16_t counts(uint32_t sum0, uint32_t n0, uint32_t sum1, uint32_t n1)
    {
        return (int16_t)((counts_q4(sum0, n0, sum1, n1) + 8) >> FRAC_BITS);
    }

    private:

    // La media en dieciseisavos de cuenta. El camino corto multiplica primero, que es
    // una división y no dos, y entra en 32 bits mientras la suma no pase de 2^32/16:
    // 32 filas de 91 conversiones de 4095 dan 190 millones y sobra. Pero `loop_div`
    // llega a 255, y con filas de 255 ticks y la media de 4 ticks la ventana junta
    // decenas de miles de conversiones: ahí la suma sí se pasa, y el camino largo la
    // parte en cociente y resto, que es exacto y no desborda.
    //
    // La guarda descuenta el medio divisor que se suma para redondear, y no sólo el
    // producto: sin eso hay una ventana de 4080 valores de `suma` --justo debajo de
    // 2^32/16-- en la que `suma * 16 + n / 2` da la vuelta y la media sale 0 en lugar
    // de 32896, o sea la corriente publicada a -2048 cuentas de golpe. Se alcanza con
    // `loop_div` de 129 para arriba y la ventana de 32 filas.
    static uint32_t media_q4(uint32_t suma, uint32_t n)
    {
        if (suma <= (0xFFFFFFFFUL - n / 2) / 16UL)
        {
            return (suma * 16UL + n / 2) / n;
        }
        return (suma / n) * 16UL + ((suma % n) * 16UL + n / 2) / n;
    }

    // AVCC_MV / 5120 en Q12. Con 5006 mV da 4005, que erra +50 ppm contra el cociente
    // exacto: tres órdenes de magnitud menos que la incertidumbre del propio 5006.
    static const uint16_t AVCC_Q12 = (uint16_t)(((uint32_t)AVCC_MV * 4096UL + 2560UL) / 5120UL);

    static uint16_t cap(uint32_t v)
    {
        return (uint16_t)(v > (uint32_t)FULL_Q4 ? FULL_Q4 : v);
    }

    uint16_t m_scale_q4;
};

#endif  // SENSE_SUPPLYRATIO_H
