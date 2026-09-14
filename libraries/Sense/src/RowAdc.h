// El ADC corriendo libre sobre un solo canal, acumulando todas sus conversiones, y
// entregando el promedio de las que cayeron en cada fila.
//
// Existe porque la corriente del motor no se puede medir con una conversión por
// fila. El PWM y el muestreador salen del mismo cristal, así que una conversión
// por fila cae siempre en la misma fase del período, y la corriente adentro del
// período es un escalón de cientos de mA: medido en el banco, según la fase, la
// lectura se desvía de la media entre -230 y +315 mA. Promediar unas pocas fases
// fijas tampoco alcanza (±40 mA con cinco). Con el ADC libre, en cambio, las
// conversiones recorren todas las fases --en el LGT8F328P a /32 son 250 fases en
// 11 ms-- y el promedio de la fila da la media sin sesgo, a cambio de un error de
// patrón chico (≤ 13 mA medidos) que se promedia en las filas siguientes.
//
// El reparto: on_conversion() va en la interrupción del ADC y suma; close_row() va
// en la del muestreador, en el tick de la fila, y cierra la ventana; row() lo lee
// el lazo cuando quiera. Así la ventana de cada fila queda pegada a su tick, igual
// que el ángulo.
//
// El preescalador depende de la placa. La hoja de datos del LGT8F328P pide un reloj
// de 300 kHz a 3 MHz: /32 son 500 kHz, y una conversión libre de 22 relojes son
// 44 us. El ATmega328P quiere 50 a 200 kHz para sus 10 bits: /128, 104 us. Medido en
// el clon, la interrupción que sólo acumula se lleva el 16 % de la CPU a /32.
//
// Siempre contra AVCC, que tiene un problema medido: mientras el transistor
// conduce, su corriente de base carga la alimentación del micro y todo lo que se lee
// sube un 2,2 %. Se compensa afuera, con el ciclo de trabajo (SupplySag.h). Lo que se
// probó y no sirvió, para no volver a probarlo:
//
// - El divisor interno del LGT8F328P contra su referencia de 1,024 V: no se corre con
//   la caída de AVCC, pero con el I2C del AS5600 funcionando la lectura se va un 6 %
//   y su ruido se multiplica por 14. Descartando las conversiones que se superponen
//   con una transferencia el ruido baja a la mitad, pero con PWM lee igual +90 a
//   +120 mA sin corriente en el motor: peor que AVCC compensado.
// - La referencia interna de 4,096 V: no queda elegida en esta placa.
// - AREF a los 3,3 V de la placa: sigue a la alimentación del micro, que por el USB
//   está en ~4 V y deja al regulador de 3,3 V sin margen. El corrimiento baja sólo
//   a la mitad.
//
// Las dos placas publican en la misma escala: una cuenta son 5 * 1024 / 4096 =
// 1,25 mV en la entrada, contra un AVCC de 5006 mV medidos en el UNO.

#ifndef SENSE_ROWADC_H
#define SENSE_ROWADC_H

#include <Arduino.h>
#include <stdint.h>
#include <util/atomic.h>

template <uint8_t Channel>
class RowAdc
{
    public:

    // Una cuenta publicada, en uV en la entrada: 5 * 1024 mV / 4096.
    static const uint16_t UV_PER_COUNT = 1250;

    constexpr RowAdc()
        : m_sum(0)
        , m_n(0)
        , m_row_sum(0)
        , m_row_n(0)
        , m_shift(0)
    {
    }

    // `full_scale` es lo que el conversor de esta placa da a fondo de escala: 4096
    // en el clon, 1024 en el UNO.
    void begin(uint16_t full_scale)
    {
        const bool lgt = (full_scale >= 4096);

        m_shift = lgt ? 0 : 2;

        const uint8_t prescaler = lgt
                                ? (uint8_t)(_BV(ADPS2) | _BV(ADPS0))                  // /32
                                : (uint8_t)(_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0));    // /128

        ADCSRA = 0;
        ADMUX  = (uint8_t)(_BV(REFS0) | (Channel & 0x07));
        ADCSRB = (uint8_t)(ADCSRB & ~0x07);                  // libre
        ADCSRA = (uint8_t)(_BV(ADEN) | _BV(ADIE) | _BV(ADATE) | prescaler);
        ADCSRA |= _BV(ADSC);
    }

    // Llamar desde ISR(ADC_vect).
    void on_conversion(void)
    {
        m_sum += ADC;
        m_n++;
    }

    // Llamar desde la ISR del muestreador, en el tick de la fila.
    void close_row(void)
    {
        m_row_sum = m_sum;
        m_row_n   = m_n;
        m_sum     = 0;
        m_n       = 0;
    }

    // La suma de la última fila cerrada, ya en la escala publicada, y cuántas
    // conversiones entraron. Cero conversiones quiere decir que el ADC no corrió.
    void row(uint32_t& sum, uint16_t& n) const
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            sum = m_row_sum;
            n   = m_row_n;
        }

        // A 12 bits, y de 5006 mV a 5120 mV de fondo. 5006/5120 es 1 - 1/45 con cinco
        // decimales, y así no desborda con filas largas.
        sum <<= m_shift;
        sum -= sum / 45;
    }

    private:

    volatile uint32_t m_sum;
    volatile uint16_t m_n;
    volatile uint32_t m_row_sum;
    volatile uint16_t m_row_n;
    uint8_t           m_shift;
};

#endif  // SENSE_ROWADC_H
