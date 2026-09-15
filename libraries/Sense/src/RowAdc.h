// El ADC corriendo libre, acumulando todas sus conversiones, y entregando la suma de
// las que cayeron en cada fila. Sobre un canal, o alternando entre el del sensor y el
// de su alimentación (ver SupplyRatio.h).
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
// Alternando canales hay que saber de cuál es cada conversión. Corriendo libre, la
// conversión siguiente arranca apenas termina una, antes de que corra la
// interrupción, así que el canal que se escribe en la interrupción rige para la
// conversión después de la que ya empezó: se lleva el canal de las dos.
//
// El preescalador depende de la placa y del modo. La hoja de datos del LGT8F328P pide
// un reloj de 300 kHz a 3 MHz: /32 son 500 kHz, y una conversión libre de 22 relojes
// son 44 us; medido en el clon, la interrupción que sólo acumula se lleva el 16 % de
// la CPU. Alternando con A1, en cambio, va a /64 (88 us): medido en el banco con el
// divisor de A1 sin capacitor, el ruido de la corriente con 10 filas es de 31 mA a
// /32, de 9 mA a /64 y de 120 mA a /128 --contra 5 mA midiendo sólo A0 a /32--, y a
// /64 siguen cayendo unas 11 conversiones por período de PWM. El ATmega328P quiere
// 50 a 200 kHz para sus 10 bits: /128, 104 us, en los dos modos.
//
// Siempre contra AVCC. Lo que se probó para la referencia y no sirvió, para no
// volver a probarlo:
//
// - El divisor interno del LGT8F328P: con el I2C del AS5600 funcionando la lectura
//   se va un 6 % y su ruido se multiplica por 10, contra AVCC o contra la referencia
//   interna. Lo que se mueve es el divisor, no la referencia.
// - La referencia interna de 4,096 V: con el micro a 3,3 V no existe.
// - AREF con un divisor desde los 5 V: el LGT8F328P sostiene el pin AREF con su
//   referencia de ~1,06 V a través de unos 400 ohm en todos los modos que se
//   probaron, y el divisor no lo mueve más que hasta 1,6 V; la referencia que usa
//   el ADC en ese modo tampoco es la tensión del pin. De ahí el divisor en A1.
//
// Las dos placas publican en la misma escala: una cuenta son 5 * 1024 / 4096 =
// 1,25 mV en la entrada, contra un AVCC de 5006 mV medidos en el UNO. Sin divisor esa
// escala sólo vale en una placa a 5 V; con divisor la da SupplyRatio en cualquiera.

#ifndef SENSE_ROWADC_H
#define SENSE_ROWADC_H

#include <Arduino.h>
#include <stdint.h>
#include <util/atomic.h>

template <uint8_t Channel, uint8_t SupplyChannel>
class RowAdc
{
    public:

    // Una cuenta publicada, en uV en la entrada: 5 * 1024 mV / 4096.
    static const uint16_t UV_PER_COUNT = 1250;

    constexpr RowAdc()
        : m_sum()
        , m_n()
        , m_row_sum()
        , m_row_n()
        , m_done(0)
        , m_running(0)
        , m_alternate(false)
        , m_lgt(false)
        , m_shift(0)
    {
    }

    // `full_scale` es lo que el conversor de esta placa da a fondo de escala: 4096
    // en el clon, 1024 en el UNO.
    void begin(uint16_t full_scale)
    {
        const bool lgt = (full_scale >= 4096);

        m_shift = lgt ? 0 : 2;
        m_lgt   = lgt;

        const uint8_t prescaler = this->prescaler();

        ADCSRA = 0;
        ADMUX  = (uint8_t)(_BV(REFS0) | (Channel & 0x1F));
        ADCSRB = (uint8_t)(ADCSRB & ~0x07);                  // libre
        m_done = m_running = 0;
        ADCSRA = (uint8_t)(_BV(ADEN) | _BV(ADIE) | _BV(ADATE) | prescaler);
        ADCSRA |= _BV(ADSC);
    }

    // Alternar con el canal de la alimentación, o quedarse en el del sensor.
    void alternate(bool on)
    {
        if (on == m_alternate)
        {
            return;
        }
        m_alternate = on;
        ADCSRA = (uint8_t)((ADCSRA & ~0x07) | prescaler());
    }

    // Llamar desde ISR(ADC_vect).
    void on_conversion(void)
    {
        const uint8_t k = m_done;
        m_sum[k] += ADC;
        m_n[k]++;

        // La que terminó ahora era la que corría; la que ya arrancó pasa a ser la
        // próxima en terminar; y lo que se escribe ahora rige para la siguiente.
        m_done    = m_running;
        m_running = (m_alternate && !m_running) ? 1 : 0;
        ADMUX = (uint8_t)((ADMUX & ~0x1F) | ((m_running ? SupplyChannel : Channel) & 0x1F));
    }

    // Llamar desde la ISR del muestreador, en el tick de la fila.
    void close_row(void)
    {
        for (uint8_t k = 0; k < 2; k++)
        {
            m_row_sum[k] = m_sum[k];
            m_row_n[k]   = m_n[k];
            m_sum[k]     = 0;
            m_n[k]       = 0;
        }
    }

    // Las sumas de la última fila cerrada, del sensor y de la alimentación, ya en la
    // escala publicada, y cuántas conversiones entraron en cada una. Cero
    // conversiones del sensor quiere decir que el ADC no corrió.
    void row(uint32_t& sum, uint16_t& n, uint32_t& supply_sum, uint16_t& supply_n) const
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            sum        = m_row_sum[0];
            n          = m_row_n[0];
            supply_sum = m_row_sum[1];
            supply_n   = m_row_n[1];
        }

        // A 12 bits, y de 5006 mV a 5120 mV de fondo. 5006/5120 es 1 - 1/45 con cinco
        // decimales, y así no desborda con filas largas.
        sum <<= m_shift;
        sum -= sum / 45;
        supply_sum <<= m_shift;
        supply_sum -= supply_sum / 45;
    }

    private:

    uint8_t prescaler(void) const
    {
        if (!m_lgt)
        {
            return (uint8_t)(_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0));            // /128
        }
        return m_alternate ? (uint8_t)(_BV(ADPS2) | _BV(ADPS1))                // /64
                           : (uint8_t)(_BV(ADPS2) | _BV(ADPS0));               // /32
    }

    volatile uint32_t m_sum[2];
    volatile uint16_t m_n[2];
    volatile uint32_t m_row_sum[2];
    volatile uint16_t m_row_n[2];
    volatile uint8_t  m_done;       // de qué canal es la conversión que termina ahora
    volatile uint8_t  m_running;    // de qué canal es la que ya arrancó
    volatile bool     m_alternate;
    bool              m_lgt;
    uint8_t           m_shift;
};

#endif  // SENSE_ROWADC_H
