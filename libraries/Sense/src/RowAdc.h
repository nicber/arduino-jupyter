// El ADC corriendo libre, acumulando todas sus conversiones, y entregando la suma de
// las que cayeron en cada fila. Sobre un canal, o alternando entre el del sensor y el
// de su alimentación (ver SupplyRatio.h).
//
// Existe porque la corriente del motor no se puede medir con una conversión por
// fila. El PWM y el muestreador salen del mismo reloj, así que una conversión
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
// Sin filtro antes de sumar la fila, a propósito. El ruido de la corriente es blanco a
// ~0,9 mA/√Hz desde la entrada hasta varios kHz, y para ruido blanco el promedio de la
// fila deja la misma densidad en 0-250 Hz que un antialiasing ideal. Simulado sobre
// 11 s de conversiones crudas: con un FIR de 1001 coeficientes cortando en 200 Hz, el
// ruido en 55-200 Hz baja de 16,4 a 16,0 mA; dos polos por corrimiento suben el piso
// por debajo de 45 Hz. Los tonos que sí se pliegan caen en ceros: el PWM de 1050 Hz en
// 50 Hz y sus armónicos, donde anulan la ventana de 20 ms y el notch, y el muestreo
// del AS5600, 5 kHz, en continua, que absorbe el cero.
//
// Alternando canales hay que saber de cuál es cada conversión, y corriendo libre no se
// sabe: la conversión siguiente arranca apenas termina una, y en el LGT8F328P un
// cambio de canal escrito en la interrupción alcanza a la que ya arrancó o a la
// siguiente según cuánto tarde la interrupción. Medido en el banco: moviendo la
// escritura unos µs dentro de la interrupción, A1 pasa de leer 1735 a leer 2400, una
// mezcla con A0. Así que el conversor no corre libre: cada interrupción lee, escribe
// el canal de la próxima y recién ahí la arranca. El período queda un poco menos
// parejo, lo que para promediar no importa, y cada conversión es del canal que dice.
//
// El preescalador depende de la placa. La hoja de datos del LGT8F328P pide un reloj
// de 300 kHz a 3 MHz: /32 son 500 kHz, y una conversión de 22 relojes son 44 us más
// lo que tarde la interrupción en arrancar la siguiente. El ATmega328P quiere 50 a
// 200 kHz para sus 10 bits: /128, 104 us. Medido en el clon con el divisor de A1
// sin capacitor, a /32: el ruido de la corriente es de 17 mA por fila y 6 mA con 10
// filas midiendo A0/A1, y de 21 y 5 mA midiendo sólo A0 contra AVCC.
//
// Siempre contra AVCC. Las alternativas para la referencia se descartan por lo
// siguiente:
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
        , m_channel(0)
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
        ADCSRB = (uint8_t)(ADCSRB & ~0x07);
        m_channel = 0;
        ADCSRA = (uint8_t)(_BV(ADEN) | _BV(ADIE) | prescaler);     // sin ADATE: a mano
        ADCSRA |= _BV(ADSC);
    }

    // Alternar con el canal de la alimentación, o quedarse en el del sensor.
    void alternate(bool on)
    {
        m_alternate = on;
    }

    // Llamar desde ISR(ADC_vect).
    void on_conversion(void)
    {
        const uint8_t k = m_channel;
        const uint16_t v = ADC;

        // La próxima: su canal primero, y recién después arrancarla.
        m_channel = (m_alternate && !k) ? 1 : 0;
        ADMUX = (uint8_t)((ADMUX & ~0x1F) | ((m_channel ? SupplyChannel : Channel) & 0x1F));
        ADCSRA |= _BV(ADSC);

        m_sum[k] += v;
        m_n[k]++;
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

        // A 12 bits, y de 5006 mV a 5120 mV de fondo. 5006/5120 es 1 - 1/45 con cuatro
        // decimales, y así no desborda con filas largas.
        sum <<= m_shift;
        sum -= sum / 45;
        supply_sum <<= m_shift;
        supply_sum -= supply_sum / 45;
    }

    private:

    uint8_t prescaler(void) const
    {
        return m_lgt ? (uint8_t)(_BV(ADPS2) | _BV(ADPS0))                      // /32
                     : (uint8_t)(_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0));        // /128
    }

    volatile uint32_t m_sum[2];
    volatile uint16_t m_n[2];
    volatile uint32_t m_row_sum[2];
    volatile uint16_t m_row_n[2];
    volatile uint8_t  m_channel;    // de qué canal es la conversión en curso
    volatile bool     m_alternate;
    bool              m_lgt;
    uint8_t           m_shift;
};

#endif  // SENSE_ROWADC_H
