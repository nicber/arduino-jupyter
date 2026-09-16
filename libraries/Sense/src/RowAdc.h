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
// El reparto: on_conversion() va en la interrupción del ADC y suma; close_tick() va
// en la del muestreador, en cada tick de 5 kHz, y close_row() en el tick de la fila,
// que cierra la ventana; row() lo lee el lazo cuando quiera. Así la ventana de cada
// fila queda pegada a su tick, igual que el ángulo.
//
// El ruido de la corriente es blanco a ~0,9 mA/√Hz desde la entrada hasta varios kHz,
// y para ruido blanco el promedio de la fila deja la misma densidad en 0-250 Hz que un
// antialiasing ideal. Simulado sobre 11 s de conversiones crudas: con un FIR de 1001
// coeficientes cortando en 200 Hz, el ruido en 55-200 Hz baja de 16,4 a 16,0 mA; dos
// polos por corrimiento suben el piso por debajo de 45 Hz. Lo que sí hay que sacar
// antes de decimar son los armónicos del PWM, que no son ruido: con el PWM a 1250 Hz,
// 4 ticks son un período justo, y `ma` pasa cada tick por una media de los últimos 4,
// con ceros en 1250, 2500, 3750... Hz. La fila de 10 ticks queda así con una ventana
// trapezoidal de 13 (2,6 ms, 0,3 ms más de retardo). Lo que se pliega igual cae en
// 250 Hz, el Nyquist de las filas, y en continua el muestreo del AS5600, 5 kHz, que
// absorbe el cero.
//
// Lo que la media no puede sacar: son unas 2 conversiones de A0 por tick, no un
// promedio continuo, y los armónicos del PWM también se pliegan contra ese peine. Medido
// con el motor en régimen: la corriente media depende de cómo quedaron las conversiones
// contra el PWM, que cambia al arrancar una captura, y salta entre dos niveles. Ver el
// bloque siguiente, que es el mismo problema visto en reposo.
//
// **El salto de dos niveles entre capturas.** Medido en reposo, en tramos de 3 s durante
// tres minutos: la corriente publicada toma uno de dos niveles, el nivel se sortea al
// arrancar cada captura y no se mueve mientras la captura dura --el ruido entre filas es
// de 0,5 mA y la primera mitad contra la segunda difieren 0,3 mA, contra 62 mA entre
// tramos--. No es el divisor (`cur_a1` no se mueve ni el 0,3 %) ni el tráfico de I2C del
// AS5600 (cortándolo el salto sigue: 9,1 cuentas con el bus andando, 7,8 sin él). Lo que
// sí lo mueve:
//
// - Un capacitor de 100 nF de A0 a masa: de 62 a 26 mA.
// - Emitir los cinco canales en lugar de uno: de 26 a 3,6 mA. Una fila más larga sacude
//   la alineación lo suficiente como para que los dos estados se promedien adentro de la
//   captura. Es el modo por omisión de `capture()`, y es la razón por la que el cero hay
//   que medirlo con los mismos canales con los que se va a medir (ver
//   `Bench.zero_current()`).
// - El tiempo exacto de esta interrupción: agregar un par de ciclos acá movió el salto
//   de 26 mA a 1,8 mA, y sacarlos lo devolvió. Es el mismo filo de navaja que el párrafo
//   de la alternancia de canales de más abajo, y quiere decir que el número concreto
//   depende de la compilación.
//
// Atar las conversiones al tick en lugar de dejarlas libres --una ráfaga de N por tick,
// con la fase fijada por construcción-- se probó y es peor, que es el argumento entero a
// favor de dejarlo libre. Medido con el motor, contra el modo libre y descontando el
// reposo de cada configuración: a /32 con ráfagas de 3 el sesgo es de -29, +9 y -26 mA
// con comandos de 80, 150 y 220, y a /16 con ráfagas de 5, de -12, -3 y -6 mA; la
// dispersión entre repeticiones sube de 0,5..3,8 mA a 1,7..17 mA. La ráfaga no llega a
// llenar el tick --a /32 entran 3 conversiones en 200 us y a /16 entran 5-- así que
// siempre queda sin mirar la misma fracción del período del PWM, y eso es exactamente el
// sesgo de fase que el conversor libre evita.
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
// sin capacitor, a /32 y en reposo, con la media de 4 ticks y el notch de 250 Hz
// apagados para ver la ventana sola: el ruido de la corriente es de 2,9 mA por fila y
// 0,37 mA con 10 filas midiendo A0/A1. Los dos son el desvío de la diferencia entre
// filas consecutivas sobre raíz de dos, que es el ruido de banda: el desvío a secas
// trae además la deriva del cero, que no se promedia y que en reposo es más grande
// que todo esto (7 cuentas en un minuto, medidas).
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
// Lo que sale de acá son cuentas contra AVCC, corridas al fondo de escala de 4096 de
// la placa de 12 bits, y nada más: cuánto vale AVCC no se sabe acá y no hace falta.
// Llevarlas a la escala publicada --1,25 mV por cuenta-- es de SupplyRatio, que es
// quien sabe si hay divisor. Con divisor, cualquier factor común a los dos canales se
// cancela en el cociente, así que aplicarlo acá era un no-op que sólo truncaba.

#ifndef SENSE_ROWADC_H
#define SENSE_ROWADC_H

#include <Arduino.h>
#include <stdint.h>
#include <util/atomic.h>

template <uint8_t Channel, uint8_t SupplyChannel>
class RowAdc
{
    public:

    // 1 pasa cada tick por una media de los últimos 4 ticks antes de sumarlo a la fila;
    // 0 suma cada tick tal cual. Público porque la tabla del enlace toma su dirección.
    uint8_t ma;

    constexpr RowAdc()
        : ma(0)
        , m_sum()
        , m_n()
        , m_hist_sum()
        , m_hist_n()
        , m_acc_sum()
        , m_acc_n()
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

    // Llamar desde la ISR del muestreador en cada tick, antes de close_row(). Cierra
    // lo del tick y lo acumula en la fila: tal cual, o como la suma de los últimos 4
    // ticks (una media móvil de 4, con sus conversiones contadas en n, así que el
    // promedio sigue siendo suma sobre n).
    void close_tick(void)
    {
        for (uint8_t k = 0; k < 2; k++)
        {
            const uint32_t s = m_sum[k];
            const uint16_t n = m_n[k];
            m_sum[k] = 0;
            m_n[k]   = 0;

            if (ma)
            {
                m_acc_sum[k] += s + m_hist_sum[k][0] + m_hist_sum[k][1] + m_hist_sum[k][2];
                m_acc_n[k]   += (uint16_t)(n + m_hist_n[k][0] + m_hist_n[k][1] + m_hist_n[k][2]);
            }
            else
            {
                m_acc_sum[k] += s;
                m_acc_n[k]   += n;
            }

            m_hist_sum[k][2] = m_hist_sum[k][1];
            m_hist_sum[k][1] = m_hist_sum[k][0];
            m_hist_sum[k][0] = s;
            m_hist_n[k][2]   = m_hist_n[k][1];
            m_hist_n[k][1]   = m_hist_n[k][0];
            m_hist_n[k][0]   = n;
        }
    }

    // Llamar desde la ISR del muestreador, en el tick de la fila, después de
    // close_tick().
    void close_row(void)
    {
        for (uint8_t k = 0; k < 2; k++)
        {
            m_row_sum[k] = m_acc_sum[k];
            m_row_n[k]   = m_acc_n[k];
            m_acc_sum[k] = 0;
            m_acc_n[k]   = 0;
        }
    }

    // Las sumas de la última fila cerrada, del sensor y de la alimentación, en cuentas
    // contra AVCC y al fondo de escala de 4096, y cuántas conversiones entraron en
    // cada una. Cero conversiones del sensor quiere decir que el ADC no corrió.
    void row(uint32_t& sum, uint16_t& n, uint32_t& supply_sum, uint16_t& supply_n) const
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            sum        = m_row_sum[0];
            n          = m_row_n[0];
            supply_sum = m_row_sum[1];
            supply_n   = m_row_n[1];
        }

        // A 12 bits, que es el fondo de escala en el que hablan las dos placas.
        sum <<= m_shift;
        supply_sum <<= m_shift;
    }

    private:

    uint8_t prescaler(void) const
    {
        return m_lgt ? (uint8_t)(_BV(ADPS2) | _BV(ADPS0))                      // /32
                     : (uint8_t)(_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0));        // /128
    }

    volatile uint32_t m_sum[2];         // lo del tick en curso
    volatile uint16_t m_n[2];
    uint32_t          m_hist_sum[2][3]; // los tres ticks anteriores, para la media de 4
    uint16_t          m_hist_n[2][3];
    uint32_t          m_acc_sum[2];     // lo de la fila en curso
    uint16_t          m_acc_n[2];
    volatile uint32_t m_row_sum[2];
    volatile uint16_t m_row_n[2];
    volatile uint8_t  m_channel;    // de qué canal es la conversión en curso
    volatile bool     m_alternate;
    bool              m_lgt;
    uint8_t           m_shift;
};

#endif  // SENSE_ROWADC_H
