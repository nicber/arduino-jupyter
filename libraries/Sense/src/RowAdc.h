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
// conversiones no caen siempre en la misma y el promedio de la fila da la media sin
// sesgo, a cambio de un error de patrón chico (≤ 13 mA medidos) que se promedia en las
// filas siguientes. Que no quede sesgo está medido y no supuesto: ver el bloque de la
// paridad, donde se compara contra atar las conversiones al tick, que sí sesga.
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
// Lo que la media no puede sacar: son una conversión y media de A0 por tick --tres en
// total, alternando-- y no un promedio continuo, así que los armónicos del PWM también se
// pliegan contra ese peine. Medido con el motor en régimen: la corriente media dependía de
// cómo quedaban las conversiones contra el PWM, que cambia al arrancar una captura, y
// saltaba entre dos niveles. Lo resuelve el relleno del bloque siguiente, que es el mismo
// problema visto en reposo.
//
// **La paridad de las conversiones, y por qué hay un relleno de 3 us.** Alternando dos
// canales, la secuencia de conversiones tiene período 2, y la interrupción del
// muestreador le roba tiempo a una conversión de cada tick. Si en un tick entra un número
// PAR de conversiones, la paridad se conserva de un tick al siguiente y esa perturbación
// cae siempre sobre el MISMO canal; cuál de los dos, se sortea al arrancar el flujo. El
// resultado es que la corriente publicada toma uno de dos niveles, constante mientras la
// captura dura y distinto en la captura siguiente. Con un número impar la paridad se da
// vuelta en cada tick y el efecto se cancela solo.
//
// Está medido y no es teoría. Barriendo el TOP del Timer2 --o sea el ritmo del tick-- con
// un solo binario, para que la alineación del código no cambie entre puntos:
//
//   conversiones por tick   3,000   3,41   3,70   3,96   3,99   4,000
//   salto entre capturas     0-4     2      1      2      5      41..55 mA
//
// Y el control cruzado, en el punto peor y con el mismo binario: moviendo el conteo de
// 4,000 a 3,974 --0,026 de diferencia-- el salto pasa de 48 mA a 1,5 mA, y volviendo a
// 4,000 vuelve a 48. Lo que descarta las otras explicaciones: no es el divisor (`cur_a1`
// no se mueve ni el 0,3 % y no correlaciona con el nivel) ni el tráfico de I2C del AS5600
// (cortando las transferencias el salto sigue), y necesita la alternancia (sin divisor,
// con un canal solo, no aparece).
//
// De ahí el relleno de `SETTLE_US`: no deja entrar una cuarta conversión, así que el
// conteo se queda en 3 y nunca cae en un par. Medido con el firmware final, en reposo y
// con un solo canal emitiendo, que es el caso sensible: el rango entre veinte capturas
// pasa de 24,7 mA a 1,1 mA, sin dos niveles, y con los cinco canales de 3,6 a 1,4 mA.
// El ruido entre filas no cambia (0,53 mA) y no se pierde ningún período.
//
// **Esto vale para el clon a /32, no para un UNO.** `SETTLE_US = 3` está elegido para
// que a /32 --32 us por conversión-- el tick cierre con 3 y no con 4. En un UNO el
// preescalador es /128 y la conversión son 104 us: entran 1,92 conversiones por tick, y
// el relleno las lleva a 1,87. Eso no impide que un tick cierre con 2, que es par, o sea
// que alternando canales en un UNO el salto de dos niveles volvería. Un UNO funciona a
// 5 V y no necesita el divisor (`cur_div = 0` y sin alternancia), así que hoy no se
// alcanza; quien ponga el divisor en un UNO tiene que volver a elegir `SETTLE_US` con
// las perillas de SENSE_DIAG, y el valor será otro.
//
// Las dos alternativas se probaron y son peores:
//
// - **Romper la alternancia**, eligiendo el canal con un bit pseudoaleatorio, mata el
//   enganche --en el punto peor, de 55 a 2,2 mA-- pero convierte el sesgo en ruido
//   blanco: el ruido entre filas sube de 0,55 a 2,6 mA, cinco veces.
// - **Atar las conversiones al tick**, una ráfaga de N por tick con la fase fijada por
//   construcción, reintroduce el sesgo de fase del PWM, que es el argumento entero a
//   favor de dejar el conversor libre. Medido con el motor, contra el modo libre y
//   descontando el reposo de cada configuración: a /32 con ráfagas de 3 el sesgo es de
//   -29, +9 y -26 mA con comandos de 80, 150 y 220, y a /16 con ráfagas de 5, de -12, -3
//   y -6 mA; la dispersión entre repeticiones sube de 0,5..3,8 mA a 1,7..17 mA. La ráfaga
//   no llega a llenar el tick, así que queda sin mirar siempre la misma fracción del
//   período del PWM. El relleno, en cambio, no toca la fase: la cadena sigue corriendo
//   continua entre ticks, sólo que más lenta, y el sesgo medido contra el modo de antes
//   es de +3,1, -0,8 y +2,7 mA, sin tendencia con el ciclo de trabajo.
//
// Un capacitor de 100 nF de A0 a masa baja el salto de 62 a 26 mA por su cuenta, y está
// puesto en este banco, pero no lo elimina: lo que lo elimina es el relleno.
//
// Alternando canales hay que saber de cuál es cada conversión, y corriendo libre no se
// sabe: la conversión siguiente arranca apenas termina una, y en el LGT8F328P un
// cambio de canal escrito en la interrupción alcanza a la que ya arrancó o a la
// siguiente según cuánto tarde la interrupción. Medido en el banco: moviendo la
// escritura unos µs dentro de la interrupción, A1 pasa de leer 1774 a leer 2455, una
// mezcla con A0. Así que el conversor no corre libre: cada interrupción lee, escribe
// el canal de la próxima y recién ahí la arranca. El período queda un poco menos
// parejo, lo que para promediar no importa, y cada conversión es del canal que dice.
//
// El preescalador depende de la placa. La hoja de datos del LGT8F328P pide un reloj
// de 300 kHz a 3 MHz: /32 son 500 kHz, y una conversión de 22 relojes son 44 us más
// lo que tarde la interrupción en arrancar la siguiente. El ATmega328P quiere 50 a
// 200 kHz para sus 10 bits: /128, 104 us. Medido en el clon con el divisor de A1
// sin capacitor, a /32 y en reposo, con la media de 4 ticks y el notch de 250 Hz
// apagados para ver la ventana sola: el ruido de la corriente es de 3,1 mA por fila y
// 0,39 mA con 10 filas midiendo A0/A1. Los dos son el desvío de la diferencia entre
// filas consecutivas sobre raíz de dos, que es el ruido de banda: el desvío a secas
// trae además la deriva del cero, que no se promedia, y que con el relleno de SETTLE_US
// puesto es de 0,11 mA por minuto.
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
#include <util/delay.h>

// Las perillas con las que se midió la paridad de las conversiones --el ritmo del
// conversor, el largo de la ráfaga, el orden de los canales y las medias crudas de cada
// canal-- viven acá pero se compilan afuera. Con `-DSENSE_DIAG=1` aparecen, y con ellas
// se vuelve a elegir `SETTLE_US` si cambia el trabajo de alguna de las dos
// interrupciones. El binario por omisión es el mismo que sin este bloque, verificado
// comparando el .hex.
//
// Se compila agregando la bandera a las propiedades de compilación de `python/placa.py`:
//
//   compiler.cpp.extra_flags=-O2 -DSENSE_DIAG=1
//
// y ahí aparecen, en la tabla del enlace, `dbg_settle` (el relleno, para barrerlo),
// `dbg_lock` (una ráfaga de N conversiones por tick), `dbg_pre` (el preescalador),
// `dbg_mix` (el canal elegido por un LFSR), `dbg_ocr` (el TOP del Timer2, que mueve el
// ritmo del tick sin tocar esta interrupción) y `dbg_i2c` (corta las transferencias del
// AS5600); y los canales `a0`, `a1` y `conv`, que son las medias crudas de cada canal y
// cuántas conversiones entraron en la fila. Cuestan 874 bytes de flash y 15 de RAM.
#ifndef SENSE_DIAG
#define SENSE_DIAG 0
#endif

template <uint8_t Channel, uint8_t SupplyChannel>
class RowAdc
{
    public:

    // Microsegundos de espera entre escribir el canal y arrancar la conversión, y sólo
    // cuando se alterna. No es tiempo de asentamiento: es lo que impide que entre una
    // cuarta conversión en el tick, y está ahí para que el conteo por tick no caiga
    // nunca en un número PAR. Ver el bloque de la paridad en la cabecera de este
    // archivo, y elegirlo de nuevo si cambia el trabajo que hace alguna de las dos
    // interrupciones: las perillas con las que se midió se compilan con
    // `-DSENSE_DIAG=1` (ver SENSE_DIAG, arriba).
    //
    // Medido en el clon, barriendo el ritmo del tick de 4,35 a 5,56 kHz --veinte veces
    // más de lo que se corre el reloj RC de esta placa-- con los cinco canales emitiendo:
    //
    //   relleno   conteo por tick, de 5,56 a 4,35 kHz          períodos perdidos
    //     0 us    3,00 3,00 3,70 3,99 4,00 4,00                    0
    //     2 us    3,00 3,00 3,00 3,75 4,00 4,00                    0
    //     3 us    2,61 3,00 3,00 3,08 3,03 3,65                    0
    //     4 us    2,41 3,00 3,00 3,00 3,03 3,47                  128
    //     6 us    2,28 2,73 3,00 3,00 3,01 3,02                  508
    //
    // Con 2 o menos el conteo todavía llega a 4 en los ritmos lentos; con 4 o más el
    // muestreador empieza a perder períodos. Tres es el único que cumple las dos cosas.
    static const uint8_t SETTLE_US = 3;

    // 1 pasa cada tick por una media de los últimos 4 ticks antes de sumarlo a la fila;
    // 0 suma cada tick tal cual. Público porque la tabla del enlace toma su dirección.
    uint8_t ma;

#if SENSE_DIAG
    uint8_t settle_us;  // reemplaza a SETTLE_US, para barrerlo
    uint8_t lock_n;     // N > 0: una ráfaga de N conversiones por tick, con la fase fija
    uint8_t pre;        // preescalador del ADC (3 = /8, 4 = /16, 5 = /32), en caliente
    uint8_t mix;        // 1: el canal lo elige un LFSR, así la secuencia no tiene período 2
#endif

    constexpr RowAdc()
        : ma(0)
#if SENSE_DIAG
        , settle_us(SETTLE_US)
        , lock_n(0)
        , pre(0)
        , mix(0)
        , m_burst(0)
        , m_lfsr(0xACE1)
#endif
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

    // Reaplicar el preescalador sin reiniciar el ADC entero, y levantar la cadena si
    // quedó parada.
    //
    // Se compila SIEMPRE, no sólo con SENSE_DIAG. on_conversion() rearranca la cadena
    // en cada conversión, pero si la cadena se corta por cualquier motivo no queda
    // quién la vuelva a arrancar, y esto era lo único que lo hacía. Medido con ADEN
    // bajado y subido sin ADSC: la corriente publicada se fue a +13648 mA y se quedó
    // ahí a través de tres capturas, sin que nada del diagnóstico lo delatara.
    // Cuesta ~10 ciclos por escritura de parámetro, que ocurre entre corridas.
    void reconfigure(void)
    {
        const uint8_t p = prescaler();
        if ((ADCSRA & 0x07) != p)
        {
            ADCSRA = (uint8_t)(_BV(ADEN) | _BV(ADIE) | p);
        }
        if (!(ADCSRA & _BV(ADSC)))
        {
#if SENSE_DIAG
            m_burst = 0;
#endif
            m_channel = 0;
            ADMUX = (uint8_t)((ADMUX & ~0x1F) | (Channel & 0x1F));
            ADCSRA |= _BV(ADSC);
        }
    }

#if SENSE_DIAG
    // Las medias crudas de la última fila, en dieciseisavos de cuenta, y cuántas
    // conversiones entraron: para mirar cada canal por separado.
    void raw_means(uint16_t& a0, uint16_t& a1, uint16_t& conv) const
    {
        uint32_t s0, s1;
        uint16_t n0, n1;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            s0 = m_row_sum[0]; n0 = m_row_n[0];
            s1 = m_row_sum[1]; n1 = m_row_n[1];
        }
        a0 = n0 ? (uint16_t)((s0 * 16UL + n0 / 2) / n0) : 0;
        a1 = n1 ? (uint16_t)((s1 * 16UL + n1 / 2) / n1) : 0;
        conv = (uint16_t)(n0 + n1);
    }
#endif

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

#if SENSE_DIAG
        // Con la ráfaga atada al tick no se encadena más allá de las N conversiones; el
        // tick siguiente arranca la próxima.
        if (lock_n && ++m_burst >= lock_n)
        {
            m_sum[k] += v;
            m_n[k]++;
            return;
        }
#endif

        // La próxima: su canal primero, y recién después arrancarla.
#if SENSE_DIAG
        if (mix && m_alternate)
        {
            m_lfsr = (uint16_t)((m_lfsr >> 1) ^ (uint16_t)(-(int16_t)(m_lfsr & 1) & 0xB400));
            m_channel = (uint8_t)(m_lfsr & 1);
        }
        else
        {
            m_channel = (m_alternate && !k) ? 1 : 0;
        }
#else
        m_channel = (m_alternate && !k) ? 1 : 0;
#endif
        ADMUX = (uint8_t)((ADMUX & ~0x1F) | ((m_channel ? SupplyChannel : Channel) & 0x1F));
        if (m_alternate)
        {
#if SENSE_DIAG
            for (uint8_t w = 0; w < settle_us; w++) { _delay_us(1); }
#else
            _delay_us(SETTLE_US);   // que no entre una cuarta conversión; ver SETTLE_US
#endif
        }
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
#if SENSE_DIAG
        if (lock_n && !(ADCSRA & _BV(ADSC)))
        {
            m_burst   = 0;
            m_channel = 0;
            ADMUX = (uint8_t)((ADMUX & ~0x1F) | (Channel & 0x1F));
            ADCSRA |= _BV(ADSC);
        }
#endif

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
#if SENSE_DIAG
        if (pre)
        {
            return (uint8_t)(pre & 0x07);
        }
#endif
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
#if SENSE_DIAG
    volatile uint8_t  m_burst;
    volatile uint16_t m_lfsr;
#endif
    bool              m_lgt;
    uint8_t           m_shift;
};

#endif  // SENSE_ROWADC_H
