// El reloj de un lazo de control: un temporizador de período rígido que muestrea,
// y un divisor que decide cada cuántas muestras corre la ley de control.
//
// Ese reparto es el punto de todo el módulo. El muestreo mantiene su período
// aunque el cálculo de control fluctúe, y lo que fluctúa queda medido en lugar de
// perderse: `late` dice cuánto tardó el lazo en atender un tick y `missed` cuenta
// los que nunca atendió.
//
// No sabe nada de qué se muestrea. Quien lo use pone en la ISR lo que haya que
// muestrear y le pregunta a on_isr() si además vence un período de control.
//
// Se queda con el Timer2, así que analogWrite() en los pines 3 y 11 y tone()
// dejan de funcionar. El Timer0 queda intacto: millis() y micros() andan como
// siempre, y este módulo los necesita.

#ifndef SAMPLER_SAMPLECLOCK_H
#define SAMPLER_SAMPLECLOCK_H

#include <Arduino.h>
#include <stdint.h>

class SampleClock
{
    public:

    // Cuántas muestras entran en un período de control. Con nombre porque es lo que
    // viaja hasta la computadora y lo que la computadora convierte en hertz: se
    // guarda el divisor y no la frecuencia porque es por lo que cuenta el hardware,
    // y la conversión la hace el lado que tiene la aritmética para hacerla.
    typedef uint8_t  Divider;
    typedef uint16_t Micros;     // un retardo de atención, en us

    // --------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección.

    Divider  divide;     // muestras por período de control
    Micros   late;       // peor retardo observado entre el tick y su atención
    uint16_t missed;     // períodos que el lazo nunca atendió

    constexpr SampleClock(Divider initial_divide)
        : divide(initial_divide)
        , late(0)
        , missed(0)
        , m_due(false)
        , m_fired_us(0)
        , m_missed_isr(0)
        , m_divider(initial_divide)
        , m_count(0)
        , m_running(false)
    {
    }

    // Arranca el muestreo a `hz`. Sólo cierran exactamente las frecuencias que el
    // preescalador y un TOP de 8 bits puedan dar; con F_CPU de 16 MHz, /32 y un TOP
    // de 99 son exactamente 5 kHz.
    //
    // Timer2 en CTC con TOP = OCR2A.
    void begin(uint16_t hz)
    {
        const uint32_t ticks = (uint32_t)F_CPU / 32UL / (uint32_t)hz;

        TCCR2A = _BV(WGM21);                    // CTC, TOP = OCR2A
        TCCR2B = _BV(CS21) | _BV(CS20);         // preescalador /32
        OCR2A  = (uint8_t)(ticks - 1);
        TCNT2  = 0;
        TIMSK2 = _BV(OCIE2A);

        m_running = true;
    }

    // Si el muestreo ya está corriendo. Lo consulta quien quiera hacerle al sensor
    // una lectura de mantenimiento que viaje en un tick: antes del primer tick no
    // hay quién la lleve.
    bool running(void) const { return m_running; }

    // Llamar desde la ISR del temporizador, una vez por muestra. Devuelve true
    // cuando además vence un período de control.
    //
    // El contador de muestras es un miembro y no una variable escondida adentro de
    // la función, así que se puede leer, poner en cero y razonar sobre él desde
    // afuera.
    bool on_isr(void)
    {
        if (++m_count < m_divider)
        {
            return false;
        }
        m_count = 0;

        if (m_due)
        {
            // El lazo no atendió el tick anterior: el período de control se está
            // perdiendo del todo, que es peor que una simple fluctuación.
            m_missed_isr++;
        }

        m_fired_us = micros();
        m_due       = true;
        return true;
    }

    // Levanta un período de control vencido, si hay uno. Llamar desde el lazo
    // principal. Devuelve false cuando no hay nada que hacer.
    //
    // Actualiza `late` y `missed` de paso: los acumula en copias comunes en lugar de
    // dejarlos en los contadores de la ISR, para que la computadora pueda ponerlos
    // en cero sin competir con ella.
    bool take(void)
    {
        if (!m_due)
        {
            return false;
        }

        uint32_t fired;
        uint16_t lost;

        // La ISR puede caer entre las dos mitades de una lectura de 32 bits.
        noInterrupts();
        fired        = m_fired_us;
        lost         = m_missed_isr;
        m_missed_isr = 0;
        m_due        = false;
        interrupts();

        missed += lost;

        const Micros delay = (Micros)((uint16_t)micros() - (uint16_t)fired);
        if (delay > late)
        {
            late = delay;
        }

        return true;
    }

    // Aplica un `divide` que la computadora haya movido. El divisor que usa la ISR
    // y el período que se le informa a la computadora cambian juntos, así que
    // ninguno de los dos puede quedar describiendo una frecuencia a la que el lazo
    // no esté corriendo: por eso esto devuelve el período resultante en lugar de
    // dejar que quien llama lo recalcule por su cuenta.
    uint32_t apply(uint16_t hz)
    {
        if (divide == 0)
        {
            divide = 1;
        }

        m_divider = divide;
        return (uint32_t)divide * 1000000UL / (uint32_t)hz;
    }

    // Pone en cero lo que describe una ventana de medición. Se llama al abrir una:
    // arrancar una captura cuesta unos milisegundos de puerto serie, y los períodos
    // que se pierden ahí son el precio de arrancarla y no una falla del lazo.
    //
    // Hay que limpiar también el contador de la ISR, que todavía guarda los ticks
    // perdidos durante ese bloqueo y los sumaría en la pasada siguiente.
    void clear_health(void)
    {
        noInterrupts();
        m_missed_isr = 0;
        interrupts();

        missed = 0;
        late   = 0;
    }

    // Para una pausa acotada en la que nadie puede pedirle el bus al sensor. Se usa
    // para escribirle un registro: encolar una escritura reserva memoria, y el
    // muestreador llama al bus desde una ISR.
    void pause(void)  { TIMSK2 &= (uint8_t)~_BV(OCIE2A); }
    void resume(void) { TIMSK2 |=  _BV(OCIE2A); }

    private:

    volatile bool     m_due;
    volatile uint32_t m_fired_us;
    volatile uint16_t m_missed_isr;
    volatile Divider  m_divider;

    uint8_t m_count;      // muestras desde el último período: era un static de la ISR
    bool    m_running;
};

#endif  // SAMPLER_SAMPLECLOCK_H
