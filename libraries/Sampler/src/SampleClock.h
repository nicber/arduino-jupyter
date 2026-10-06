// El reloj de un lazo de control: un temporizador de período rígido que muestrea,
// y un divisor que decide cada cuántas muestras corre la ley de control.
//
// Ese reparto es el punto de todo el módulo. El muestreo mantiene su período
// aunque el cálculo de control fluctúe, y lo que fluctúa queda medido en lugar de
// perderse: `late` dice cuánto tardó el lazo en atender un tick y `missed` cuenta
// los que nunca atendió.
//
// `late` se mide desde el PRIMER tick sin atender, no desde el último. Medido en el
// banco: con `loop_div = 1` y cinco canales se pierde el 92 % de los períodos y el
// retardo real es de 1,8 ms; medido desde el último tick, eso da 268 us --el 134 % de
// un período--, un número que no puede expresar un lazo desbordado y que se lee como
// margen de tiempo. Cuesta cero ciclos medirlo bien: es no pisar la marca de tiempo
// cuando ya hay un período sin atender.
//
// No sabe nada de qué se muestrea. Quien lo use pone en la ISR lo que haya que
// muestrear y le pregunta a on_isr() si además vence un período de control.
//
// Se queda con el Timer2, así que analogWrite() en los pines 3 y 11 y tone()
// dejan de funcionar. El Timer0 queda para millis() y micros(), y este módulo usa
// micros() para `late`. Con board::millis_1000hz() micros() corre un 2,4 % rápido y
// salta 28 us en cada ms, así que `late` es aproximado en esa medida.

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

    // Los dos contadores de salud saturan en lugar de dar la vuelta: un número chico
    // y tranquilizador después de haber dado la vuelta es peor que un tope. `missed`
    // llega a 65535 en 11,9 s de captura con `loop_div = 1`, y `late` da la vuelta
    // a los 65,5 ms. Leer el tope significa «por lo menos esto», no «esto».
    static const Micros   LATE_MAX   = 0xFFFF;
    static const uint16_t MISSED_MAX = 0xFFFF;

    // --------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección.

    Divider  divide;     // muestras por período de control
    Micros   late;       // peor retardo entre el primer tick sin atender y su atención
    uint16_t missed;     // períodos que el lazo nunca atendió

    constexpr SampleClock(Divider initial_divide)
        : divide(initial_divide)
        , late(0)
        , missed(0)
        , m_due(false)
        , m_hold(false)
        , m_streaming(false)
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
            // perdiendo del todo, que es peor que una simple fluctuación. La marca de
            // tiempo NO se pisa, así que sigue siendo la del primer tick sin atender
            // y `late` mide el retardo de verdad y no uno acotado a un período.
            if (m_missed_isr != MISSED_MAX)
            {
                m_missed_isr++;
            }
        }
        else
        {
            m_fired_us = micros();
        }

        m_due = true;
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

        // Fuera de una ventana de emisión los contadores no se tocan: la computadora
        // los lee varios comandos después del `stop`, y lo que pase entre medio --una
        // lectura de mantenimiento que se destapa justo cuando el flujo para-- no
        // pertenece a la ventana. Medido: sin esto, dos corridas idénticas informan
        // 32 us y 9872 us de `late` según cuánto tarde el `get`.
        if (m_hold)
        {
            return true;
        }

        missed = (missed > (uint16_t)(MISSED_MAX - lost)) ? MISSED_MAX
                                                          : (uint16_t)(missed + lost);

        // En 32 bits y saturando: con el lazo desbordado el retardo pasa de los
        // 65,5 ms que entran en un uint16, y dar la vuelta ahí informaría un retardo
        // chico justo cuando es enorme.
        const uint32_t d = micros() - fired;
        const Micros delay = (Micros)(d > (uint32_t)LATE_MAX ? LATE_MAX : d);
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

        // Con un divisor nuevo el contador arranca de cero: si no, el primer período
        // después de mover `loop_div` dura cualquier cosa entre 1 y el divisor viejo,
        // o sea una fila con marca de tiempo mentirosa. Y sólo entonces: esto corre
        // con CADA escritura de parámetro --el `set` de un escalón incluido--, y
        // reiniciar la cuenta sin motivo estiraría justo el período en el que cae.
        //
        // Los dos juntos y con la ISR afuera: entre uno y otro, la ISR vería el
        // divisor nuevo con la cuenta vieja y podría cerrar un período corto.
        if (divide != m_divider)
        {
            noInterrupts();
            m_divider = divide;
            m_count   = 0;
            interrupts();
        }

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

        // La marca de tiempo también, y no sólo los contadores: como on_isr() no la
        // pisa mientras haya un período sin atender, un tick sin atender de ANTES de
        // la ventana la dejaría apuntando al pasado, y el primer take() de la captura
        // le cargaría los milisegundos de puerto serie que costó arrancarla. Que es lo
        // que esta función existe para descartar. Medido en la placa: sin esto, una
        // captura por omisión sana informa 12,7 ms de retardo con cero períodos
        // perdidos, que es una contradicción.
        m_fired_us = micros();
        interrupts();

        missed = 0;
        late   = 0;
        m_hold = false;
    }

    // Abre y cierra la ventana de medición al ritmo de la emisión. Se llama en cada
    // pasada de loop() con CtrlLink::streaming(), después de CtrlLink::poll(), que
    // es donde se atiende `start` y se imprime el encabezado: así la ventana empieza
    // a contar recién cuando ya salió.
    //
    // Al abrirla, clear_health(). Al cerrarla, `late` y `missed` se congelan en lo
    // que describieron la ventana: la computadora los lee varios comandos después
    // del `stop`, y lo que pase entre medio no describe la captura.
    void window(bool streaming)
    {
        if (streaming && !m_streaming)
        {
            clear_health();
        }
        else if (!streaming && m_streaming)
        {
            m_hold = true;
        }

        m_streaming = streaming;
    }

    // Para una pausa acotada en la que nadie puede pedirle el bus al sensor. Se usa
    // para escribirle un registro: encolar una escritura reserva memoria, y el
    // muestreador llama al bus desde una ISR.
    void pause(void)  { TIMSK2 &= (uint8_t)~_BV(OCIE2A); }
    void resume(void) { TIMSK2 |=  _BV(OCIE2A); }

    private:

    volatile bool     m_due;
    bool              m_hold;     // fuera de una ventana de emisión: no acumular salud
    bool              m_streaming;  // lo que window() vio la última vez
    volatile uint32_t m_fired_us;
    volatile uint16_t m_missed_isr;
    volatile Divider  m_divider;

    uint8_t m_count;      // muestras desde el último período de control
    bool    m_running;
};

#endif  // SAMPLER_SAMPLECLOCK_H
