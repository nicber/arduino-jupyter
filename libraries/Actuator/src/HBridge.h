// Un puente en H gobernado por una magnitud modulada y dos entradas de sentido,
// que es el reparto del L298N y el de casi cualquier módulo de puente: ENA lleva
// el ciclo de trabajo y el par IN1/IN2 elige para qué lado.
//
// Toda la modulación queda en un solo pin y el sentido en dos salidas digitales
// comunes, y eso es lo que permite apagar el puente entero con una sola
// escritura. Ver write().
//
// Sirve igual para el actuador más pobre, un transistor a masa con su diodo de
// rueda libre gobernado desde ENA: IN1 e IN2 no van a ningún lado, y un comando
// negativo empuja para el mismo lado que uno positivo. Eso no se esconde acá: la
// placa pone lo que se le pide y es la medición la que muestra qué hizo el eje.
//
// El pin de ENA no es una preferencia: este código habla con OC1A del Timer1
// directamente, porque es el único temporizador que queda libre en un AVR de la
// familia del UNO cuando el Timer0 lleva millis() y el Timer2 muestrea. Así que
// Ena tiene que ser el pin de OC1A --el 9 en un UNO-- y mudarlo al 10 es cambiar
// OCR1A por OCR1B y COM1A1 por COM1B1 acá adentro. IN1 e IN2 van a donde sea.
//
// Los pines son parámetros de plantilla y no argumentos, para que digitalWrite()
// los vea como constantes y el compilador no cargue una tabla en tiempo de
// ejecución para resolverlos.

#ifndef ACTUATOR_HBRIDGE_H
#define ACTUATOR_HBRIDGE_H

#include <Arduino.h>
#include <stdint.h>

template <uint8_t Ena, uint8_t In1, uint8_t In2>
class HBridge
{
    public:

    // El comando que entra y el TOP que sale, cada uno con nombre: el primero está
    // en cuentas de -MAX..MAX y el segundo en cuentas del temporizador, y
    // confundirlos es el error que estos dos typedef existen para hacer visible.
    typedef int16_t  Command;
    typedef uint16_t Top;

    // El techo del comando. No es negociable sin tocar duty(): aprovecha que
    // MAX + 1 sea una potencia de dos para escalar con un corrimiento en lugar de
    // una división.
    static const Command MAX = 255;

    Command u;          // el último comando aplicado, para la telemetría

    // `top` es el TOP del Timer1: f = F_CPU / (2 * top). Por debajo de 255 el
    // ciclo de trabajo tendría menos escalones que el comando.
    constexpr explicit HBridge(Top top)
        : u(0)
        , m_top(top < 255 ? 255 : top)
        , m_dir(0)
    {
    }

    // Deja el puente abierto y las dos entradas de sentido en bajo, que es el
    // estado del que parte write().
    //
    // ENA primero: mientras el puente esté abierto las entradas de sentido no
    // gobiernan nada, así que ése es el orden en el que ningún estado intermedio
    // acciona el motor.
    void begin(void)
    {
        start_timer();

        pinMode(Ena, OUTPUT);
        digitalWrite(Ena, LOW);
        pinMode(In1, OUTPUT);
        digitalWrite(In1, LOW);
        pinMode(In2, OUTPUT);
        digitalWrite(In2, LOW);
    }

    // Pone `command` sobre el puente: la magnitud en ENA por PWM, el sentido en
    // IN1/IN2. Se recorta a -MAX..MAX.
    //
    // Un cambio de sentido no escribe las entradas de sentido con el puente vivo.
    // Primero baja ENA, que apaga las cuatro llaves de una sola escritura; recién
    // entonces mueve IN1 e IN2, y pasa por el estado con las dos en bajo antes de
    // levantar la que corresponde. El PWM vuelve al final, ya con el sentido nuevo
    // en pie.
    //
    // Lo que ningún orden de escrituras arregla es lo otro que pasa al invertir: la
    // corriente que ya circula por el motor no se puede cortar, así que sale por
    // los diodos del puente contra la fuente, durante la constante L/R del motor.
    //
    // Un comando en cero deja las entradas de sentido donde estaban en lugar de
    // forzarlas: con ENA en cero el puente ya está abierto y el motor en punto
    // muerto --no frena, sólo deja de empujar--, así que un comando que ronda el
    // cero no golpea IN1/IN2 en cada período.
    void write(Command command)
    {
        if (command >  MAX) command =  MAX;
        if (command < -MAX) command = (Command)-MAX;

        u = command;

        const int8_t sign = (command > 0) ? 1 : ((command < 0) ? -1 : 0);
        const int8_t want = sign ? sign : m_dir;

        if (want != m_dir)
        {
            off();                              // ENA: puente abierto
            digitalWrite(In1, LOW);
            digitalWrite(In2, LOW);
            digitalWrite((want > 0) ? In1 : In2, HIGH);
            m_dir = want;
        }

        duty((command >= 0) ? command : (Command)-command);
    }

    // Desconecta la salida de comparación del pin, que vuelve a ser una salida
    // común con su bit de PORT en bajo desde begin(): ENA queda en bajo y el puente
    // abierto, en el ciclo en el que se pide y no al final del período de PWM.
    void off(void)
    {
        TCCR1A &= (uint8_t)~_BV(COM1A1);
    }

    // La magnitud sola, sin tocar el sentido.
    //
    // El camino normal es write(), que decide las dos cosas juntas y es el único que
    // garantiza que un cambio de sentido no atraviese un estado conduciendo. Esto
    // existe para un diagnóstico que necesita las dos mitades por separado: mover
    // ENA con IN1 e IN2 quietos, y leer cada pin de vuelta. Ver Puente_Bringup.
    //
    // `magnitude` va de 0 a MAX y el temporizador cuenta hasta el TOP, que es otra
    // escala. Se divide por MAX + 1 = 256 en lugar de por 255, que es un
    // corrimiento en vez de una división; el extremo de arriba --MAX tiene que ser
    // encendido permanente y no 255/256 de él-- se atiende aparte. OCR1A está
    // doblemente amortiguado, así que el valor nuevo entra al terminar el período
    // en curso y ningún pulso sale cortado por la mitad.
    void duty(Command magnitude)
    {
        if (magnitude <= 0)
        {
            off();
            return;
        }

        OCR1A = (magnitude >= MAX) ? m_top
                                   : (Top)(((uint32_t)magnitude * m_top) >> 8);

        TCCR1A |= _BV(COM1A1);
    }

    private:

    // Timer1, phase-correct con TOP = ICR1 y preescalador 1, de modo que
    // f = F_CPU / (2 * top). El precio del TOP propio es que analogWrite() deja de
    // servir sobre este pin, porque da por sentado que el TOP son 255. De ahí
    // duty().
    //
    // Arranca con la salida de comparación desconectada, que es el puente abierto:
    // la conecta duty() cuando hay algo que accionar.
    void start_timer(void)
    {
        TCCR1A = _BV(WGM11);                    // modo 10: phase-correct, TOP = ICR1
        TCCR1B = _BV(WGM13) | _BV(CS10);        // preescalador /1
        TCNT1  = 0;
        ICR1   = m_top;
    }

    Top    m_top;
    int8_t m_dir;       // el sentido que está cableado ahora
};

#endif  // ACTUATOR_HBRIDGE_H
