// Un puente en H gobernado por una magnitud modulada y dos entradas de sentido,
// que es el reparto del L298N y el de casi cualquier módulo de puente: ENA lleva
// el ciclo de trabajo y el par IN1/IN2 elige para qué lado.
//
// Toda la modulación queda en un solo pin y el sentido en dos salidas digitales
// comunes, y eso es lo que permite apagar el puente entero con una sola
// escritura. Ver write().
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

    // El piso del TOP son 255. Por debajo de eso el ciclo de trabajo tendría menos
    // escalones que el comando, así que el comando dejaría de ser fiel; y los
    // 31,4 kHz que ese piso significa ya están bastante más arriba de lo que le
    // conviene a un puente de Darlington bipolares. El techo lo pone el propio
    // uint16: 65535 son 122 Hz, lo bastante lento como para ver la ondulación.
    static const Top TOP_MIN = 255;

    // ---------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección. Quien los mueva tiene
    // que llamar a apply() después; no se vigilan solos.

    Top     top;        // TOP del Timer1: f = F_CPU / (2 * top)
    uint8_t bidir;      // 1: el puente acciona en los dos sentidos
    uint8_t invert;     // 1: un comando positivo hace bajar la magnitud realimentada
    Command u;          // el último comando aplicado, para la telemetría

    constexpr HBridge(Top initial_top)
        : top(initial_top)
        , bidir(1)
        , invert(0)
        , u(0)
        , m_dir(0)
        , m_applied(0)
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

        m_applied = top;
    }

    // El piso del comando, que es lo que hay que informarle a la lógica
    // anti-windup: un puente cableado para un solo cuadrante recorta en cero, y
    // prometerle al integrador un sentido que el hardware no tiene lo deja
    // cargando contra un límite que no existe.
    Command floor(void) const { return bidir ? (Command)-MAX : (Command)0; }

    // Aplica un `top` que la computadora haya movido, y sólo si de verdad cambió.
    //
    // ICR1 no está amortiguado en este modo, así que escribirlo con el contador ya
    // pasado del TOP nuevo cuesta un período largo hasta que la cuenta dé la vuelta
    // entera; rearrancar el temporizador desde cero lo evita. Pero eso interrumpe
    // el PWM, y quien llama a esto lo hace después de cada escritura de cualquier
    // parámetro: un barrido de ganancia no tiene por qué sacudir el puente. De ahí
    // que el valor aplicado sea un miembro y no una variable escondida adentro de
    // la función.
    void apply(void)
    {
        if (top < TOP_MIN)
        {
            top = TOP_MIN;
        }

        if (top != m_applied)
        {
            m_applied = top;
            start_timer();
        }
    }

    // Pone `command` sobre el puente: la magnitud en ENA por PWM, el sentido en
    // IN1/IN2.
    //
    // Un cambio de sentido no escribe las entradas de sentido con el puente vivo.
    // Primero baja ENA, que apaga las cuatro llaves de una sola escritura; recién
    // entonces mueve IN1 e IN2, y pasa por el estado con las dos en bajo antes de
    // levantar la que corresponde. El PWM vuelve al final, ya con el sentido nuevo
    // en pie.
    //
    // El tiempo muerto sobra sin escribir un solo delay: off() suelta el pin en el
    // ciclo en que se ejecuta, y el digitalWrite() que sigue se pasa unos 4 us
    // leyendo tablas y deshabilitando interrupciones antes de tocar su propio pin,
    // contra el orden de 1 a 2 us que tarda un L298 en abrir una salida. Pero la
    // protección de verdad no es ese tiempo: cada medio puente del L298 cuelga de
    // una sola entrada lógica y el reparto entre el transistor de arriba y el de
    // abajo es interno, así que desde afuera no hay forma de pedirle a una rama que
    // conduzca por los dos lados. Lo que compra bajar ENA primero es que el cambio
    // de sentido no atraviese ningún estado conduciendo.
    //
    // Lo que ningún tiempo muerto arregla es lo otro que pasa al invertir: la
    // corriente que ya circula por el motor no se puede cortar, así que sale por
    // los diodos del puente contra la fuente. Eso es milisegundos --la constante
    // L/R del motor-- y la respuesta es no pedir saltos de +MAX a -MAX.
    //
    // Un comando en cero deja las entradas de sentido donde estaban en lugar de
    // forzarlas: con ENA en cero el puente ya está abierto y el motor en punto
    // muerto, así que un comando que ronda el cero no golpea IN1/IN2 en cada
    // período.
    void write(Command command)
    {
        u = command;

        // `invert` reconcilia dos convenciones de signo que se fijan con cables: la
        // del motor en las salidas del puente, y la de lo que mire el sensor. Si no
        // coinciden, un lazo de posición realimenta en positivo y se escapa en
        // lugar de establecerse, y se escapa con la referencia de cualquier signo,
        // así que no hay manera de descubrirlo probando. Dar vuelta los dos cables
        // del motor es el arreglo físico y equivale exactamente a esto.
        int8_t sign = (command > 0) ? 1 : ((command < 0) ? -1 : 0);

        if (invert)
        {
            sign = (int8_t)-sign;
        }

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

    private:

    // Timer1, phase-correct con TOP = ICR1 y preescalador 1, de modo que
    // f = F_CPU / (2 * top). El TOP propio es lo que hace que la frecuencia sea un
    // parámetro y no un modo fijo; el precio es que analogWrite() deja de servir
    // sobre este pin, porque da por sentado que el TOP son 255. De ahí duty().
    //
    // Arranca con la salida de comparación desconectada, que es el puente abierto:
    // la conecta duty() cuando hay algo que accionar.
    void start_timer(void)
    {
        TCCR1A = _BV(WGM11);                    // modo 10: phase-correct, TOP = ICR1
        TCCR1B = _BV(WGM13) | _BV(CS10);        // preescalador /1
        TCNT1  = 0;
        ICR1   = top;
    }

    // `magnitude` va de 0 a MAX y el temporizador cuenta hasta `top`, que es otra
    // escala. Se divide por MAX + 1 = 256 en lugar de por 255, que es un
    // corrimiento en vez de una división y deja el ciclo de trabajo a lo sumo un
    // escalón corto; el extremo de arriba, que es el que se notaría --MAX tiene que
    // ser encendido permanente y no 255/256 de él-- se atiende aparte. OCR1A está
    // doblemente amortiguado, así que el valor nuevo entra al terminar el período
    // en curso y ningún pulso sale cortado por la mitad.
    void duty(Command magnitude)
    {
        if (magnitude <= 0)
        {
            off();
            return;
        }

        OCR1A = (magnitude >= MAX) ? top
                                   : (Top)(((uint32_t)magnitude * top) >> 8);

        TCCR1A |= _BV(COM1A1);
    }

    int8_t m_dir;       // el sentido que está cableado ahora: era un static de drive()
    Top    m_applied;   // el TOP que el temporizador tiene puesto de verdad
};

#endif  // ACTUATOR_HBRIDGE_H
