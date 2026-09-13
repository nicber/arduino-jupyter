// El actuador que gobierna un lazo: el puente, más lo que un lazo necesita saber
// de él y la computadora mover sin recompilar.
//
// HBridge pone el comando que se le pide y nada más, que es lo que quiere un banco
// de identificación. La frecuencia del PWM como parámetro, cuántos cuadrantes
// tiene el puente y el signo del motor viven acá, del lado del control.

#ifndef CONTROL_LOOPDRIVE_H
#define CONTROL_LOOPDRIVE_H

#include <stdint.h>

template <class Bridge>
class LoopDrive
{
    public:

    typedef typename Bridge::Command Command;
    typedef typename Bridge::Top     Top;

    static const Command MAX = Bridge::MAX;

    // ---------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección. Quien los mueva tiene
    // que llamar a apply() después; no se vigilan solos.

    Top     top;        // TOP del Timer1: f = F_CPU / (2 * top)
    uint8_t bidir;      // 1: el puente acciona en los dos sentidos
    uint8_t invert;     // 1: un comando positivo hace bajar la magnitud realimentada
    Command u;          // el último comando pedido, antes de `invert`, para la telemetría

    constexpr explicit LoopDrive(Top initial_top)
        : top(initial_top)
        , bidir(1)
        , invert(0)
        , u(0)
        , m_bridge(initial_top)
    {
    }

    void begin(void) { m_bridge.begin(); }

    // El piso del comando, que es lo que hay que informarle a la lógica
    // anti-windup: un puente cableado para un solo cuadrante recorta en cero, y
    // prometerle al integrador un sentido que el hardware no tiene lo deja
    // cargando contra un límite que no existe.
    Command floor(void) const { return bidir ? (Command)-MAX : (Command)0; }

    // Aplica un `top` que la computadora haya movido. Se llama después de cada
    // escritura de cualquier parámetro, y el puente sólo rearranca el temporizador
    // si de verdad cambió: un barrido de ganancia no tiene por qué sacudirlo. Lo
    // que quedó puesto vuelve a `top`, así la computadora ve el piso aplicado.
    void apply(void) { top = m_bridge.set_top(top); }

    // `invert` reconcilia dos convenciones de signo que se fijan con cables: la
    // del motor en las salidas del puente, y la de lo que mire el sensor. Si no
    // coinciden, un lazo de posición realimenta en positivo y se escapa en lugar
    // de establecerse, y se escapa con la referencia de cualquier signo, así que no
    // hay manera de descubrirlo probando. Dar vuelta los dos cables del motor es el
    // arreglo físico y equivale exactamente a esto.
    void write(Command command)
    {
        u = command;
        m_bridge.write(invert ? (Command)-command : command);
    }

    private:

    Bridge m_bridge;
};

#endif  // CONTROL_LOOPDRIVE_H
