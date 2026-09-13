// La referencia de un lazo: a dónde se le pide que vaya, y cuánto le falta.
//
// Lleva bits fraccionarios debajo del valor entero, y eso es lo único que hace que
// una rampa pueda avanzar menos de una cuenta por período sin que la cuantización la
// anule. Sin esa fracción, una pendiente lenta se redondea a cero y el eje no se
// mueve nunca.
//
// No sabe sobre qué magnitud cierra el lazo. Recibe la medición por error(), así que
// el mismo objeto sirve para un ángulo en cuentas y para una corriente en cuentas de
// conversor, y quién decide cuál es asunto de quien lo use.

#ifndef CONTROL_SETPOINT_H
#define CONTROL_SETPOINT_H

#include <stdint.h>

class Setpoint
{
    public:

    // La referencia se guarda corrida `FRAC` bits a la izquierda; la medición y el
    // error salen en unidades enteras. Los dos typedef existen para que no se
    // confundan: sumarle una medición a la referencia sin correrla es un error de 256
    // veces.
    typedef int32_t Fixed;      // unidades del objetivo << FRAC
    typedef int32_t Value;      // unidades del objetivo, enteras

    static const uint8_t FRAC = 8;

    // --------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección.

    Fixed   ref;        // la referencia
    Fixed   rate;       // cuánto avanza por período de control
    int16_t uff;        // comando prealimentado, o de lazo abierto
    uint8_t target;     // sobre qué magnitud cierra el lazo
    uint8_t mode;       // qué controlador gobierna
    int16_t e;          // el error del último período, recortado para la telemetría

    constexpr Setpoint(uint8_t initial_mode, uint8_t initial_target)
        : ref(0)
        , rate(0)
        , uff(0)
        , target(initial_target)
        , mode(initial_mode)
        , e(0)
    {
    }

    // Un período de rampa. Es lo único que distingue una rampa de un escalón, así que
    // un modo de rampa es el controlador de siempre más esta línea.
    void advance(void) { ref += rate; }

    // Cuánto le falta a la medición para llegar a la referencia.
    //
    // Publica de paso el error recortado, así que un controlador que ignore el valor
    // devuelto igual deja `e` vivo para que la computadora lo mire.
    Value error(Value measured)
    {
        const Value diff = (Value)(ref >> FRAC) - measured;

        e = clamp(diff);
        return diff;
    }

    // La referencia en unidades enteras, para quien la quiera mirar sin la fracción.
    Value value(void) const { return (Value)(ref >> FRAC); }

    private:

    static int16_t clamp(Value v)
    {
        if (v < INT16_MIN) return INT16_MIN;
        if (v > INT16_MAX) return INT16_MAX;
        return (int16_t)v;
    }
};

#endif  // CONTROL_SETPOINT_H
