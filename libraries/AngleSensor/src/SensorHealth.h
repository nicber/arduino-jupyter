// Lo que hay que saber sobre un sensor para creerle lo que dice: si contesta, si
// el bus lo sigue, y qué opina él mismo de su montaje.
//
// Recibe números por update() y no le pregunta nada a nadie, así que no sabe qué
// sensor es ni por qué bus habla. Lo que sí sabe son las dos formas que tiene un
// dato de salud, que no se tratan igual:
//
//   Las cuentas son totales acumulados. Los contadores de un sensor corren libres
//   y no se pueden borrar, así que lo que se publica es el total de sus
//   incrementos: eso es lo que hace que la computadora pueda poner uno en cero
//   igual que cualquier otro parámetro, en lugar de que se lo pisen en el período
//   siguiente.
//
//   Los estados son lecturas de ahora. Ponerlos en cero sería inventar una
//   lectura, así que no se acumulan ni se limpian.

#ifndef ANGLESENSOR_SENSORHEALTH_H
#define ANGLESENSOR_SENSORHEALTH_H

#include <Arduino.h>
#include <stdint.h>

class SensorHealth
{
    public:

    // --------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección.

    uint16_t overruns;  // muestras que el bus no llegó a seguir
    uint16_t errors;    // transferencias que fallaron
    uint8_t  present;   // el sensor contesta en el bus
    uint8_t  status;    // el registro de estado del sensor, tal cual
    uint8_t  agc;       // la ganancia con la que lee: contra un extremo, mal montado
    uint16_t magnitude; // el módulo del vector de campo

    constexpr SensorHealth()
        : overruns(0)
        , errors(0)
        , present(1)
        , status(0)
        , agc(0)
        , magnitude(0)
        , m_last_overruns(0)
        , m_last_errors(0)
        , m_turn(0)
        , m_last_ms(0)
        , m_complete(false)
    {
    }

    // Los contadores libres del sensor -> los totales publicados. La diferencia
    // contra la lectura anterior es lo que se acumula, y las dos lecturas anteriores
    // son miembros y no variables escondidas adentro de la función.
    void accumulate(uint16_t sensor_overruns, uint16_t sensor_errors, bool responds)
    {
        overruns += (uint16_t)(sensor_overruns - m_last_overruns);
        errors   += (uint16_t)(sensor_errors   - m_last_errors);

        m_last_overruns = sensor_overruns;
        m_last_errors   = sensor_errors;

        // A diferencia de los contadores, esto es un estado y no una cuenta: dice si
        // el sensor está contestando ahora, no cuántas veces falló. Es lo que
        // distingue un imán mal montado --el sensor contesta y se queja del imán--
        // de un sensor que directamente no está en el bus.
        present = responds ? 1 : 0;
    }

    // Sin sensor en el bus no hay nada que informar del montaje, y dejar el último
    // valor sería peor que no decir nada.
    void forget_mounting(void)
    {
        status    = 0;
        agc       = 0;
        magnitude = 0;
    }

    // Si toca refrescar el diagnóstico de montaje.
    //
    // Los registros se leen por turnos, así que el diagnóstico entero tarda varios
    // refrescos en llenarse. Al ritmo lento eso son un par de segundos de arranque
    // en los que la ganancia todavía vale cero, y quien pregunte enseguida lee un
    // cero y lo informa como un imán contra el borde. Así que la primera vuelta va
    // rápido y recién después se afloja al ritmo de algo que se mira entre corridas.
    bool due(uint32_t now_ms)
    {
        const uint16_t wait = m_complete ? SLOW_MS : FAST_MS;

        if ((uint32_t)(now_ms - m_last_ms) < wait)
        {
            return false;
        }

        m_last_ms = now_ms;
        return true;
    }

    // Qué registro toca leer, de a uno por refresco.
    //
    // De a uno y no todos juntos porque una lectura larga no entra en un período de
    // muestreo, así que el tick siguiente encuentra el bus ocupado y se cuenta un
    // desborde. Medido en un banco: 4,5 desbordes por segundo. Lo caro no es la
    // muestra perdida, es que la puesta en marcha informaba una falla de bus en un
    // equipo sano, y una verificación que grita en falso enseña a ignorarla.
    uint8_t turn(void) const { return m_turn; }

    // Cierra un refresco y pasa el turno. Cuando la ronda se completa, el ritmo se
    // afloja.
    void advance(uint8_t turns)
    {
        m_turn = (uint8_t)((m_turn + 1) % turns);

        if (m_turn == 0)
        {
            m_complete = true;
        }
    }

    private:

    static const uint16_t FAST_MS = 50;    // la primera vuelta, para que no se lea un cero
    static const uint16_t SLOW_MS = 500;   // después, el ritmo de algo que se mira entre corridas

    uint16_t m_last_overruns;   // eran dos static de collect_sensor_health()
    uint16_t m_last_errors;

    uint8_t  m_turn;            // eran tres static de refresh_magnet_status()
    uint32_t m_last_ms;
    bool     m_complete;
};

#endif  // ANGLESENSOR_SENSORHEALTH_H
