// Lo que hay que saber sobre un sensor para validar sus lecturas: si contesta, si
// el bus lo sigue, y qué informa él mismo de su montaje.
//
// No sabe por qué bus habla el sensor: los contadores le llegan por accumulate(), y
// los registros de montaje los lee refresh_mounting() a través del sensor que se le
// pase como parámetro de plantilla. Lo que sí sabe son las dos formas que tiene un
// dato de salud, que no se tratan igual:
//
//   Las cuentas son totales acumulados. Los contadores de un sensor corren libres
//   y no se pueden borrar, así que lo que se publica es el total de sus
//   incrementos: eso es lo que hace que la computadora pueda poner uno en cero
//   igual que cualquier otro parámetro, en lugar de que se lo pisen en el período
//   siguiente. Saturan en lugar de dar la vuelta, como `late` y `missed` en
//   SampleClock: un bus trabado suma 512 desbordes de una vez.
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
        overruns = add_sat(overruns, (uint16_t)(sensor_overruns - m_last_overruns));
        errors   = add_sat(errors,   (uint16_t)(sensor_errors   - m_last_errors));

        m_last_overruns = sensor_overruns;
        m_last_errors   = sensor_errors;

        // A diferencia de los contadores, esto es un estado y no una cuenta: dice si
        // el sensor está contestando ahora, no cuántas veces falló. Es lo que
        // distingue un imán mal montado --el sensor contesta y se queja del imán--
        // de un sensor que directamente no está en el bus.
        present = responds ? 1 : 0;
    }

    // Refresca la visión que el propio sensor tiene del imán: el registro de estado,
    // la ganancia con la que lee y el módulo del campo. Cada lectura le cuesta al
    // muestreo dos muestras, así que quien llama lo hace sólo entre capturas.
    //
    // Un registro por vez, por turnos. AGC y MAGNITUDE son contiguos y saldrían en
    // una sola lectura de tres bytes, pero una lectura de tres bytes a 400 kHz no
    // entra en el período de muestreo de 200 us, así que el tick siguiente encuentra
    // el bus ocupado y se cuenta un desborde: una verificación que informa una falla
    // de bus en un equipo sano. El driver acepta a lo sumo dos bytes por pedido (ver
    // AS5600::AUX_MAX).
    //
    // `Sensor` es el driver: present(), read_registers(reg, buf, n) y los registros
    // REG_STATUS, REG_AGC y REG_MAGNITUDE_H, como en AS5600.
    template <class Sensor>
    void refresh_mounting(uint32_t now_ms)
    {
        if (!due(now_ms))
        {
            return;
        }

        // Sin sensor en el bus no hay nada que informar del montaje, y dejar el
        // último valor sería peor que no decir nada.
        if (!Sensor::present())
        {
            status    = 0;
            agc       = 0;
            magnitude = 0;
            return;
        }

        uint8_t buf[2];

        switch (m_turn)
        {
            case 0:
                if (Sensor::read_registers(Sensor::REG_STATUS, buf, 1))
                {
                    status = buf[0];
                }
                break;

            case 1:
                if (Sensor::read_registers(Sensor::REG_AGC, buf, 1))
                {
                    agc = buf[0];
                }
                break;

            default:
                if (Sensor::read_registers(Sensor::REG_MAGNITUDE_H, buf, 2))
                {
                    magnitude = (uint16_t)((((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF);
                }
                break;
        }

        // Cuando la ronda se completa, el ritmo se afloja.
        m_turn = (uint8_t)((m_turn + 1) % TURNS);
        if (m_turn == 0)
        {
            m_complete = true;
        }
    }

    private:

    static const uint8_t  TURNS   = 3;     // STATUS, AGC y MAGNITUDE
    static const uint16_t FAST_MS = 50;    // la primera vuelta, para que no se lea un cero
    static const uint16_t SLOW_MS = 500;   // después, el ritmo de algo que se mira entre corridas

    static uint16_t add_sat(uint16_t total, uint16_t delta)
    {
        return (total > (uint16_t)(0xFFFF - delta)) ? (uint16_t)0xFFFF
                                                     : (uint16_t)(total + delta);
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

    uint16_t m_last_overruns;   // la lectura anterior de los contadores del sensor
    uint16_t m_last_errors;

    uint8_t  m_turn;            // el turno y el ritmo del diagnóstico de montaje
    uint32_t m_last_ms;
    bool     m_complete;
};

#endif  // ANGLESENSOR_SENSORHEALTH_H
