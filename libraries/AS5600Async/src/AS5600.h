// Sensor magnético de ángulo AS5600, leído de manera asíncrona para un lazo de
// muestreo rápido.
//
// El AS5600 suprime el autoincremento de su puntero de direcciones en las
// lecturas de los registros ANGLE, RAW ANGLE y MAGNITUDE (hoja de datos [v1-06]
// 2018-Jun-20, página 13), así que mientras el puntero está estacionado en RAW
// ANGLE cada muestra es una lectura pelada de dos bytes, sin escritura de
// registro:
//   START + SLA+R + alto + bajo + STOP  ~= 90 us a 400 kHz.
// Eso entra con holgura en un presupuesto de 200 us (5 kHz); recargar el puntero
// en cada muestra costaría alrededor de la mitad más.
//
// El puntero se prepara de manera perezosa: cada vez que se sabe que está en otro
// lado (al arrancar, después de leer STATUS, después de un error de bus) la
// muestra siguiente usa la forma con dirección de registro, que cuesta ~140 us
// una sola vez y lo vuelve a estacionar.
//
// Bus es una política de despacho estático (sin funciones virtuales). Tiene que
// proveer:
//   static void begin(uint8_t address);
//   static bool read(uint8_t* buffer, uint8_t length, void (*callback)(uint8_t));
//   static bool read_register(uint8_t reg, uint8_t* buffer, uint8_t length,
//                             void (*callback)(uint8_t));
//   static bool ok(uint8_t status);

#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <stdint.h>
#include <util/atomic.h>

template <class Bus>
class AS5600
{
    public:

    static const uint8_t DEVICE_ADDRESS = 0x36;  // 7 bits, 0110110b
    static const uint8_t REG_STATUS     = 0x0B;
    static const uint8_t REG_RAWANGLE_H = 0x0C;

    // Bits del registro STATUS (Figura 23 de la hoja de datos).
    static const uint8_t STATUS_MH = _BV(3);  // desborde de ganancia mínima del AGC, imán muy fuerte
    static const uint8_t STATUS_ML = _BV(4);  // desborde de ganancia máxima del AGC, imán muy débil
    static const uint8_t STATUS_MD = _BV(5);  // se detectó el imán

    // Fallas de transferencia seguidas a partir de las cuales se da el sensor
    // por desconectado. Treinta y dos a 5 kHz son 6,4 ms: lo bastante como para
    // no confundir un chispazo del bus con una desconexión.
    static const uint8_t MISSING_AFTER = 32;

    // Dado por ausente, una de cada RETRY_SAMPLES muestras vuelve a intentar de
    // verdad. A 5 kHz son dos sondeos por segundo, que alcanzan para que el
    // sensor se detecte solo al reconectarlo y no le cuestan nada al lazo.
    static const uint16_t RETRY_SAMPLES = 2500;

    // Llamar una vez, antes de que arranque el lazo de muestreo. El puntero de
    // direcciones lo prepara el primer do_transfer(), así que esto no toca el bus
    // y no puede fallar.
    static void begin(void)
    {
        Bus::begin(DEVICE_ADDRESS);
        m_armed = false;
    }

    // Lanza una muestra. No bloquea; está pensado para llamarse desde la ISR de
    // un temporizador.
    static void do_transfer(void)
    {
        if (m_inflight)
        {
            // La transferencia anterior no terminó: el bus no está llegando.
            m_overruns++;
            return;
        }

        // Con el sensor desconectado cada intento falla, y a 5 kHz esa tormenta
        // de errores le come al lazo de control casi la mitad de sus períodos:
        // el bus y la ISR de TWI se quedan con el tiempo que el lazo necesita.
        // Una vez dado por ausente se lo sondea de a ratos, así el lazo recupera
        // su período y el sensor se sigue detectando solo si vuelve.
        if (!m_present && ++m_backoff < RETRY_SAMPLES)
        {
            return;
        }
        m_backoff = 0;

        m_inflight = true;

        bool started;

        if (m_status_request)
        {
            // Tarea de mantenimiento, en lugar de una muestra. Leer STATUS mueve
            // el puntero de direcciones, así que la muestra siguiente tiene que
            // volver a prepararlo.
            m_status_request = false;
            m_armed = false;
            started = Bus::read_register(REG_STATUS, m_rx, 1, &process_status_data);
        }
        else if (m_armed)
        {
            started = Bus::read(m_rx, sizeof(m_rx), &process_read_data);
        }
        else
        {
            started = Bus::read_register(REG_RAWANGLE_H, m_rx, sizeof(m_rx), &process_read_data);
        }

        if (!started)
        {
            m_inflight = false;
            fail();
        }
    }

    // Última muestra completada, 0..4095.
    // Una lectura de 16 bits son dos instrucciones en AVR y la ISR de TWI puede
    // caer entre las dos, así que la lectura se hace atómica y no meramente
    // volátil.
    static uint16_t counts(void) { return snapshot(m_counts); }

    static uint16_t samples(void)  { return snapshot(m_samples); }
    static uint16_t overruns(void) { return snapshot(m_overruns); }
    static uint16_t errors(void)   { return snapshot(m_errors); }

    // Si el sensor contestó alguna de las últimas transferencias. Falso quiere
    // decir que no está en el bus: desconectado, sin alimentación o sin
    // pull-ups. El muestreo sigue corriendo igual, en modo de sondeo espaciado.
    static bool present(void) { return m_present; }

    // Fallas seguidas; vuelve a cero con cada transferencia exitosa.
    static uint8_t consecutive_errors(void) { return m_consecutive; }

    // Busca el registro STATUS. Le encarga el trabajo al lazo de muestreo, que lo
    // hace en su próximo tick en lugar de una muestra, y después espera el
    // resultado. Quien llama se bloquea a lo sumo un período de muestreo; el lazo
    // en sí nunca se demora y nunca lee un puntero desactualizado. Cuesta
    // 2 muestras de 5000. Devuelve false si el lazo de muestreo no está
    // corriendo.
    static bool read_status(uint8_t& out, uint16_t timeout_ms = 5)
    {
        // Sin sensor en el bus la petición no se puede satisfacer, y esperar el
        // tiempo completo sólo serviría para bloquear a quien llama.
        if (!m_present)
        {
            return false;
        }

        m_status_ready = false;
        m_status_request = true;

        uint32_t deadline = millis() + timeout_ms;

        while (!m_status_ready)
        {
            if ((int32_t)(millis() - deadline) >= 0)
            {
                return false;
            }
        }

        out = m_status;
        return true;
    }

    private:

    // Contabiliza una transferencia fallida y, pasadas MISSING_AFTER seguidas,
    // da el sensor por desconectado. Corre en la ISR de TWI.
    static void fail(void)
    {
        m_errors++;

        if (m_consecutive < MISSING_AFTER && ++m_consecutive == MISSING_AFTER)
        {
            m_present = false;
        }
    }

    // Cualquier transferencia que el sensor conteste lo declara presente, así
    // que reconectarlo alcanza para que el muestreo vuelva al ritmo pleno.
    static void succeed(void)
    {
        m_consecutive = 0;
        m_present     = true;
    }

    // Corre en la ISR de TWI.
    static void process_read_data(uint8_t status)
    {
        if (Bus::ok(status))
        {
            // Valor de 12 bits, byte alto primero, nibble superior sin usar.
            m_counts = (((uint16_t)m_rx[0] << 8) | m_rx[1]) & 0x0FFF;
            m_samples++;
            m_armed = true;
            succeed();
        }
        else
        {
            // Una transferencia fallida puede haber dejado el puntero en
            // cualquier lado.
            fail();
            m_armed = false;
        }

        m_inflight = false;
    }

    // Corre en la ISR de TWI.
    static void process_status_data(uint8_t status)
    {
        if (Bus::ok(status))
        {
            m_status = m_rx[0];
            m_status_ready = true;
            succeed();
        }
        else
        {
            fail();
        }

        // El puntero autoincrementó hasta RAW ANGLE, pero la supresión sólo está
        // documentada para cuando el puntero fue *escrito* al byte alto, así que
        // se lo trata como no preparado y se deja que la muestra siguiente lo
        // fije explícitamente.
        m_armed = false;
        m_inflight = false;
    }

    static uint16_t snapshot(const volatile uint16_t& counter)
    {
        uint16_t value;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = counter; }
        return value;
    }

    // Sólo se toca en contexto de ISR, así que va sin calificar.
    static uint8_t m_rx[2];

    static volatile bool     m_inflight;
    static volatile bool     m_armed;
    static volatile bool     m_status_request;
    static volatile bool     m_status_ready;
    static volatile uint8_t  m_status;
    static volatile uint16_t m_counts;
    static volatile uint16_t m_samples;
    static volatile uint16_t m_overruns;
    static volatile uint16_t m_errors;
    static volatile bool     m_present;
    static volatile uint8_t  m_consecutive;

    // Sólo se toca en la ISR del temporizador, así que va sin calificar.
    static uint16_t m_backoff;
};

template <class Bus> uint8_t           AS5600<Bus>::m_rx[2];
template <class Bus> volatile bool     AS5600<Bus>::m_inflight       = false;
template <class Bus> volatile bool     AS5600<Bus>::m_armed          = false;
template <class Bus> volatile bool     AS5600<Bus>::m_status_request = false;
template <class Bus> volatile bool     AS5600<Bus>::m_status_ready   = false;
template <class Bus> volatile uint8_t  AS5600<Bus>::m_status         = 0;
template <class Bus> volatile uint16_t AS5600<Bus>::m_counts         = 0;
template <class Bus> volatile uint16_t AS5600<Bus>::m_samples        = 0;
template <class Bus> volatile uint16_t AS5600<Bus>::m_overruns       = 0;
template <class Bus> volatile uint16_t AS5600<Bus>::m_errors         = 0;
template <class Bus> volatile bool     AS5600<Bus>::m_present        = true;
template <class Bus> volatile uint8_t  AS5600<Bus>::m_consecutive    = 0;
template <class Bus> uint16_t          AS5600<Bus>::m_backoff        = 0;

#endif  // AS5600_H
