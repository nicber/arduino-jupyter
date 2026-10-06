// Sensor magnético de ángulo AS5600, leído de manera asíncrona para un lazo de
// muestreo rápido.
//
// El AS5600 suprime el autoincremento de su puntero de direcciones en las
// lecturas de los registros ANGLE, RAW ANGLE y MAGNITUDE (hoja de datos [v1-06]
// 2018-Jun-20, página 13), así que mientras el puntero está estacionado en RAW
// ANGLE cada muestra es una lectura simple de dos bytes, sin escritura de
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
//   static bool write_register(uint8_t reg, const uint8_t* data, uint8_t length);
//   static bool ok(uint8_t status);
//
// La escritura no lleva callback y no es parte del lazo de muestreo: sirve para
// configurar el sensor, que es algo que pasa entre corridas y no dentro de una.
// Ver write_registers().

#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <stdint.h>
#include <util/atomic.h>

template <class Bus>
class AS5600
{
    public:

    // Los registros que usa el driver, de la hoja de datos.
    static const uint8_t DEVICE_ADDRESS  = 0x36;  // 7 bits, 0110110b
    static const uint8_t REG_CONF_H      = 0x07;  // filtro, histéresis, salida, potencia
    static const uint8_t REG_STATUS      = 0x0B;
    static const uint8_t REG_RAWANGLE_H  = 0x0C;  // sin recortar por ZPOS/MPOS
    static const uint8_t REG_AGC         = 0x1A;
    static const uint8_t REG_MAGNITUDE_H = 0x1B;

    // Bits SF del registro CONF (figura 22): el filtro lento.
    //
    // Con 16x --el valor de encendido-- el retardo de respuesta al escalón son 2,2 ms
    // y el ruido de salida 0,015 grados RMS; con 2x son 0,286 ms y 0,043 grados. Para
    // un lazo de control esos 1,9 ms son mucho más caros que el ruido, y para medir el
    // error de ángulo del sensor son decisivos: a 5 vueltas por segundo, 2,2 ms son
    // 45 cuentas de corrimiento sobre un error que se espera de unas pocas.
    static const uint8_t SF_16X = 0;
    static const uint8_t SF_8X  = 1;
    static const uint8_t SF_4X  = 2;
    static const uint8_t SF_2X  = 3;

    // Lo más largo que pide una lectura de mantenimiento. Dos bytes son el CONF
    // o el MAGNITUDE. Una lectura con dirección de registro ya le cuesta dos
    // muestras al lazo (ver do_transfer()), y una más larga ocuparía el bus
    // todavía más tiempo. Quien necesite más registros que lea de a poco.
    static const uint8_t AUX_MAX = 2;

    // Bits del registro STATUS (figura 23), los que decodifica la computadora en
    // `ang_status`.
    static const uint8_t STATUS_MH = _BV(3);   // desborde de ganancia mínima: imán muy fuerte
    static const uint8_t STATUS_ML = _BV(4);   // desborde de ganancia máxima: imán muy débil
    static const uint8_t STATUS_MD = _BV(5);   // se detectó el imán

    // Fallas de transferencia seguidas a partir de las cuales se da el sensor
    // por desconectado. Treinta y dos a 5 kHz son 6,4 ms: lo bastante como para
    // no confundir una falla transitoria del bus con una desconexión.
    static const uint8_t MISSING_AFTER = 32;

    // Ticks con una transferencia en curso a partir de los cuales se da el bus por
    // trabado y se reinicia la máquina de estados de TWI. `m_inflight` sólo lo bajan
    // las dos devoluciones de llamada, así que si TWI se cuelga --SDA sujeto por el
    // esclavo, un NACK en un estado no contemplado-- no se lanza ninguna
    // transferencia nunca más: el ángulo se congela en su último valor, cada tick
    // cuenta un desborde y `bus_recover()` sólo corre en setup(). Y nI2C no lo
    // arregla: en el camino asíncrono no hay plazo, WaitForComplete() vive adentro
    // de #ifdef CTWI_USING_BLOCKING_ACCESS.
    //
    // 512 ticks a 5 kHz son 102 ms. Muy por encima de cualquier transferencia real
    // --una muestra son ~90 us y la más larga ~140-- y de sobra para no confundirlo
    // con una transferencia lenta: lo que se recupera son bloqueos permanentes.
    static const uint16_t STALL_TICKS = 512;

    // Dado por ausente, una de cada RETRY_SAMPLES muestras vuelve a intentar
    // una transferencia. A 5 kHz son dos sondeos por segundo, que alcanzan para que el
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
            // La transferencia anterior no terminó. Si era una muestra, el bus no
            // está llegando y eso es un desborde. Si era una lectura de
            // mantenimiento, no: una lectura con dirección de registro escribe el
            // puntero, hace un restart y recién ahí lee, y puede no terminar
            // antes del tick siguiente. La muestra se pierde igual --en total dos
            // de las 5000 del segundo-- pero es un costo previsto de la lectura, y
            // contarlo como desborde haría que la puesta en marcha informara
            // falsas fallas de bus en un equipo sano.
            if (!m_aux_inflight)
            {
                m_overruns++;
            }

            // Y si lleva demasiados ticks así, el bus está trabado, y se reinicia el
            // TWI. Eso libera el hardware, pero no la cola de nI2C: el paquete
            // trabado sigue al frente y nI2C no lo relanza hasta que la cola se llena
            // (SIZE_QUEUE = 4), así que la salida tarda unos cuatro ciclos de
            // STALL_TICKS, ~0,4 s, más una espera activa de nI2C de decenas de ms
            // adentro de esta ISR. Destrabar la cola en el acto pide tocar nI2C. Sin
            // verificar en el banco.
            if (++m_stall >= STALL_TICKS)
            {
                m_stall = 0;
#ifdef TWCR
                TWCR = (uint8_t)(TWCR & ~_BV(TWEN));
                TWCR = (uint8_t)(TWCR |  _BV(TWEN));
#endif
                m_armed        = false;
                m_aux_inflight = false;
                m_inflight     = false;
                fail();
            }
            return;
        }

        m_stall = 0;

        // Con el sensor desconectado cada intento falla, y a 5 kHz esa sucesión
        // de errores le quita al lazo de control casi la mitad de sus períodos:
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

        if (m_aux_request)
        {
            // Tarea de mantenimiento, en lugar de una muestra. Leer cualquier
            // otro registro mueve el puntero de direcciones, así que la muestra
            // siguiente tiene que volver a prepararlo.
            m_aux_request  = false;
            m_aux_inflight = true;
            m_aux_flying   = m_aux_seq;
            m_armed = false;
            started = Bus::read_register(m_aux_reg, m_aux, m_aux_len, &process_aux_data);
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
            m_inflight     = false;
            m_aux_inflight = false;
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

    // Busca uno o varios registros contiguos. Le encarga el trabajo al lazo de
    // muestreo, que lo hace en su próximo tick en lugar de una muestra, y después
    // espera el resultado. Quien llama se bloquea a lo sumo un período de
    // muestreo; el lazo en sí nunca se demora y nunca lee un puntero
    // desactualizado. Cuesta 2 muestras de 5000. Devuelve false si el lazo de
    // muestreo no está corriendo.
    static bool read_registers(uint8_t reg, uint8_t* out, uint8_t length,
                               uint16_t timeout_ms = 5)
    {
        // Sin sensor en el bus la petición no se puede satisfacer, y esperar el
        // tiempo completo sólo serviría para bloquear a quien llama.
        if (!m_present || length == 0 || length > AUX_MAX)
        {
            return false;
        }

        // Cada pedido lleva su número, y quien espera compara contra el suyo. Sin
        // eso, un pedido que venció después de que la ISR ya lo había tomado deja la
        // transferencia en curso: `m_aux_ready` se pone en verdadero más tarde, con
        // `m_aux[]` cargado del registro VIEJO, y el pedido siguiente --que acaba de
        // poner `m_aux_ready` en falso-- lo ve puesto enseguida y devuelve los bytes
        // equivocados. En concreto: los dos bytes de MAGNITUDE adentro de
        // `g_health.status`. Con el número, una respuesta atrasada no lo satisface a
        // nadie y el que espera vence, que es lo correcto.
        const uint8_t seq = (uint8_t)(m_aux_seq + 1);

        m_aux_ready   = false;
        m_aux_reg     = reg;
        m_aux_len     = length;
        m_aux_seq     = seq;
        m_aux_request = true;

        uint32_t deadline = millis() + timeout_ms;

        while (!(m_aux_ready && m_aux_done == seq))
        {
            if ((int32_t)(millis() - deadline) >= 0)
            {
                // La petición puede seguir en pie; retirarla evita que un tick
                // posterior gaste una muestra en algo que ya nadie espera.
                m_aux_request = false;
                return false;
            }
        }

        for (uint8_t i = 0; i < length; i++)
        {
            out[i] = m_aux[i];
        }

        return true;
    }

    // Escribe registros contiguos. A diferencia de las lecturas, esto NO pasa por
    // el lazo de muestreo: nI2C encola la escritura y la completa su propia ISR,
    // pero encolar reserva memoria, y hacer malloc adentro de una ISR de
    // temporizador puede fallar de forma esporádica y difícil de reproducir. Así
    // que quien llama tiene que parar el muestreador primero, como hace
    // write_slow_filter(). Configurar el sensor pasa entre corridas, no dentro de una.
    //
    // El puntero de direcciones queda donde lo deje la escritura, así que la
    // muestra siguiente vuelve a fijarlo.
    static bool write_registers(uint8_t reg, const uint8_t* data, uint8_t length)
    {
        m_armed = false;
        return Bus::write_register(reg, data, length);
    }

    // Si hay una transferencia en curso. Se consulta antes de escribir, para no
    // meterle una escritura al bus por encima de una lectura a medio hacer.
    static bool busy(void) { return m_inflight; }

    // Escribe los bits SF del CONF, el filtro lento del sensor. Lee-modifica-escribe,
    // porque CONF también lleva la histéresis, el modo de potencia y la salida.
    //
    // La lectura viaja en un tick del muestreador, así que hace falta que ya esté
    // corriendo. La escritura no puede (ver write_registers()), así que `clock` --lo
    // que llama a do_transfer() desde su ISR; alcanza con que tenga pause() y
    // resume()-- se para un par de milisegundos: se espera, acotado, a que termine
    // la transferencia que ya estaba en el aire, y se deja holgura para la
    // escritura. Devuelve false si el sensor no contestó.
    template <class Clock>
    static bool write_slow_filter(uint8_t sf, Clock& clock)
    {
        uint8_t conf[2];

        if (!read_registers(REG_CONF_H, conf, 2))
        {
            return false;
        }

        uint16_t value = (uint16_t)(((uint16_t)conf[0] << 8) | conf[1]);
        value = (uint16_t)((value & ~0x0300u) | ((uint16_t)(sf & 0x03) << 8));

        conf[0] = (uint8_t)(value >> 8);
        conf[1] = (uint8_t)value;

        clock.pause();

        // Sin sensor en el bus esto no puede quedarse esperando para siempre.
        const uint32_t deadline = millis() + 5;
        while (busy() && (int32_t)(millis() - deadline) < 0)
        {
        }

        const bool queued = write_registers(REG_CONF_H, conf, 2);

        // Cuatro bytes a 400 kHz son unos 100 us; dos milisegundos es holgura, no
        // cálculo.
        delay(2);

        clock.resume();

        return queued;
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
    static void process_aux_data(uint8_t status)
    {
        if (Bus::ok(status))
        {
            m_aux_done  = m_aux_flying;
            m_aux_ready = true;
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
        m_armed        = false;
        m_aux_inflight = false;
        m_inflight     = false;
    }

    static uint16_t snapshot(const volatile uint16_t& counter)
    {
        uint16_t value;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = counter; }
        return value;
    }

    // Sólo se tocan en contexto de ISR, así que van sin calificar. m_aux es un
    // buffer aparte y no una ampliación de m_rx: la lectura de mantenimiento y la
    // muestra usan el mismo bus pero no el mismo camino, y compartir el buffer
    // haría que un STATUS a destiempo apareciera como un ángulo.
    static uint8_t m_rx[2];
    static uint8_t m_aux[AUX_MAX];
    // volatile: se escriben en loop() y la ISR del muestreador los lee. Sin calificar,
    // el lenguaje no garantiza su orden contra la escritura de m_aux_request, que es
    // volatile, y el compilador puede moverlos después. No cuesta nada.
    static volatile uint8_t m_aux_reg;
    static volatile uint8_t m_aux_len;

    static volatile uint8_t  m_aux_seq;     // el número del último pedido
    static volatile uint8_t  m_aux_flying;  // el del que la ISR tomó
    static volatile uint8_t  m_aux_done;    // el del que contestó
    static volatile uint16_t m_stall;       // ticks seguidos con el bus ocupado

    static volatile bool     m_inflight;
    static volatile bool     m_armed;
    static volatile bool     m_aux_request;
    static volatile bool     m_aux_inflight;
    static volatile bool     m_aux_ready;
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
template <class Bus> uint8_t           AS5600<Bus>::m_aux[AS5600<Bus>::AUX_MAX];
template <class Bus> volatile uint8_t  AS5600<Bus>::m_aux_reg        = 0;
template <class Bus> volatile uint8_t  AS5600<Bus>::m_aux_len        = 0;
template <class Bus> volatile uint8_t  AS5600<Bus>::m_aux_seq        = 0;
template <class Bus> volatile uint8_t  AS5600<Bus>::m_aux_flying     = 0;
template <class Bus> volatile uint8_t  AS5600<Bus>::m_aux_done       = 0;
template <class Bus> volatile uint16_t AS5600<Bus>::m_stall          = 0;
template <class Bus> volatile bool     AS5600<Bus>::m_inflight       = false;
template <class Bus> volatile bool     AS5600<Bus>::m_armed          = false;
template <class Bus> volatile bool     AS5600<Bus>::m_aux_request    = false;
template <class Bus> volatile bool     AS5600<Bus>::m_aux_inflight   = false;
template <class Bus> volatile bool     AS5600<Bus>::m_aux_ready      = false;
template <class Bus> volatile uint16_t AS5600<Bus>::m_counts         = 0;
template <class Bus> volatile uint16_t AS5600<Bus>::m_samples        = 0;
template <class Bus> volatile uint16_t AS5600<Bus>::m_overruns       = 0;
template <class Bus> volatile uint16_t AS5600<Bus>::m_errors         = 0;
template <class Bus> volatile bool     AS5600<Bus>::m_present        = true;
template <class Bus> volatile uint8_t  AS5600<Bus>::m_consecutive    = 0;
template <class Bus> uint16_t          AS5600<Bus>::m_backoff        = 0;

#endif  // AS5600_H
