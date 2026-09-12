// Un PID entero de punta a punta, dueño de su propio estado y de su propia
// reconfiguración.
//
// Las ganancias son por muestra, no por segundo: u = kp*e + ki*sum(e) + kd*diff(e),
// sin ningún dt en el medio. Eso es lo que hace la aritmética, así que eso es lo
// que significa el parámetro, y así toda escala queda como constante de tiempo de
// compilación. Una computadora que prefiera ganancias en tiempo continuo
// multiplica por dt de su lado.
//
// Lo que este módulo se lleva de su dueño, y que antes andaba suelto, es la
// pregunta de cuándo hay que olvidar la historia. Un PID que hereda el integrador
// de otra configuración da un salto en su primer período, y el caso que de verdad
// muerde no es cambiar de controlador sino cambiar *sobre qué* se cierra el lazo:
// el integrador acumula en las unidades de la magnitud realimentada, así que pasar
// de un ángulo en cuentas a una corriente en cuentas de conversor deja adentro una
// suma que no significa nada en el dominio nuevo. Por eso configure() mira las dos
// cosas y no una. Ver el comentario de arriba de esa función.

#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include <stdint.h>

#include <FixedPoint.h>
#include <FirstOrderFilter.h>

class Pid
{
    public:

    // Las escalas de punto fijo, elegidas según el rango que realmente necesita cada
    // ganancia. El compromiso es magnitud contra resolución, y la tabla del enlace
    // publica FRAC directamente desde estos tipos, así que a la computadora se le
    // informa cada formato desde la declaración que lo define y no desde una
    // constante que hay que mantener sincronizada con ella.
    typedef Fixed<int32_t, 22> Kp;      // +/-511,   resolución 2.4e-7
    typedef Fixed<int32_t, 30> Ki;      // +/-1.99,  resolución 9.3e-10
    typedef Fixed<int32_t, 16> Kd;      // +/-32767, resolución 1.5e-5

    // El error que entra y el comando que sale, cada uno con nombre. El error está en
    // las unidades crudas de lo que se realimente y el comando en cuentas del
    // actuador: son dos dominios distintos y la ganancia es lo único que los une.
    typedef int32_t Error;
    typedef int16_t Command;

    // El filtro del término derivativo. Un polo, y sobre el error: derivar una señal
    // ruidosa sin filtrarla primero es amplificar el ruido y nada más.
    typedef FirstOrderFilter<8>     Filter;
    typedef typename Filter::Alpha  Alpha;

    // --------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección. Llegan ya en la forma
    // que quiere la aritmética: esa conversión es trabajo de la computadora.

    int32_t kp;
    int32_t ki;
    int32_t kd;

    constexpr Pid()
        : kp(0)
        , ki(0)
        , kd(0)
        , m_integral(0)
        , m_integral_max(INT32_MAX / 2)
        , m_e_filt(0)
        , m_e_prev(0)
        , m_filt()
        , m_mode(0xFF)
        , m_target(0xFF)
    {
    }

    // Avisa con qué configuración se está por correr, y reinicia si cambió alguna de
    // las dos.
    //
    // Las dos y no sólo el modo. Un controlador que hereda el integrador del
    // anterior da un salto en su primer período, y eso vale igual cuando lo que
    // cambió es la magnitud realimentada: el integrador quedó acumulado en cuentas
    // de ángulo y de golpe se le aplica a una corriente. Mirar sólo el modo dejaba
    // ese caso afuera, y era alcanzable con dos celdas seguidas de notebook que
    // cambian el objetivo sin cambiar el controlador.
    //
    // Devuelve true si reinició, para que quien llame pueda decirlo si le importa.
    bool configure(uint8_t mode, uint8_t target, Error e)
    {
        if (mode == m_mode && target == m_target)
        {
            return false;
        }

        m_mode   = mode;
        m_target = target;
        reset(e);
        return true;
    }

    void reset(Error e)
    {
        m_integral = 0;
        m_e_filt   = e;
        m_e_prev   = e;
        m_filt.reset(e);
    }

    // Un período: error adentro, comando recortado afuera.
    //
    // `feed_forward` se suma antes de recortar, así que una prealimentación cuenta
    // para la saturación igual que cualquier otro término. `lo` y `hi` los pone el
    // actuador, que es el que sabe en qué cuadrantes puede empujar.
    Command step(Error e, Command lo, Command hi, Command feed_forward)
    {
        m_e_prev = m_e_filt;
        m_e_filt = m_filt.update(e);

        const int32_t candidate = Kp::from_raw(kp).scale(e)
                                + Ki::from_raw(ki).scale(m_integral)
                                + Kd::from_raw(kd).scale(m_e_filt - m_e_prev)
                                + (int32_t)feed_forward;

        const Command u = clamp(candidate, lo, hi);

        // Integración condicional: dejar de cargar el integrador en cuanto el
        // actuador satura en el sentido hacia el que el integrador está empujando.
        const bool saturated = (candidate > hi && e > 0)
                            || (candidate < lo && e < 0);

        if (!saturated)
        {
            // Segunda línea de defensa, y la que importa cuando se cambia ki en plena
            // corrida: acotar la suma en el punto donde su término por sí solo
            // saturaría el actuador, para que el integrador siempre pueda descargarse
            // en un período o dos. La suma se forma en un tipo ancho porque la cota se
            // aplica recién después, y un único error grande podría de otro modo
            // desbordar el acumulador en el camino.
            int64_t sum = (int64_t)m_integral + e;

            if (sum >  m_integral_max) sum =  m_integral_max;
            if (sum < -m_integral_max) sum = -m_integral_max;

            m_integral = (int32_t)sum;
        }

        return u;
    }

    void set_alpha(Alpha alpha) { m_filt.set_alpha(alpha); }

    // Recalcula la cota del integrador a partir de ki y del techo del actuador.
    //
    // Es asunto del controlador y de nadie más: la cota existe para que el término
    // integral pueda descargarse en un período o dos, así que depende de ki, y quien
    // mueva ki no tiene por qué acordarse de esto. Un ki lo bastante chico como para
    // poner la cota más allá de lo que entra en un int32 deja en pie el límite del
    // propio tipo: la acumulación tiene que quedar en rango le importe o no a ki.
    void refresh(Command reach)
    {
        const int32_t magnitude = (ki < 0) ? -ki : ki;

        m_integral_max = INT32_MAX / 2;

        if (magnitude != 0)
        {
            const int64_t bound = ((int64_t)reach << Ki::FRAC) / magnitude;

            if (bound < m_integral_max)
            {
                m_integral_max = (int32_t)bound;
            }
        }
    }

    // Sólo para mirar: es estado interno, y publicarlo como parámetro escribible
    // dejaría a la computadora metiendo mano adentro de la integración.
    int32_t integral(void) const { return m_integral; }

    static Command clamp(int32_t v, Command lo, Command hi)
    {
        if (v < (int32_t)lo) return lo;
        if (v > (int32_t)hi) return hi;
        return (Command)v;
    }

    private:

    // El estado del integrador, en unidades de error sumadas a lo largo de los
    // ticks. Guardar la suma cruda y aplicar ki una sola vez al final es lo que
    // permite que sobreviva una ganancia de 5e-5: la cuantización cae sobre la
    // ganancia, donde es una fracción de un por ciento, en lugar de caer sobre la
    // acumulación, donde se truncaría a cero en cada período.
    int32_t m_integral;
    int32_t m_integral_max;
    int32_t m_e_filt;
    int32_t m_e_prev;
    Filter  m_filt;

    // Con qué configuración se corrió el período anterior. Eran un static adentro de
    // control_step(), y que lo fueran es la razón por la que sólo se vigilaba una de
    // las dos.
    uint8_t m_mode;
    uint8_t m_target;
};

#endif  // CONTROL_PID_H
