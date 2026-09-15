// Un ángulo de una vuelta convertido en uno que no salta al dar la vuelta.
//
// No sabe de qué sensor viene la cuenta. Recibe el ángulo ya corregido por
// update(), y de la calibración tampoco sabe nada: eso deja a este módulo entero
// del lado de la aritmética, comprobable en la máquina de escritorio, y deja la
// decisión de corregir o no en manos de quien la toma.
//
// Tampoco filtra, ni elige un cero, ni da vuelta el signo. Un filtro acá se
// identifica después como si fuera un polo del motor, y el signo con el que el
// ángulo acompaña al comando es de qué lado están dos cables, no de este módulo:
// lo aplica quien lo usa, sobre lo ya desenrollado (Banco con `ang_inv`).

#ifndef ANGLESENSOR_ANGLETRACKER_H
#define ANGLESENSOR_ANGLETRACKER_H

#include <stdint.h>

template <uint16_t PerRev = 4096>
class AngleTracker
{
    public:

    // Adentro de la vuelta y a lo largo de las vueltas, cada uno con nombre: el
    // primero da la vuelta a las PerRev cuentas y el segundo no.
    typedef int16_t Counts;
    typedef int32_t Unwrapped;

    static const uint16_t PER_REV = PerRev;

    Counts    y;        // el último ángulo de adentro de la vuelta
    Unwrapped y_uw;     // el mismo ángulo desenrollado

    constexpr AngleTracker()
        : y(0)
        , y_uw(0)
    {
    }

    // El camino más corto de `b` hasta `a` en la circunferencia. Vale mientras el
    // eje gire menos de media vuelta entre dos llamadas a update(): llamado a 5 kHz,
    // como lo llama Banco, son 2500 vueltas por segundo (15 700 rad/s).
    static Counts wrapped_error(Counts a, Counts b)
    {
        const Counts half = (Counts)(PerRev / 2);
        return (Counts)((((Counts)(a - b) + half) & (Counts)(PerRev - 1)) - half);
    }

    void update(Counts counts)
    {
        y_uw += wrapped_error(counts, y);
        y     = counts;
    }
};

#endif  // ANGLESENSOR_ANGLETRACKER_H
