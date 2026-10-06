// La corriente con sentido, a partir de las sumas de conversiones que entrega RowAdc
// fila por fila: la cadena entera, en el orden en que va.
//
//   1. WindowMean promedia las últimas `window.rows` filas, de A0 y de A1 con la
//      misma ventana.
//   2. SupplyRatio lleva ese promedio a cuentas equivalentes, contra la alimentación
//      del sensor si hay divisor (`ratio.div_e4`), en dieciseisavos de cuenta.
//   3. Se resta el cero del sensor (`zero`).
//   4. MainsNotch saca la red y el Nyquist de las filas, si están prendidos.
//   5. Se aplica el signo del banco (`invert`).
//
// Todo en dieciseisavos de cuenta (FRAC_BITS), sin redondear entre un paso y el
// siguiente: el redondeo a cuenta entera es, medido, el que más ruido pone en toda la
// cadena (ver SupplyRatio.h). Quien necesite cuentas enteras redondea una vez, al
// final, como LoopCurrent.
//
// No toca el conversor ni sabe cuál es, así que es aritmética pura salvo el punto
// flotante de MainsNotch::apply(), y se prueba en la máquina de escritorio. No filtra
// más allá de la ventana y el notch: un filtro acá se confunde con la planta que se
// mide.

#ifndef SENSE_CURRENTSENSE_H
#define SENSE_CURRENTSENSE_H

#include <stdint.h>

#include "MainsNotch.h"
#include "SupplyRatio.h"
#include "WindowMean.h"

class CurrentSense
{
    public:

    // Bits fraccionarios de `i`. La unidad la fija SupplyRatio y la comparte el notch,
    // que lleva la señal adentro sin volver a escalarla.
    static const uint8_t FRAC_BITS = SupplyRatio::FRAC_BITS;
    static_assert((int)MainsNotch::FRAC_BITS == (int)SupplyRatio::FRAC_BITS,
                  "el notch y el cociente tienen que llevar la senal en la misma unidad");

    // --------------------------------------------------------------- parámetros
    // Públicos porque la tabla del enlace toma su dirección. Mover y llamar a apply().

    SupplyRatio ratio;      // `div_e4`: el divisor de A1; `supply`: la última media de A1

    // El cero del sensor, en cuentas crudas del conversor y no en dieciseisavos: es la
    // unidad en la que la computadora lo mide y la que informa `cur_zero`. En
    // dieciseisavos un reposo de 2000 cuentas son 32000, que no le deja al int16 margen
    // para la tolerancia del divisor.
    int16_t zero;

    uint8_t invert;         // 1: la corriente sale negativa cuando un comando positivo la hace circular

    int16_t i;              // la corriente, alrededor del cero y con signo, en dieciseisavos

    // Los grandes al final: con los chicos primero, el AVR los alcanza con un
    // desplazamiento corto desde el comienzo del objeto.
    MainsNotch  notch;      // `harmonics`, `pole_milli`, `nyquist`
    WindowMean  window;     // `rows`: filas en la ventana

    constexpr CurrentSense(int16_t initial_zero, uint8_t initial_rows)
        : ratio()
        , zero(initial_zero)
        , invert(0)
        , i(0)
        , notch()
        , window(initial_rows)
        , m_supply(initial_rows)
    {
    }

    // `row_hz` es la frecuencia de las filas, que fija dónde caen los notch.
    void apply(float row_hz)
    {
        ratio.apply();
        window.apply();
        m_supply.rows = window.rows;
        m_supply.apply();
        notch.apply(row_hz);
    }

    // Una fila: la suma de las conversiones de A0 y cuántas fueron, y lo mismo de A1
    // (ceros si no se alterna). Ver RowAdc::row().
    void push(uint32_t sum0, uint16_t n0, uint32_t sum1, uint16_t n1)
    {
        window.push(sum0, n0);
        m_supply.push(sum1, n1);
        update_q4(ratio.counts_q4(window.total(), window.count(),
                                  m_supply.total(), m_supply.count()));
    }

    // Los pasos 3 a 5, desde una cuenta ya promediada, en dieciseisavos.
    void update_q4(int32_t raw_q4)
    {
        // La resta en 32 bits y saturando: con el cero cerca de un extremo `raw - zero`
        // se sale del int16 y daría la vuelta, o sea la corriente con el signo cambiado.
        const int16_t centered = sat(raw_q4 - ((int32_t)zero << FRAC_BITS));
        const int16_t y = notch.step_q4(centered);

        // La negación también en 32 bits: -INT16_MIN no entra en un int16 y el signo
        // no se aplicaría. Se alcanza sin ninguna perilla rara: una fila sin
        // conversiones de A0 da 0 cuentas, que con el cero en 2048 es 0 - 2048 * 16,
        // exactamente INT16_MIN.
        i = invert ? sat(-(int32_t)y) : y;
    }

    private:

    static int16_t sat(int32_t v)
    {
        if (v > INT16_MAX) return INT16_MAX;
        if (v < INT16_MIN) return INT16_MIN;
        return (int16_t)v;
    }

    WindowMean m_supply;    // A1, con la misma ventana que A0
};

#endif  // SENSE_CURRENTSENSE_H
