#pragma once

#include <Arduino.h>
#include <avr/power.h>

// El reloj: dejarlo donde el compilador cree que esta.
//
// Parte de BoardStart, que eran tres trabajos en un header: el reloj, el ADC y el
// bus. Cada uno falla de manera distinta y se arregla en otro lugar, asi que cada
// uno tiene el suyo. BoardStart.h sigue existiendo e incluye los tres, para quien
// quiera el arranque entero sin elegir.

namespace board
{
// ------------------------------------------------------------------ el reloj

// Todo lo que hace este proyecto está calculado para 16 MHz: el puerto serie a
// 1 Mbaud sale de un divisor exacto del USART, el muestreador de 5 kHz de un TOP
// del Timer2, y el PWM del puente de un TOP del Timer1. Si la placa no corre a
// 16 MHz, ninguna de esas tres cosas vale --y el síntoma es el peor de todos,
// porque el puerto serie también emite al ritmo equivocado y entonces la placa
// no puede ni avisar lo que le pasa. Se ve como un monitor lleno de basura.
//
// En este banco hay dos clases de placa, y `CLKPR` las distingue. El core de
// Arduino no lo toca, así que al entrar a `setup()` todavía dice lo que dejó el
// fusible CKDIV8 al arrancar:
//
//   UNO con cristal de 16 MHz --> CLKPR = 0. Corre a 16 MHz y no hay nada que
//   hacer.
//
//   Clon con LGT8F328P --> CLKPR = 3, o sea dividido por 8. Ese chip no lleva
//   cristal: usa un RC interno de 32 MHz, así que arranca a 4 MHz, un cuarto de
//   lo que el sketch supone, y a 1 Mbaud emite a 250 kbaud. Pasar el divisor a 2
//   lo deja en 16 MHz. Medido en este banco contra el reloj de la computadora:
//   16,04 MHz, 0,25 % de error, bastante menos de lo que un UART tolera.
//
// La regla es entonces deliberadamente conservadora: a una placa que arrancó sin
// dividir no se le toca nada, así que la que hoy anda sigue andando igual. Sólo
// se corrige la que arrancó dividida, que es exactamente la que hoy no anda de
// ninguna manera.
//
// Que la corrección haya quedado bien no se da por sentado: lo verifica
// `bringup()` desde la computadora, que mide la frecuencia real del lazo contra
// el reloj del host. Un reloj cuatro veces más lento aparece ahí como 125 Hz
// donde se esperaban 500.
// El CLKPR con el que arrancó la placa, guardado antes de corregirlo. Sirve de
// segunda opinión para distinguir las dos placas del banco cuando hay que
// decidir algo más que el reloj; ver adc_full_scale(). Vale 0 si nadie llamó
// todavía a clock_begin(), que es el caso del UNO de todos modos.
// Se guarda en un miembro de clase y no en un static adentro de la función: un
// static de función es estado escondido en un lugar donde nadie lo busca, y además
// arrastra el guardia de inicialización que el core apaga con
// -fno-threadsafe-statics. La plantilla es lo que permite definir el miembro en el
// header sin un .cpp, y deja una sola instancia aunque lo incluyan varias unidades.
template <class Dummy>
struct ClockStateT
{
    static uint8_t reset_clkpr;
};
template <class Dummy> uint8_t ClockStateT<Dummy>::reset_clkpr = 0;

typedef ClockStateT<void> ClockState;

inline uint8_t& reset_clkpr()
{
    return ClockState::reset_clkpr;
}

inline void clock_begin()
{
    reset_clkpr() = CLKPR;

    if (CLKPR != 0) {
        clock_prescale_set(clock_div_2);
    }
}
}  // namespace board
