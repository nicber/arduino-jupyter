#pragma once

#include <Arduino.h>
#include <avr/power.h>

// El reloj: dejarlo en la frecuencia para la que se compiló el sketch.
//
// Parte de BoardStart, que reúne tres trabajos: el reloj, el ADC y el bus. Cada uno
// falla de manera distinta y se arregla en otro lugar, así que cada uno tiene su
// header. BoardStart.h incluye los tres, para quien quiera el arranque entero sin
// elegir.

namespace board
{
// ------------------------------------------------------------------ el reloj

// Todo lo que hace este proyecto está calculado para 16 MHz: el puerto serie a
// 1 Mbaud sale de un divisor exacto del USART, el muestreador de 5 kHz de un TOP
// del Timer2, y el PWM del puente de un TOP del Timer1. Si la placa no corre a
// 16 MHz, ninguna de esas tres cosas vale --y el síntoma es especialmente difícil
// de diagnosticar, porque el puerto serie también emite al ritmo equivocado y
// entonces la placa no puede informar lo que le pasa. Se ve como un monitor serie
// lleno de caracteres ilegibles.
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
//   16,04 MHz, 0,25 % de error, bastante menos de lo que un UART tolera. Un RC no
//   tiene la exactitud de un cristal: en otra medición, las filas del lazo salieron
//   a 503 Hz en lugar de 500 (ver Sense/MainsNotch.h).
//
// La regla es entonces deliberadamente conservadora: a una placa que arrancó sin
// dividir no se le toca nada, así que la que ya funciona sigue funcionando igual.
// Sólo se corrige la que arrancó dividida, que es exactamente la que sin esto no
// funciona de ninguna manera.
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

// El Timer0, que lleva millis(), a 1000 Hz exactos en lugar de los 976,6 Hz del core.
//
// La interrupción de millis() demora un poco lo que esté corriendo, las conversiones
// del ADC incluidas, así que se ve en la corriente. A 976,6 Hz eso se pliega contra
// filas de 500 Hz a 23,4 Hz: medido en el banco, un tono de 1 a 2 mA con una fila. A
// 1000 Hz sale del mismo reloj que el muestreador de 5 kHz, en fase fija, y se pliega
// a continua, donde lo absorbe el cero.
//
// Modo 7, PWM rápido con TOP = OCR0A: el desborde, que es la interrupción de millis(),
// llega en cada TOP, y 16 MHz / 64 / 250 = 1000 Hz. Sin salidas de comparación, así
// que analogWrite() en los pines 5 y 6 deja de servir.
//
// El precio: el core suma 1,024 ms por desborde, porque cuenta con 256 pasos, así que
// millis(), micros() y delay() quedan un 2,4 % rápidos, y micros() además salta 28 us
// en cada desborde. En este proyecto millis() sólo mide plazos y micros() el retardo
// de atención de una fila (`loop_late`); el tiempo de las filas lo lleva el
// muestreador y no depende de esto.
inline void millis_1000hz()
{
    const uint8_t sreg = SREG;
    cli();
    TCCR0A = _BV(WGM01) | _BV(WGM00);
    TCCR0B = _BV(WGM02) | _BV(CS01) | _BV(CS00);
    OCR0A  = 249;
    TCNT0  = 0;
    SREG   = sreg;
}
}  // namespace board
