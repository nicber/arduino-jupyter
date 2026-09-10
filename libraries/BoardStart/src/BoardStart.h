#pragma once

#include <Arduino.h>
#include <avr/power.h>

// Las dos cosas que hay que hacer antes de que el sketch pueda confiar en el
// hardware: dejar el reloj donde el compilador cree que está, y destrabar el bus
// I2C si quedó tomado. Las dos son propiedades del banco y no del programa, las
// dos fallan de manera muda --el síntoma es una placa que «no anda»-- y las dos
// se arreglan en tres líneas si uno sabe cuáles.

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
inline void boardClockBegin()
{
    if (CLKPR != 0) {
        clock_prescale_set(clock_div_2);
    }
}

// -------------------------------------------------------------- el bus I2C

// Destraba el bus cuando quedó tomado por un esclavo a medio hablar. Devuelve
// cuántos pulsos de reloj hicieron falta; cero quiere decir que estaba libre.
//
// El caso es rutinario acá y no una rareza. Grabar la placa la resetea, y el
// reset no le avisa al AS5600: si cayó en medio de una lectura, el sensor se
// queda esperando los pulsos de reloj que le faltan, y mientras espera mantiene
// SDA en bajo. Para el maestro que arranca de nuevo eso es un bus ocupado, así
// que no llega a generar el START --TWSTA queda pedido y TWINT nunca se activa--
// y se queda ahí para siempre. El muestreador de 5 kHz informa entonces cero
// muestras y un desborde por tick, que es la forma más confusa posible de decir
// «un cable». Y como depende de en qué parte de una transferencia cayó el reset,
// aparece y desaparece entre una grabación y la siguiente.
//
// El remedio es el de la especificación: darle al esclavo los pulsos de reloj
// que le faltan --nueve alcanzan para terminar cualquier byte más su ACK-- hasta
// que suelte SDA, y cerrar con un STOP para que quede en un estado conocido. Se
// hace moviendo los pines a mano, así que hay que soltarlos del periférico
// primero: mientras TWEN esté puesto, SDA y SCL los gobierna el TWI y no el
// puerto.
inline uint8_t i2cBusRecover()
{
    const uint8_t twcr = TWCR;
    TWCR = 0;

    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(10);

    uint8_t pulses = 0;
    while (digitalRead(SDA) == LOW && pulses < 9) {
        // Bajar el pin es soltar el pull-up antes de pasar a salida: al revés,
        // entre las dos instrucciones el pin queda en alto y le pelea al esclavo.
        digitalWrite(SCL, LOW);
        pinMode(SCL, OUTPUT);
        delayMicroseconds(5);
        pinMode(SCL, INPUT_PULLUP);   // colector abierto: sube por el pull-up
        delayMicroseconds(5);
        pulses++;
    }

    if (pulses) {
        // STOP a mano: con SCL arriba, SDA pasa de bajo a alto.
        digitalWrite(SDA, LOW);
        pinMode(SDA, OUTPUT);
        delayMicroseconds(5);
        pinMode(SDA, INPUT_PULLUP);
        delayMicroseconds(5);
    }

    TWCR = twcr;
    return pulses;
}
