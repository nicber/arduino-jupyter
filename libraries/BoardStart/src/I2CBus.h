#pragma once

#include <Arduino.h>

// El bus I2C: destrabarlo.
//
// Parte de BoardStart, que eran tres trabajos en un header: el reloj, el ADC y el
// bus. Cada uno falla de manera distinta y se arregla en otro lugar, asi que cada
// uno tiene el suyo. BoardStart.h sigue existiendo e incluye los tres, para quien
// quiera el arranque entero sin elegir.

namespace board
{
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
inline uint8_t bus_recover()
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

}  // namespace board
