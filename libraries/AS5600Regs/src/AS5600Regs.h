// El mapa de registros del AS5600, tal como lo declara la hoja de datos.
//
// Vive aparte del driver porque hay dos clases de código que lo necesitan y no
// pueden compartir el driver: el lazo, que habla por un bus asincrónico desde una
// ISR, y la puesta en marcha, que habla por Wire y bloquea a propósito porque no
// tiene ningún plazo que cumplir y quiere el camino más simple posible.
//
// Estaba declarado dos veces, y ésa es la clase de duplicación que muerde callada:
// una dirección corregida en un lado y no en el otro no falla al compilar.
//
// Y está en una librería propia, sin ninguna dependencia, porque el build de Arduino
// compila entera cualquier librería cuyo header se incluya: dejarlo adentro del
// driver asincrónico hacía que la puesta en marcha arrastrara ese bus, y su vector
// de interrupción de TWI choca con el de Wire. El error que sale de ahí es
// «multiple definition of __vector_24» y no dice nada de esto.
//
// Es un namespace y no una clase base. No hay nada que heredar acá --son números de
// la hoja de datos-- y el driver los reexporta con sus propios nombres, así que
// quien use el driver los sigue encontrando donde los encontraba.

#ifndef AS5600ASYNC_AS5600REGS_H
#define AS5600ASYNC_AS5600REGS_H

#include <stdint.h>
#include <avr/io.h>

namespace as5600
{
    static const uint8_t DEVICE_ADDRESS = 0x36;   // 7 bits, 0110110b

    // Configuración, que vive en memoria no volátil salvo CONF.
    static const uint8_t REG_ZMCO        = 0x00;  // cuántas veces se grabó ZPOS/MPOS
    static const uint8_t REG_ZPOS_H      = 0x01;  // principio del rango
    static const uint8_t REG_MPOS_H      = 0x03;  // fin del rango
    static const uint8_t REG_MANG_H      = 0x05;  // ángulo máximo
    static const uint8_t REG_CONF_H      = 0x07;  // filtro, histéresis, salida, potencia

    // Salida.
    static const uint8_t REG_RAWANGLE_H  = 0x0C;  // sin recortar por ZPOS/MPOS
    static const uint8_t REG_ANGLE_H     = 0x0E;  // recortado y escalado

    // Estado del imán.
    static const uint8_t REG_STATUS      = 0x0B;
    static const uint8_t REG_AGC         = 0x1A;
    static const uint8_t REG_MAGNITUDE_H = 0x1B;

    // Bits del registro STATUS (figura 23 de la hoja de datos).
    static const uint8_t STATUS_MH = _BV(3);   // desborde de ganancia mínima: imán muy fuerte
    static const uint8_t STATUS_ML = _BV(4);   // desborde de ganancia máxima: imán muy débil
    static const uint8_t STATUS_MD = _BV(5);   // se detectó el imán

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
}

#endif  // AS5600ASYNC_AS5600REGS_H
