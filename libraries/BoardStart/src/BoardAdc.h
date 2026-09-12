#pragma once

#include <Arduino.h>

// El ADC: que placa es, cuanto vale su referencia, y elegirla.
//
// Parte de BoardStart, que eran tres trabajos en un header: el reloj, el ADC y el
// bus. Cada uno falla de manera distinta y se arregla en otro lugar, asi que cada
// uno tiene el suyo. BoardStart.h sigue existiendo e incluye los tres, para quien
// quiera el arranque entero sin elegir.

namespace board
{
// ------------------------------------------------------------------- el ADC

// Una conversión suelta, con el multiplexor y la referencia que se le pidan, y
// dejando el ADC como estaba. Es para el arranque: adentro del lazo el ADC se
// maneja libre y sin esperar.
inline uint16_t adc_once(uint8_t admux)
{
    const uint8_t mux = ADMUX;
    const uint8_t sra = ADCSRA;

    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);   // /128
    ADMUX  = admux;
    delay(5);   // cambiar de referencia pide asentarse, y el bandgap más todavía

    uint16_t valor = 0;
    for (uint8_t i = 0; i < 4; i++) {       // las primeras no valen
        ADCSRA |= _BV(ADSC);
        while (ADCSRA & _BV(ADSC)) {
            ;
        }
        valor = ADC;
    }

    ADMUX  = mux;
    ADCSRA = sra;
    return valor;
}

// Elige la referencia del ADC de verdad, en las dos placas del banco.
//
// En el LGT8F328P los bits REFS del ADMUX NO la eligen. La eligen DACON, VCAL y el
// bit REFS2 de ADCSRD, y REFS queda de resabio porque el core lgt8fx lo escribe
// igual --`ADMUX = analog_reference << 6`-- después de haber configurado los otros
// tres. Ver analogReference() en su wiring_analog.c.
//
// Un sketch que escriba sólo REFS no elige nada en esa placa: la referencia queda
// en lo que haya quedado de antes. Eso es lo que explica por qué la misma lectura
// del canal interno da números distintos en corridas distintas.
//
// Y por eso esto vive acá y no adentro del lazo: todo lo que mide con el ADC al
// arrancar --el fondo de escala, el bandgap, y el estado eléctrico de las líneas
// del bus, que se juzga con umbrales que son fracciones del fondo-- necesita que la
// referencia ya esté elegida. Llamarlo después dejaba esas tres mediciones corriendo
// contra una referencia de resabio, y la del bus es la que importa: sus umbrales no
// significan volts, así que con una referencia chica las dos líneas leen saturadas y
// el diagnóstico no puede quejarse nunca.
//
// Las dos placas corren el mismo binario y el core es el del ATmega, así que estos
// registros no existen por nombre y van por dirección. Sólo se los toca cuando la
// placa es la del ADC de 12 bits: en el UNO 0xA0 y 0xAD no son registros, y no hay
// por qué escribirles.
static const uint16_t LGT_DACON  = 0xA0;
static const uint16_t LGT_ADCSRD = 0xAD;
static const uint16_t LGT_VCAL   = 0xC8;
static const uint16_t LGT_VCAL1  = 0xCD;   // el valor de calibración de 1,024 V
static const uint8_t  LGT_REFS2  = 6;

// `doce_bits` distingue las dos placas, y es lo que devuelve adc_full_scale():
// esa sonda no necesita una referencia correcta, porque cae al CLKPR de arranque.
inline void adc_select_reference(bool doce_bits, bool interna)
{
    if (!doce_bits) {
        return;                 // un ATmega: los bits REFS alcanzan y son los suyos
    }

    _SFR_MEM8(LGT_ADCSRD) &= (uint8_t)~_BV(LGT_REFS2);

    if (interna) {
        // La referencia interna, con VCAL cargado con la calibración de 1,024 V.
        _SFR_MEM8(LGT_DACON) = (uint8_t)((_SFR_MEM8(LGT_DACON) & 0x0C) | 0x02);
        _SFR_MEM8(LGT_VCAL)  = _SFR_MEM8(LGT_VCAL1);
    } else {
        // DEFAULT del core: Vcc, que es la misma elección que REFS=01 en el UNO.
        _SFR_MEM8(LGT_DACON) &= 0x0C;
    }
}

// Cuántas cuentas da el ADC a fondo de escala: 1024 en el UNO, 4096 en el clon
// con LGT8F328P, que trae un ADC de 12 bits. La misma tensión mide cuatro veces
// más en una placa que en la otra, y en este banco las dos corren el mismo
// binario, así que el número no puede ser una constante compilada.
//
// La sonda mide el bandgap tomando como referencia el bandgap mismo: entrada y
// referencia son la misma cosa, así que el resultado es el fondo de escala, valga
// el bandgap 1,0 o 1,2 V. Un ADC de 10 bits satura en 1023 y no puede devolver más,
// de manera que cualquier cosa por encima de 1023 prueba que hay más de 10 bits.
//
// OJO que ese razonamiento vale en el ATmega y no en el clon. Ahí los bits REFS no
// eligen la referencia --ver adc_select_reference()-- así que no hay ninguna
// garantía de que entrada y referencia sean la misma tensión, y la sonda puede
// devolver cualquier cosa. Lo que contesta bien en esa placa es el respaldo: el
// CLKPR de arranque, que la distingue sin ambigüedad. Así que en el clon el que
// carga el peso es el respaldo y no la sonda, y por eso hay que llamar a
// clock_begin() antes que a esto.
inline uint16_t adc_full_scale()
{
    const uint16_t saturado = adc_once((_BV(REFS1) | _BV(REFS0)) | 0x0E);

    if (saturado > 1023) {
        return 4096;
    }
    // Saturar en 1023 es lo que hace un UNO, pero también sería lo que haría una
    // placa de 12 bits cuya sonda midiera algo por debajo de la referencia. No se
    // adivina: el CLKPR de arranque ya distingue las dos placas del banco, y acá
    // contesta bien en los dos casos.
    return reset_clkpr() ? 4096 : 1024;
}

// El bandgap medido contra AVcc, en cuentas. Es la mitad de la calibración de la
// referencia: da la razón entre las dos tensiones, y la otra mitad --cuánto vale
// una de las dos en volts-- hay que medirla una vez con un tester, porque acá
// adentro no hay ninguna tensión conocida contra la cual calibrar.
//
// Con AVcc medido, la referencia interna vale AVcc * cuentas / fondo de escala, y
// ese número es el que va en ADC_REF_MV. El error de ganancia que corrige no es
// chico: el bandgap está especificado entre 1,0 y 1,2 V, o sea +/-10 % de chip a
// chip, y va derecho a los miliamperes que se informan.
//
// En el clon el número NO significa eso, y conviene decirlo con todas las letras
// en lugar de dejar una cuenta que parece una calibración. Los bits REFS del
// ADMUX son los del ATmega y el LGT8F328P tiene su propio juego de referencias
// internas --1,024, 2,048 y 4,096 V--, así que esto mide una contra otra y no un
// bandgap contra AVcc. Medido en este banco: 1027 cuentas de 4096, o sea 0,2507,
// que es 1,024/4,096 con cuatro decimales de acuerdo. El valor además depende de
// con qué referencia venía trabajando el ADC: el mismo código, en un sketch que
// arranca de otra manera, da 2585. Es un dato curioso y no una calibración.
//
// La referencia del clon se resuelve con el core lgt8fx, que la declara por
// nombre (INTERNAL1V024, INTERNAL2V048, INTERNAL4V096) en lugar de dejarla
// adivinar. Ver el comentario de ADC_REF_MV en ControlDemo.ino.
inline uint16_t adc_bandgap()
{
    return adc_once(_BV(REFS0) | 0x0E);
}
}  // namespace board
