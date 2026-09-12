#pragma once

#include <Arduino.h>

// El ADC: que placa es, y dejarlo midiendo contra Vcc.
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

// Deja el ADC midiendo contra Vcc de verdad, en las dos placas del banco.
//
// En el LGT8F328P los bits REFS del ADMUX NO eligen la referencia. La eligen DACON y
// el bit REFS2 de ADCSRD, y REFS queda de resabio porque el core lgt8fx lo escribe
// igual después de haber configurado los otros. Un sketch que escriba sólo REFS no
// elige nada en esa placa: la referencia queda en lo que haya quedado de antes, y
// eso es lo que explicaba que la misma lectura diera números distintos en corridas
// distintas.
//
// Las dos placas corren el mismo binario y el core es el del ATmega, así que estos
// registros no existen por nombre y van por dirección. Sólo se los toca cuando la
// placa es la del ADC de 12 bits: en el UNO 0xA0 y 0xAD no son registros, y ahí los
// bits REFS alcanzan y son los suyos.
static const uint16_t LGT_DACON  = 0xA0;
static const uint16_t LGT_ADCSRD = 0xAD;
static const uint8_t  LGT_REFS2  = 6;

// `doce_bits` distingue las dos placas, y es lo que devuelve adc_full_scale().
inline void adc_select_vcc(bool doce_bits)
{
    if (!doce_bits) {
        return;
    }

    _SFR_MEM8(LGT_ADCSRD) &= (uint8_t)~_BV(LGT_REFS2);
    _SFR_MEM8(LGT_DACON)  &= 0x0C;      // DEFAULT del core: Vcc
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
// eligen la referencia --ver adc_select_vcc()-- así que no hay ninguna
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

}  // namespace board
