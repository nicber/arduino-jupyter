#pragma once

#include <Arduino.h>

// El ADC: qué placa es, y dejarlo midiendo contra Vcc.
//
// Parte de BoardStart, que reúne tres trabajos: el reloj, el ADC y el bus. Cada uno
// falla de manera distinta y se arregla en otro lugar, así que cada uno tiene su
// header. BoardStart.h incluye los tres, para quien quiera el arranque entero sin
// elegir.

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

// Deja el ADC midiendo efectivamente contra Vcc, en las dos placas del banco.
//
// En el LGT8F328P los bits REFS del ADMUX NO eligen la referencia. La elige el bit
// REFS2 de ADCSRD, y REFS queda como un vestigio del ATmega: el core lgt8fx lo escribe
// igual después de haber configurado los otros. Un sketch que escriba sólo REFS no
// elige nada en esa placa: la referencia queda en lo que haya quedado de antes, y la
// misma lectura puede dar números distintos en corridas distintas.
//
// Medido en el clon del banco, barriendo los 256 valores de DACON, los 4 de REFS y
// cuatro bytes de calibración, con 256 conversiones por punto: lo único que mueve la
// lectura es REFS2. Con REFS2 en 0 A1 lee 1846 a 1853 cuentas (AVCC); con REFS2 en 1,
// A1 satura en 4095. DACON no cambia nada --al arrancar vale 3 y da la misma lectura
// que 0-- y REFS tampoco (A1 entre 1846,4 y 1853,3). Se lo escribe igual, que es lo que
// hace el core, pero lo que decide es REFS2.
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
// Atención: ese razonamiento vale en el ATmega y no en el clon. Ahí los bits REFS no
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

// El bandgap medido contra AVcc, en cuentas. Es la mitad de la calibración de la
// referencia: da la razón entre las dos tensiones, y la otra mitad --cuánto vale
// una de las dos en volts-- hay que medirla una vez con un multímetro, porque acá
// adentro no hay ninguna tensión conocida contra la cual calibrar.
//
// Con AVcc medido, la referencia interna vale AVcc * cuentas / fondo de escala. El error de ganancia que corrige no es
// chico: el bandgap está especificado entre 1,0 y 1,2 V, o sea +/-10 % de chip a
// chip, y va derecho a los miliamperes que se informan.
//
// En el clon el número NO significa eso, y conviene aclararlo explícitamente en
// lugar de dejar una cuenta que parece una calibración. Los bits REFS del
// ADMUX son los del ATmega y el LGT8F328P tiene su propio juego de referencias
// internas, así que esto mide una contra otra y no un bandgap contra AVcc. El valor
// además depende de con qué referencia venía trabajando el ADC: el mismo código, en un
// sketch que arranca de otra manera, da 2585. No sirve como calibración.
//
// Con el micro a 3,3 V la única referencia interna alcanzable es la de 1,024 V: en el
// barrido no hay ninguna combinación que dé las cuentas de 2,048 ni de 4,096 V. Medido
// contra un nodo interno fijo (canal 8), la razón entre la referencia baja y AVCC da
// 3,2745, o sea AVCC = 3,353 V con la baja en 1,024 V, que es lo que corresponde a esta
// placa. Esa medición es repetible --0,03 % entre reinicios-- pero no exacta: no hay
// adentro del micro ninguna tensión conocida contra la cual verificarla, y con el I2C
// del AS5600 corriendo se corre un 2,3 % y su desvío se multiplica por 25. Sirve para
// identificar la referencia, no para calibrar la escala de una medición.
//
// La referencia del clon se resuelve con el core lgt8fx, que la declara por
// nombre (INTERNAL1V024, INTERNAL2V048, INTERNAL4V096) en lugar de dejarla
// adivinar. Ningún sketch mide la corriente contra una referencia interna: ver
// Sense/RowAdc.h.
inline uint16_t adc_bandgap()
{
    return adc_once(_BV(REFS0) | 0x0E);
}

}  // namespace board
