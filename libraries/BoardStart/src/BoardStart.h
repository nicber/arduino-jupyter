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
// El CLKPR con el que arrancó la placa, guardado antes de corregirlo. Sirve de
// segunda opinión para distinguir las dos placas del banco cuando hay que
// decidir algo más que el reloj; ver boardAdcFullScale(). Vale 0 si nadie llamó
// todavía a boardClockBegin(), que es el caso del UNO de todos modos.
inline uint8_t& boardResetClkpr()
{
    static uint8_t valor = 0;
    return valor;
}

inline void boardClockBegin()
{
    boardResetClkpr() = CLKPR;

    if (CLKPR != 0) {
        clock_prescale_set(clock_div_2);
    }
}

// ------------------------------------------------------------------- el ADC

// Una conversión suelta, con el multiplexor y la referencia que se le pidan, y
// dejando el ADC como estaba. Es para el arranque: adentro del lazo el ADC se
// maneja libre y sin esperar.
inline uint16_t boardAdcOnce(uint8_t admux)
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

// Cuántas cuentas da el ADC a fondo de escala: 1024 en el UNO, 4096 en el clon
// con LGT8F328P, que trae un ADC de 12 bits. La misma tensión mide cuatro veces
// más en una placa que en la otra, y en este banco las dos corren el mismo
// binario, así que el número no puede ser una constante compilada.
//
// La sonda no depende de ningún valor de tensión: mide el bandgap tomando como
// referencia el bandgap mismo. Entrada y referencia son la misma cosa, así que
// el resultado es el fondo de escala, valga el bandgap 1,0 o 1,2 V. Un ADC de 10
// bits satura en 1023 y no puede devolver más, de manera que cualquier cosa por
// encima de 1023 es una prueba y no un indicio.
//
// Si la sonda no concluye --una placa que no implemente el canal del bandgap
// devuelve cero-- se cae al mismo discriminador que usa el reloj, que es el
// CLKPR de arranque. Por eso conviene llamar a boardClockBegin() antes.
inline uint16_t boardAdcFullScale()
{
    const uint16_t saturado = boardAdcOnce((_BV(REFS1) | _BV(REFS0)) | 0x0E);

    if (saturado > 1023) {
        return 4096;
    }
    // Saturar en 1023 es lo que hace un UNO, pero también sería lo que haría una
    // placa de 12 bits cuya sonda midiera algo por debajo de la referencia. No se
    // adivina: el CLKPR de arranque ya distingue las dos placas del banco, y acá
    // contesta bien en los dos casos.
    return boardResetClkpr() ? 4096 : 1024;
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
inline uint16_t boardAdcBandgap()
{
    return boardAdcOnce(_BV(REFS0) | 0x0E);
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

// ------------------------------------------------- el estado eléctrico del bus

// Qué le pasa a una línea del bus, antes de que el TWI la tome. Un sensor que no
// contesta tiene cuatro causas que se arreglan en lugares distintos, y desde el
// protocolo las cuatro se ven igual: silencio.
enum {
    I2C_LINEA_OK       = 0,  // reposa arriba: hay pull-ups y están alimentados
    I2C_LINEA_SIN_VCC  = 1,  // reposa abajo pero hay algo colgado, sin alimentar
    I2C_LINEA_AL_AIRE  = 2,  // no hay nada del otro lado del cable
    I2C_LINEA_A_MASA   = 3,  // sujeta contra masa: corto, o un esclavo trabado
};

// Y una quinta cosa, que no es de una línea sino de las dos: los cables cambiados
// entre sí. Va como bit aparte del byte que devuelve i2cBusCheck().
static const uint8_t I2C_BUS_INVERTIDO = 0x10;

// Dos medidas por línea alcanzan para separar los cuatro casos, y las hace el
// propio ADC sobre A4 y A5 sin agregar un solo cable.
//
// La primera es la línea en reposo, sin ayuda. Un bus utilizable descansa arriba,
// sostenido por las resistencias de pull-up, que en las plaquetas comerciales de
// AS5600 viven adentro del módulo y cuelgan de su alimentación. Eso hace que el
// reposo delate dos cosas a la vez: si hay pull-ups y si el módulo tiene VCC.
//
// La segunda es la línea con el pull-up interno del AVR, que son unas decenas de
// kiloohms. Ahí se separa un cable que no llega a ningún lado --sube a fondo de
// escala-- de uno que termina en un chip sin alimentar, que se queda a mitad de
// camino porque las resistencias del módulo lo cargan contra un riel muerto.
//
// Los umbrales salen de medir este banco con las dos placas: un módulo alimentado
// reposa en 5,00 V; sin alimentación reposa en 0,71 V y con el pull-up interno
// llega a 3,00 V; un pin al aire llega a 5,00 V. Se dejan en fracciones del fondo
// de escala para que valgan también con un módulo de 3,3 V, que reposa en 0,66 del
// fondo.
inline uint8_t i2cLineaEstado(uint8_t pin, uint16_t fondo)
{
    pinMode(pin, INPUT);
    delay(2);
    const uint16_t suelta = analogRead(pin);

    pinMode(pin, INPUT_PULLUP);
    delay(2);
    const uint16_t tirada = analogRead(pin);

    pinMode(pin, INPUT);

    // Dividir antes de multiplicar: fondo * 17 desborda un uint16_t.
    if (suelta > (fondo / 20) * 11) {
        return I2C_LINEA_OK;
    }
    if (tirada > (fondo / 20) * 17) {
        return I2C_LINEA_AL_AIRE;
    }
    if (tirada > fondo / 5) {
        return I2C_LINEA_SIN_VCC;
    }
    return I2C_LINEA_A_MASA;
}

// Las dos líneas en un byte: SDA en los bits 0-1, SCL en los bits 2-3. Se llama
// una vez al arrancar, antes de encender el TWI y antes de configurar el ADC para
// el lazo, porque deja el multiplexor donde lo dejó analogRead().
inline uint8_t i2cBusCheck(uint16_t fondo)
{
    const uint8_t sda = i2cLineaEstado(SDA, fondo);
    const uint8_t scl = i2cLineaEstado(SCL, fondo);
    return (uint8_t)(sda | (scl << 2));
}

// ------------------------------------------------- SDA y SCL cambiados de lugar

// Es la falla que sobrevive a todas las verificaciones anteriores. El bus se ve
// perfecto --hay pull-ups en las dos líneas, las dos reposan arriba y las dos se
// dejan bajar--, el módulo tiene alimentación, los cables llegan, y no contesta
// nadie, porque cada mensaje sale por el cable equivocado. Desde el protocolo es
// indistinguible de un módulo quemado, y se arregla dando vuelta dos fichas.
//
// La prueba hay que hacerla por software, moviendo los pines a mano: el TWI sólo
// sabe hablar por donde está cableado. Y no puede dar un falso positivo, porque un
// esclavo bien conectado no tiene manera de contestar así: lo que para nosotros es
// un START --los datos que bajan con el reloj arriba-- para él es su propio reloj
// bajando, que no abre ninguna transferencia.

static const uint8_t I2C_SW_US = 8;   // medio período, unos 50 kHz: lento a propósito

inline void i2cSwSuelta(uint8_t pin)
{
    pinMode(pin, INPUT_PULLUP);
}

inline void i2cSwBaja(uint8_t pin)
{
    // Bajar el pin es soltar el pull-up antes de pasar a salida: al revés, entre
    // las dos instrucciones el pin queda en alto y le pelea al esclavo.
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
}

// Un byte por software, y el ACK que devuelve el esclavo. `sda` y `scl` son los
// pines que hacen de cada cosa, que es justamente lo que esta prueba intercambia.
inline bool i2cSwByte(uint8_t sda, uint8_t scl, uint8_t valor)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (valor & 0x80) {
            i2cSwSuelta(sda);
        } else {
            i2cSwBaja(sda);
        }
        valor <<= 1;
        delayMicroseconds(I2C_SW_US);
        i2cSwSuelta(scl);
        delayMicroseconds(I2C_SW_US);
        i2cSwBaja(scl);
        delayMicroseconds(I2C_SW_US);
    }

    i2cSwSuelta(sda);                 // el noveno pulso lo maneja el esclavo
    delayMicroseconds(I2C_SW_US);
    i2cSwSuelta(scl);
    delayMicroseconds(I2C_SW_US);
    const bool ack = (digitalRead(sda) == LOW);
    i2cSwBaja(scl);
    delayMicroseconds(I2C_SW_US);
    return ack;
}

// START, dirección, STOP. Devuelve si alguien dio ACK.
inline bool i2cSwSonda(uint8_t sda, uint8_t scl, uint8_t addr)
{
    i2cSwSuelta(sda);
    i2cSwSuelta(scl);
    delayMicroseconds(I2C_SW_US);
    i2cSwBaja(sda);
    delayMicroseconds(I2C_SW_US);
    i2cSwBaja(scl);
    delayMicroseconds(I2C_SW_US);

    const bool ack = i2cSwByte(sda, scl, (uint8_t)(addr << 1));

    i2cSwBaja(sda);
    delayMicroseconds(I2C_SW_US);
    i2cSwSuelta(scl);
    delayMicroseconds(I2C_SW_US);
    i2cSwSuelta(sda);
    delayMicroseconds(I2C_SW_US);

    pinMode(sda, INPUT);
    pinMode(scl, INPUT);
    return ack;
}

// Si `addr` contesta con los dos cables cambiados de lugar. Se pregunta primero
// por el cableado derecho: si ahí contesta no hay nada que informar, y de paso no
// se le mandan pulsos raros a un bus que anda.
//
// Conviene llamarla antes de i2cBusRecover(), que después deja el bus en un estado
// conocido por si esta sonda dejó a alguien a mitad de camino.
inline bool i2cRespondeInvertido(uint8_t addr)
{
    if (i2cSwSonda(SDA, SCL, addr)) {
        return false;
    }
    return i2cSwSonda(SCL, SDA, addr);
}
