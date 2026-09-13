#pragma once

#include <Arduino.h>

// El bus I2C: destrabarlo, y decir que le pasa a sus dos cables.
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

// ------------------------------------------------- el estado eléctrico del bus

// Mientras TWEN esté puesto, SDA y SCL los gobierna el TWI y no el puerto, así que
// toda medición o maniobra a mano tiene que soltarlos primero y devolverlos
// después. bus_recover() ya lo hacía; lo que sigue también tiene que hacerlo.
//
// Y acá TWEN ya está puesto antes de que empiece setup(): nI2C construye su objeto
// global durante la inicialización estática y su constructor enciende el
// periférico. Olvidarlo no da un error: da mediciones mudas. El detector de cruce
// informó que no había cruce con los cables cruzados sobre la mesa, porque sus
// pulsos nunca salieron de los pines.
inline uint8_t release_twi(void)
{
    const uint8_t twcr = TWCR;
    TWCR = 0;
    return twcr;
}

inline void restore_twi(uint8_t twcr)
{
    TWCR = twcr;
}

// Qué le pasa a una línea del bus, antes de que el TWI la tome. Un sensor que no
// contesta tiene cuatro causas que se arreglan en lugares distintos, y desde el
// protocolo las cuatro se ven igual: silencio.
enum {
    LINE_OK       = 0,  // reposa arriba: hay pull-ups y están alimentados
    LINE_NO_VCC  = 1,  // reposa abajo pero hay algo colgado, sin alimentar
    LINE_FLOATING  = 2,  // no hay nada del otro lado del cable
    LINE_GROUNDED   = 3,  // sujeta contra masa: corto, o un esclavo trabado
};

// Y una quinta cosa, que no es de una línea sino de las dos: los cables cambiados
// entre sí. Va como bit aparte del byte que devuelve bus_check().
static const uint8_t BUS_SWAPPED = 0x10;

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
inline uint8_t line_state(uint8_t pin, uint16_t fondo)
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
        return LINE_OK;
    }
    if (tirada > (fondo / 20) * 17) {
        return LINE_FLOATING;
    }
    if (tirada > fondo / 5) {
        return LINE_NO_VCC;
    }
    return LINE_GROUNDED;
}

// Las dos líneas en un byte: SDA en los bits 0-1, SCL en los bits 2-3. Se llama
// una vez al arrancar, antes de encender el TWI y antes de configurar el ADC para
// el lazo, porque deja el multiplexor donde lo dejó analogRead().
inline uint8_t bus_check(uint16_t fondo)
{
    const uint8_t twcr = release_twi();
    const uint8_t sda  = line_state(SDA, fondo);
    const uint8_t scl  = line_state(SCL, fondo);
    restore_twi(twcr);
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

static const uint8_t SW_US = 8;   // medio período, unos 50 kHz: lento a propósito

inline void sw_release(uint8_t pin)
{
    pinMode(pin, INPUT_PULLUP);
}

inline void sw_pull_low(uint8_t pin)
{
    // Bajar el pin es soltar el pull-up antes de pasar a salida: al revés, entre
    // las dos instrucciones el pin queda en alto y le pelea al esclavo.
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
}

// Un byte por software, y el ACK que devuelve el esclavo. `sda` y `scl` son los
// pines que hacen de cada cosa, que es justamente lo que esta prueba intercambia.
inline bool sw_byte(uint8_t sda, uint8_t scl, uint8_t valor)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (valor & 0x80) {
            sw_release(sda);
        } else {
            sw_pull_low(sda);
        }
        valor <<= 1;
        delayMicroseconds(SW_US);
        sw_release(scl);
        delayMicroseconds(SW_US);
        sw_pull_low(scl);
        delayMicroseconds(SW_US);
    }

    sw_release(sda);                 // el noveno pulso lo maneja el esclavo
    delayMicroseconds(SW_US);
    sw_release(scl);
    delayMicroseconds(SW_US);
    const bool ack = (digitalRead(sda) == LOW);
    sw_pull_low(scl);
    delayMicroseconds(SW_US);
    return ack;
}

// Suelta el bus antes de preguntar, con los roles dados. Es la misma maniobra que
// bus_recover() --pulsos de reloj hasta que el esclavo largue la línea de datos,
// y un STOP para dejarlo en un estado conocido-- pero con los pines explícitos,
// porque acá se intercambian a propósito.
inline void sw_unwedge(uint8_t sda, uint8_t scl)
{
    sw_release(sda);
    sw_release(scl);
    delayMicroseconds(SW_US);

    for (uint8_t i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
        sw_pull_low(scl);
        delayMicroseconds(SW_US);
        sw_release(scl);
        delayMicroseconds(SW_US);
    }

    sw_pull_low(sda);                  // STOP
    delayMicroseconds(SW_US);
    sw_release(sda);
    delayMicroseconds(SW_US);
}

// START, dirección, STOP. Devuelve si alguien dio ACK.
//
// Suelta el bus antes de preguntar y se niega a preguntar sobre una línea que
// sigue abajo. Sin eso la prueba miente en el peor momento: un esclavo que quedó a
// mitad de camino sujeta su línea de datos, y una línea sujeta se lee exactamente
// igual que un ACK. Así fue como el detector de cruce informó que no había cruce
// justo cuando los cables estaban cruzados.
inline bool sw_probe(uint8_t sda, uint8_t scl, uint8_t addr)
{
    sw_unwedge(sda, scl);

    sw_release(sda);
    delayMicroseconds(SW_US);
    if (digitalRead(sda) == LOW) {
        pinMode(sda, INPUT);
        pinMode(scl, INPUT);
        return false;                // sujeta: no hay pregunta que hacer acá
    }

    sw_release(sda);
    sw_release(scl);
    delayMicroseconds(SW_US);
    sw_pull_low(sda);
    delayMicroseconds(SW_US);
    sw_pull_low(scl);
    delayMicroseconds(SW_US);

    const bool ack = sw_byte(sda, scl, (uint8_t)(addr << 1));

    sw_pull_low(sda);
    delayMicroseconds(SW_US);
    sw_release(scl);
    delayMicroseconds(SW_US);
    sw_release(sda);
    delayMicroseconds(SW_US);

    pinMode(sda, INPUT);
    pinMode(scl, INPUT);
    return ack;
}

// Si `addr` contesta con los dos cables cambiados de lugar. Se pregunta primero
// por el cableado derecho: si ahí contesta no hay nada que informar, y de paso no
// se le mandan pulsos raros a un bus que anda.
//
// Conviene llamarla antes de bus_recover(), que después deja el bus en un estado
// conocido por si esta sonda dejó a alguien a mitad de camino.
inline bool responds_swapped(uint8_t addr)
{
    const uint8_t twcr = release_twi();

    // Al derecho primero: si contesta ahí no hay nada que informar.
    const bool derecho = sw_probe(SDA, SCL, addr);
    const bool reves   = derecho ? false : sw_probe(SCL, SDA, addr);

    restore_twi(twcr);
    return reves;
}
}  // namespace board
