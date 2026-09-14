// La compensación de la caída de la referencia del ADC mientras el transistor
// conduce.
//
// Medido en el banco: la corriente de base del transistor sale del pin 9 (unos
// 18 mA con 220 ohm), la alimentación del micro cae mientras el pin está en alto, y
// como el ADC mide contra ella todo lo que lee sube, hasta un ~2 % a fondo. Como el
// error es proporcional a la lectura entera --cero incluido--, se corrige la suma de
// la fila antes de promediarla y antes de restar el cero.
//
// La forma del error contra el ciclo de trabajo no es una recta: una recta con un
// escalón en D = 1 dejaba 10 a 40 mA de error. Así que es una tabla de SIZE puntos
// medidos, en partes por 10000, interpolada linealmente: los primeros SIZE - 1 en
// duty = 0, 16, 32, ..., 240, y el último en duty = full, el pin siempre en alto. Con
// el sensor fuera del circuito se midió que el error depende del ciclo de trabajo y
// no de la corriente del motor: con la fuente del motor apagada, con el eje libre y
// con el eje trabado da lo mismo a pocos mA, así que la tabla se calibra con la
// fuente apagada (`calibrar_caida()` del lado de Python).
//
// Se carga como la tabla del ángulo: una entrada por escritura, (índice << 16) |
// valor, y una suma de Fletcher que verifica las SIZE con una lectura.
//
// Medir contra la referencia interna del LGT8F328P no resolvía esto: con el I2C
// del AS5600 corriendo, esa lectura se corre un 6 % y su ruido se multiplica por 14.
// La placa, en cambio, ya sabe cuándo el pin 9 está en alto: es el comando.
//
// Aritmética pura, así que se prueba en la máquina de escritorio.

#ifndef SENSE_SUPPLYSAG_H
#define SENSE_SUPPLYSAG_H

#include <stdint.h>

class SupplySag
{
    public:

    static const uint32_t PARTS    = 10000;
    static const uint8_t  SIZE     = 17;
    static const uint8_t  STEP_BITS = 4;           // un punto cada 16 cuentas de duty
    static const uint16_t MAX_ENTRY = 2000;        // 20 %: más que eso es un error
    static const uint32_t NOTHING  = 0xFFFFFFFFUL;

    // Partes por 10000 en cada punto. entry[SIZE - 1] es el pin siempre en alto.
    uint16_t entry[SIZE];

    SupplySag()
        : m_applied(NOTHING)
    {
        for (uint8_t k = 0; k < SIZE; k++)
        {
            entry[k] = 0;
        }
    }

    // El error de la fila, en partes por 10000, con `duty` de `full` aplicado. `full`
    // tiene que ser mayor que el último punto intermedio, 240: el comando es de 8 bits.
    uint16_t error(uint16_t duty, uint16_t full) const
    {
        if (duty >= full)
        {
            return entry[SIZE - 1];
        }

        uint8_t k = (uint8_t)(duty >> STEP_BITS);
        if (k > SIZE - 2)
        {
            k = SIZE - 2;
        }

        const uint16_t lo   = (uint16_t)k << STEP_BITS;
        const uint16_t hi   = (k + 1 < SIZE - 1) ? (uint16_t)((k + 1) << STEP_BITS) : full;
        const int32_t  a    = entry[k];
        const int32_t  b    = entry[k + 1];
        const int32_t  span = (int32_t)(hi - lo);
        const int32_t  off  = (int32_t)(duty - lo);

        // Redondeado al más cercano, con el signo de la diferencia.
        const int32_t num = (b - a) * off;
        const int32_t e = a + (num >= 0 ? (num + span / 2) / span : -((-num + span / 2) / span));
        return (uint16_t)(e < 0 ? 0 : e);
    }

    // La suma de una fila corregida, con `duty` de `full` el ciclo que estuvo
    // aplicado durante esa fila.
    uint32_t correct(uint32_t sum, uint16_t duty, uint16_t full) const
    {
        const uint32_t e = error(duty, full);

        // sum * (1 - e/PARTS) sin pasar por sum * e, que con filas largas desborda:
        // se parte la suma en cociente y resto de PARTS.
        return sum - (sum / PARTS) * e - ((sum % PARTS) * e + PARTS / 2) / PARTS;
    }

    // Suma de Fletcher de 16 bits sobre la tabla, byte bajo primero: la misma que
    // AngleLut::checksum(), para que la computadora la reproduzca igual.
    uint16_t checksum(void) const
    {
        uint8_t a = 0;
        uint8_t b = 0;

        for (uint8_t k = 0; k < SIZE; k++)
        {
            a += (uint8_t)entry[k];
            b += a;
            a += (uint8_t)(entry[k] >> 8);
            b += a;
        }

        return (uint16_t)(((uint16_t)b << 8) | a);
    }

    // Una entrada por escritura, empaquetada como (índice << 16) | valor. Devuelve
    // false para un índice que no existe. Ver AngleLut::write_packed().
    bool write_packed(uint32_t packed)
    {
        const uint16_t index = (uint16_t)(packed >> 16);

        if (index >= SIZE)
        {
            return false;
        }

        uint16_t value = (uint16_t)packed;
        if (value > MAX_ENTRY)
        {
            value = MAX_ENTRY;
        }

        entry[index] = value;
        return true;
    }

    // Lo mismo, sólo si el parámetro cambió desde la última vez.
    bool apply(uint32_t packed)
    {
        if (packed == m_applied)
        {
            return false;
        }

        m_applied = packed;
        return write_packed(packed);
    }

    private:

    uint32_t m_applied;
};

#endif  // SENSE_SUPPLYSAG_H
