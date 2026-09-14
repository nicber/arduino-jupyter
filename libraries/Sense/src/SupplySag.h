// La compensación de la caída de la referencia del ADC mientras el transistor
// conduce.
//
// Medido en el banco: la corriente de base del transistor sale del pin 9 (unos
// 18 mA con 220 ohm), la alimentación del micro cae 91 mV mientras el pin está en
// alto, y como el ADC mide contra ella, todo lo que lee sube un 2,2 %. Después de
// cada apagado queda además una cola que se recupera en unos 150 us. Promediado
// sobre una fila, el error relativo es casi una recta en el ciclo de trabajo:
//
//   e(D) = fixed · [0 < D < 1] + slope · D
//
// con fixed ≈ 0,25 % (la cola, que al 100 % no existe porque no hay apagado) y
// slope ≈ 2,2 %, medidos con la fuente del motor apagada. Como el error es
// proporcional a la lectura entera --cero incluido--, se corrige la suma de la fila
// antes de promediarla y antes de restar el cero.
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

    static const uint32_t PARTS = 10000;

    uint16_t fixed;     // partes por 10000 mientras el PWM conmuta
    uint16_t slope;     // partes por 10000 con el pin siempre en alto

    constexpr SupplySag()
        : fixed(0)
        , slope(0)
    {
    }

    // La suma de una fila corregida, con `duty` de `full` el ciclo que estuvo
    // aplicado durante esa fila.
    uint32_t correct(uint32_t sum, uint16_t duty, uint16_t full) const
    {
        if (duty > full)
        {
            duty = full;
        }

        const uint32_t e = ((duty > 0 && duty < full) ? fixed : 0u)
                         + (uint32_t)slope * duty / full;

        // sum * (1 - e/PARTS) sin pasar por sum * e, que con filas largas desborda:
        // se parte la suma en cociente y resto de PARTS.
        return sum - (sum / PARTS) * e - ((sum % PARTS) * e + PARTS / 2) / PARTS;
    }
};

#endif  // SENSE_SUPPLYSAG_H
