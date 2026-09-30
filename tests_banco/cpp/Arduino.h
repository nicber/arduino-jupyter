// Un Arduino.h simulado para probar en la PC los módulos que tocan registros.
//
// Los registros son variables globales y las funciones de pines anotan lo que se
// les pidió en un registro de eventos, para poder verificar el orden de las
// escrituras (que un cambio de sentido apague el puente antes de mover IN1/IN2).

#ifndef ARDUINO_H_SIMULADO
#define ARDUINO_H_SIMULADO

#include <stdint.h>

#define F_CPU 16000000UL
#define _BV(bit) (1u << (bit))

#define OUTPUT 1
#define LOW    0
#define HIGH   1

// Timer1 (el PWM del actuador)
extern uint8_t  TCCR1A, TCCR1B;
extern uint16_t TCNT1, ICR1, OCR1A;
#define WGM11  1
#define COM1A1 7
#define WGM13  4
#define CS10   0

// Timer2 (el muestreador)
extern uint8_t TCCR2A, TCCR2B, OCR2A, TCNT2, TIMSK2;
#define WGM21  1
#define CS21   1
#define CS20   0
#define OCIE2A 1

// Lo que el código pide a los pines, en orden.
struct EventoPin { char que; uint8_t pin; uint8_t valor; uint8_t tccr1a; };
extern EventoPin g_eventos[64];
extern int       g_n_eventos;

extern uint32_t g_micros;

void pinMode(uint8_t pin, uint8_t modo);
void digitalWrite(uint8_t pin, uint8_t valor);
inline uint32_t micros(void) { return g_micros; }
inline uint32_t millis(void) { return g_micros / 1000; }
inline void noInterrupts(void) {}
inline void interrupts(void) {}

#endif
