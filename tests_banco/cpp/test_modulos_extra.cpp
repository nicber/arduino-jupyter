// Pruebas de escritorio de lo que test/test_modulos.cpp no cubre: el actuador
// (HBridge, con bidir), el reloj del muestreo (SampleClock), la salud del sensor
// (SensorHealth), el límite del desenrollado (AngleTracker a 5 kHz contra por fila)
// y propiedades de WindowMean y SupplyRatio sobre todo su rango.
//
// No va en el zip del TP2. Lo compila y lo corre `python tests_banco/correr.py`, con
// -I a libraries/{Actuator,Sampler,AngleSensor,Sense}/src y a esta carpeta.
//
// Arduino.h de esta carpeta reemplaza al del core: registros como variables y pines
// que anotan lo que se les pide.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "Arduino.h"
#include "HBridge.h"
#include "SampleClock.h"
#include "SensorHealth.h"
#include "AngleTracker.h"
#include "WindowMean.h"
#include "SupplyRatio.h"

// ------------------------------------------------------- el Arduino simulado
uint8_t  TCCR1A, TCCR1B;
uint16_t TCNT1, ICR1, OCR1A;
uint8_t  TCCR2A, TCCR2B, OCR2A, TCNT2, TIMSK2;
EventoPin g_eventos[64];
int       g_n_eventos = 0;
uint32_t  g_micros = 0;

void pinMode(uint8_t pin, uint8_t modo)
{
    if (g_n_eventos < 64) { g_eventos[g_n_eventos++] = { 'm', pin, modo, TCCR1A }; }
}

void digitalWrite(uint8_t pin, uint8_t valor)
{
    if (g_n_eventos < 64) { g_eventos[g_n_eventos++] = { 'w', pin, valor, TCCR1A }; }
}

// ------------------------------------------------------------------ el informe
static int fallas = 0;

static void check(bool ok, const char* que)
{
    if (!ok) { printf("FALLA  %s\n", que); fallas++; }
    else     { printf("PASA   %s\n", que); }
}

static void check_eq(long dio, long esperaba, const char* que)
{
    if (dio != esperaba) { printf("FALLA  %-60s dio %ld, esperaba %ld\n", que, dio, esperaba); fallas++; }
    else                 { printf("PASA   %s\n", que); }
}

// Un sensor para SensorHealth::refresh_mounting(): contesta o no, y anota qué
// registro se le pidió.
struct SensorFalso
{
    static const uint8_t REG_STATUS = 0x0B, REG_AGC = 0x1A, REG_MAGNITUDE_H = 0x1B;
    static bool    contesta;
    static uint8_t pedido;
    static bool present(void) { return contesta; }
    static bool read_registers(uint8_t reg, uint8_t* buf, uint8_t n)
    {
        pedido = reg;
        buf[0] = (reg == REG_MAGNITUDE_H) ? 0xF7 : (uint8_t)(reg + 1);
        if (n > 1) { buf[1] = 0x08; }
        return true;
    }
};
bool    SensorFalso::contesta = true;
uint8_t SensorFalso::pedido   = 0;

static bool pwm_conectado(void) { return (TCCR1A & _BV(COM1A1)) != 0; }

static int nivel(uint8_t pin)
{
    int v = -1;
    for (int k = 0; k < g_n_eventos; k++) { if (g_eventos[k].que == 'w' && g_eventos[k].pin == pin) v = g_eventos[k].valor; }
    return v;
}

typedef HBridge<9, 6, 7> Puente;

int main()
{
    // ------------------------------------------------------------ HBridge
    const uint16_t TOP = 7619;
    Puente p(TOP);
    p.begin();

    check_eq(ICR1, TOP, "begin() pone el TOP del Timer1");
    check(!pwm_conectado(), "y arranca con el PWM desconectado: el puente abierto");
    check_eq(p.bidir, 0, "arranca en un solo cuadrante");

    p.write(128);
    check_eq(p.u, 128, "un comando positivo sale tal cual");
    check_eq(OCR1A, (long)((128UL * TOP) >> 8), "y el ciclo es comando * TOP / 256");
    check(pwm_conectado(), "con el PWM conectado");

    p.write(400);
    check_eq(p.u, 255, "por arriba de 255 se recorta a 255");
    check_eq(OCR1A, TOP, "y 255 es encendido permanente, no 255/256");

    p.write(-120);
    check_eq(p.u, 0, "con bidir = 0 un comando negativo sale como cero");
    check(!pwm_conectado(), "y deja el puente abierto");

    p.write(0);
    check(!pwm_conectado(), "un comando en cero abre el puente");

    // Con puente: el sentido cambia sin atravesar un estado conduciendo.
    Puente q(TOP);
    q.begin();
    q.bidir = 1;
    q.write(200);
    g_n_eventos = 0;
    q.write(-200);
    check_eq(q.u, -200, "con bidir = 1 un comando negativo sale negativo");
    check_eq(nivel(7), HIGH, "e IN2 queda en alto");
    check_eq(nivel(6), LOW, "e IN1 en bajo");
    bool abierto_antes = true;
    for (int k = 0; k < g_n_eventos; k++) { if (g_eventos[k].que == 'w' && (g_eventos[k].tccr1a & _BV(COM1A1))) abierto_antes = false; }
    check(abierto_antes, "cada escritura de IN1/IN2 al invertir pasa con el PWM desconectado");
    check(pwm_conectado(), "y el PWM vuelve recién después");
    q.write(-400);
    check_eq(q.u, -255, "y por abajo se recorta a -255");

    // --------------------------------------------------------- SampleClock
    SampleClock reloj(10);
    reloj.begin(5000);
    check_eq(OCR2A, 99, "5 kHz con preescalador /32: TOP de 8 bits = 99");
    int filas = 0;
    for (int k = 0; k < 100; k++) { if (reloj.on_isr()) { filas++; reloj.take(); } }
    check_eq(filas, 10, "loop_div = 10: una fila cada diez muestras");
    check_eq(reloj.missed, 0, "sin perder ninguna si el lazo las atiende");

    for (int k = 0; k < 30; k++) { reloj.on_isr(); }       // tres filas sin atender
    reloj.take();
    check_eq(reloj.missed, 2, "tres filas vencidas y una atendida cuentan dos perdidas");

    reloj.divide = 0;
    check_eq((long)reloj.apply(5000), 200, "divide = 0 se recorta a 1: filas cada 200 us");
    reloj.divide = 50;
    check_eq((long)reloj.apply(5000), 10000, "divide = 50: filas cada 10 ms");
    reloj.clear_health();
    check_eq(reloj.missed + reloj.late, 0, "clear_health() pone la salud en cero");

    // apply() corre con cada `set`, el del escalón incluido: con el divisor igual no
    // puede reiniciar la cuenta, o estiraría el período en el que cae el `set`.
    SampleClock fase(10);
    fase.begin(5000);
    int k_fila = 0;
    for (int k = 1; k <= 14; k++) { if (fase.on_isr()) { fase.take(); } }
    fase.apply(5000);
    for (int k = 15; k <= 30 && !k_fila; k++) { if (fase.on_isr()) { k_fila = k; } }
    check_eq(k_fila, 20, "apply() con el mismo divisor no corre la fase de las filas");
    fase.take();
    fase.divide = 4;
    fase.apply(5000);
    k_fila = 0;
    for (int k = 21; k <= 30 && !k_fila; k++) { if (fase.on_isr()) { k_fila = k; } }
    check_eq(k_fila, 24, "con un divisor nuevo la cuenta arranca de cero");
    fase.take();

    // La ventana: se limpia al abrir una captura y se congela al cerrarla.
    for (int k = 0; k < 12; k++) { fase.on_isr(); }         // tres filas, una atendida
    fase.take();
    fase.window(true);
    check_eq(fase.missed, 0, "window() limpia la salud al abrir la ventana");
    for (int k = 0; k < 8; k++) { fase.on_isr(); }          // dos filas, una atendida
    fase.take();
    check_eq(fase.missed, 1, "y adentro de la ventana cuenta");
    fase.window(false);
    for (int k = 0; k < 8; k++) { fase.on_isr(); }
    fase.take();
    check_eq(fase.missed, 1, "al cerrarla se congela en lo que describió la captura");
    fase.window(true);
    check_eq(fase.missed, 0, "y la ventana siguiente arranca de cero");

    // -------------------------------------------------------- SensorHealth
    SensorHealth salud;
    salud.accumulate(10, 2, true);
    salud.accumulate(15, 2, true);
    check_eq(salud.overruns, 15, "acumula los desbordes contando desde el contador del sensor");
    salud.overruns = 0;                                    // la computadora lo pone en cero
    salud.accumulate(18, 3, false);
    check_eq(salud.overruns, 3, "y después de ponerlo en cero cuenta sólo lo nuevo");
    check_eq(salud.errors, 3, "igual los errores");
    check_eq(salud.present, 0, "y sabe si el sensor dejó de contestar");
    salud.accumulate(0xFFFF, 3, true);
    salud.accumulate(2, 3, true);
    check_eq(salud.overruns, (long)(3 + (0xFFFF - 18) + 3), "el contador del sensor puede dar la vuelta");
    salud.accumulate(2 + 600, 3, true);
    check_eq(salud.overruns, 0xFFFF, "y el total publicado satura en lugar de dar la vuelta");

    // El montaje: un registro por refresco, la primera vuelta al ritmo rápido y
    // después al lento.
    SensorHealth montaje;
    SensorFalso::pedido = 0;
    montaje.refresh_mounting<SensorFalso>(1000);
    check(SensorFalso::pedido == SensorFalso::REG_STATUS && montaje.status == 0x0C,
          "el primer refresco vence enseguida y lee el estado");
    SensorFalso::pedido = 0;
    montaje.refresh_mounting<SensorFalso>(1020);
    check_eq(SensorFalso::pedido, 0, "el siguiente no antes de 50 ms");
    montaje.refresh_mounting<SensorFalso>(1060);
    check(SensorFalso::pedido == SensorFalso::REG_AGC && montaje.agc == 0x1B,
          "después de 50 ms sí, y lee la ganancia");
    montaje.refresh_mounting<SensorFalso>(1120);
    check_eq(montaje.magnitude, 0x708, "el módulo sale de dos bytes, en 12 bits");
    SensorFalso::pedido = 0;
    montaje.refresh_mounting<SensorFalso>(1300);
    check_eq(SensorFalso::pedido, 0, "completada una vuelta de registros, espera 500 ms");
    montaje.refresh_mounting<SensorFalso>(1650);
    check_eq(SensorFalso::pedido, SensorFalso::REG_STATUS, "y vence a los 500 ms");
    SensorFalso::contesta = false;
    montaje.refresh_mounting<SensorFalso>(2200);
    check(montaje.status == 0 && montaje.agc == 0 && montaje.magnitude == 0,
          "sin sensor en el bus olvida el montaje");

    // ------------------------------------------------- AngleTracker: el límite
    // Un eje a 700 rad/s (111 rev/s): desenrollado a 5 kHz sigue la vuelta; una vez por
    // fila a 100 Hz, media vuelta es 50 rev/s y lo lee plegado.
    const double w = 700.0, rev_s = w / (2 * M_PI);
    AngleTracker<4096> a5k, a100;
    long ultimo_5k = 0, ultimo_100 = 0;
    for (int k = 1; k <= 5000; k++)
    {
        const long cuenta = (long)floor(4096.0 * rev_s * k / 5000.0);
        a5k.update((AngleTracker<4096>::Counts)(cuenta & 4095));
        if (k % 50 == 0) { a100.update((AngleTracker<4096>::Counts)(cuenta & 4095)); }
        ultimo_5k = cuenta;
        if (k % 50 == 0) { ultimo_100 = cuenta; }
    }
    check(labs(a5k.y_uw - ultimo_5k) < 2048, "a 5 kHz, 700 rad/s se desenrollan bien (límite 15 700 rad/s)");
    check(labs(a100.y_uw - ultimo_100) > 2048, "una vez por fila a 100 Hz se pliegan (límite 314 rad/s)");

    // --------------------------------------------- WindowMean y SupplyRatio
    // WindowMean da el promedio exacto de las conversiones de las últimas `rows` filas.
    srand(7);
    WindowMean win(10);
    unsigned long sumas[40]; unsigned n[40];
    bool exacto = true;
    for (int f = 0; f < 40; f++)
    {
        n[f] = 40 + rand() % 10;
        sumas[f] = 0;
        for (unsigned c = 0; c < n[f]; c++) { sumas[f] += 3000 + rand() % 200; }
        win.push(sumas[f], (uint16_t)n[f]);
        const int16_t m = win.mean();
        unsigned long s = 0, cuantas = 0;
        for (int j = (f >= 9 ? f - 9 : 0); j <= f; j++) { s += sumas[j]; cuantas += n[j]; }
        if (m != (int16_t)((s + cuantas / 2) / cuantas)) exacto = false;
    }
    check(exacto, "WindowMean es el promedio de las conversiones de las últimas rows filas, siempre");

    // SupplyRatio: A0/A1 en cuentas equivalentes, a una cuenta de la cuenta exacta en
    // todo el rango, y sin moverse si las dos lecturas se corren juntas (AVCC que cae).
    SupplyRatio ratio;
    ratio.div_e4 = 2817;
    ratio.apply();
    const double k = 2817 / 10000.0;
    bool exacta = true, cancela = true, sin_desborde = true;
    for (unsigned long a0 = 400; a0 <= 4000; a0 += 97)
    {
        for (unsigned long a1 = 900; a1 <= 2600; a1 += 131)
        {
            const unsigned long n = 91;
            const double esperada = (double)a0 / a1 * 4000.0 * k;
            const int16_t c = ratio.counts(a0 * n, n, a1 * n, n);
            if (esperada < 4090 && fabs(c - esperada) > 1.0) exacta = false;
            // Y en dieciseisavos, que es lo que publica el canal: la misma cuenta
            // con cuatro bits más abajo.
            const uint16_t cq = ratio.counts_q4(a0 * n, n, a1 * n, n);
            // Dos dieciseisavos: uno del redondeo de la división y el resto del de
            // las medias y del m_scale_q4 truncado (11 ppm). Medido, el peor del
            // barrido es 1,18 dieciseisavos, contra 0,56 cuentas de counts().
            if (esperada < 4090 && fabs(cq / 16.0 - esperada) > 2.0 / 16) exacta = false;
            // La referencia cae un 2 %: A0 y A1 suben juntos.
            const int16_t c2 = ratio.counts((unsigned long)(a0 * n * 1.02), n,
                                            (unsigned long)(a1 * n * 1.02), n);
            if (esperada < 4090 && abs(c2 - c) > 1) cancela = false;
        }
    }
    // La ventana más grande: 32 filas de 91 conversiones a fondo, en los dos canales.
    const unsigned long nmax = 32UL * 91UL;
    if (ratio.counts(4095UL * nmax, nmax, 4095UL * nmax, nmax) != (int16_t)lround(4000.0 * k)) sin_desborde = false;
    if (ratio.counts(4095UL * nmax, nmax, 1UL * nmax, nmax) != 4095) sin_desborde = false;
    // El peor producto de counts_q4(): la media a fondo por la escala más grande.
    ratio.div_e4 = 10000;
    ratio.apply();
    if (ratio.counts_q4(4095UL * nmax, nmax, 4095UL * nmax, nmax) != 64000) sin_desborde = false;
    ratio.div_e4 = 2817;
    ratio.apply();
    check(exacta, "SupplyRatio da A0/A1 * 4000 * k a un dieciseisavo, en todo el rango");
    check(cancela, "y una caída de la referencia que corre A0 y A1 juntos no la mueve");
    check(sin_desborde, "con la ventana más grande a fondo no desborda, y un A1 chico satura");
    printf("\n%d falla(s)\n", fallas);
    return fallas ? 1 : 0;
}
