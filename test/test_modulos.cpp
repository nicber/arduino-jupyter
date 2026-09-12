// Comprobaciones de escritorio para los módulos que son aritmética pura: la tabla
// de calibración, el seguimiento de ángulo y la medición de corriente.
//
// Que se puedan probar acá es la mitad del punto de haberlos separado. Ninguno
// toca un registro ni pregunta nada a nadie: reciben números por update() o por
// corrected() y devuelven números, así que un error de signo o de redondeo se encuentra
// en un segundo en lugar de en un banco con un motor girando.
//
//   g++ -std=c++11 -O2 -Wall -Wextra \
//       -I ../libraries/Calibracion/src \
//       -I ../libraries/AngleSensor/src -I ../libraries/Sense/src \
//       test_modulos.cpp -o test_modulos && ./test_modulos
//
// Se mantiene compilando bajo C++11, que es con lo que compila el core del AVR.

#include <cstdio>
#include <cstdint>
#include <cmath>

#include "AngleLut.h"
#include "AngleTracker.h"
#include "CurrentSense.h"

static int fails = 0;

static void check(bool ok, const char* what)
{
    if (!ok) { printf("FALLA  %s\n", what); fails++; }
    else     { printf("PASA   %s\n", what); }
}

static void check_eq(long got, long want, const char* what)
{
    if (got != want)
    {
        printf("FALLA  %-52s dio %ld, esperaba %ld\n", what, got, want);
        fails++;
    }
    else { printf("PASA   %s\n", what); }
}

typedef AngleLut<4096, 64>     Lut;
typedef AngleTracker<4096>     Tracker;

// La misma interpolación que hace el notebook, escrita de nuevo a partir de la
// definición y no copiada del módulo: si las dos coinciden en todo el rango, la
// que está en la placa es la que la computadora cree que está.
static int reference_correction(const Lut& lut, int raw)
{
    const int span  = 4096 / 64;
    const int index = (raw / span) & 63;
    const int frac  = raw % span;

    const long a = lut.entry[index];
    const long b = lut.entry[(index + 1) & 63];

    long eighths = a * (span - frac) + b * frac;

    // División hacia abajo, que es lo que hace un corrimiento aritmético y lo que
    // hace numpy. Es justamente el punto donde una división hacia cero se
    // desviaría, y sólo para los negativos.
    long q = eighths / span;
    if (eighths % span != 0 && ((eighths < 0) != (span < 0))) { q--; }

    long e = q + 4;
    long r = e / 8;
    if (e % 8 != 0 && e < 0) { r--; }
    return (int)r;
}

int main()
{
    // ------------------------------------------------------- tabla de calibración

    Lut lut;

    check_eq(Lut::SPAN, 64, "la tabla cubre 64 cuentas por entrada");
    check_eq(Lut::SPAN_BITS, 6, "y el corrimiento equivalente son 6 bits");

    // Una tabla en cero no corrige nada, en ninguna parte de la vuelta.
    bool flat = true;
    for (int raw = 0; raw < 4096; raw++) { if (lut.correction(raw) != 0) flat = false; }
    check(flat, "una tabla vacia no corrige en ninguna cuenta");

    // Una entrada sola, en octavos, sale en cuentas donde tiene que salir.
    lut.entry[0] = 8;                       // una cuenta entera
    check_eq(lut.correction(0), 1, "una entrada de 8 octavos corrige una cuenta");

    // A mitad de camino de una entrada en cero la interpolación da media cuenta, y el
    // redondeo es al medio hacia arriba: 1, no 0. Vale la pena fijarlo, porque es la
    // regla que la computadora tiene que reproducir para que las dos coincidan.
    check_eq(lut.correction(32), 1, "media cuenta redondea al medio hacia arriba");

    // Y del lado negativo la misma regla, que es donde el `e < 0 ? -4 : 4` que uno
    // escribe de reflejo se equivoca: -3/8 tiene que dar 0 y no -1.
    lut.entry[0] = -3;
    lut.entry[1] = -3;
    check_eq(lut.correction(0), 0, "menos tres octavos redondean a cero, no a -1");

    // El recorte, y el caso que motivó que las entradas sean int16.
    lut.entry[1] = 32000;
    check(!lut.write_packed(((uint32_t)64 << 16) | 5),
          "un indice que no existe se rechaza");
    check(lut.write_packed(((uint32_t)1 << 16) | (uint32_t)(uint16_t)32000),
          "una entrada valida se acepta");
    check_eq(lut.entry[1], Lut::MAX, "y un valor fuera de rango queda recortado");

    // Paridad con la referencia sobre toda la vuelta, con una tabla de armónicos
    // que tiene valores de los dos signos: es donde el redondeo se desvía.
    for (int i = 0; i < 64; i++)
    {
        lut.entry[i] = (Lut::Eighths)(int)(800.0 * sin(2 * M_PI * i / 64.0)
                                         + 300.0 * sin(4 * M_PI * i / 64.0));
    }

    bool parity = true;
    int  worst  = 0;
    for (int raw = 0; raw < 4096; raw++)
    {
        const int mine = lut.correction(raw);
        const int ref  = reference_correction(lut, raw);
        if (mine != ref) { parity = false; if (abs(mine - ref) > worst) worst = abs(mine - ref); }
    }
    check(parity, "la interpolacion coincide con la referencia en las 4096 cuentas");
    if (!parity) { printf("       peor desvio: %d cuentas\n", worst); }

    // El checksum distingue dos entradas intercambiadas, que es el error que se
    // comete cargando una tabla y que una suma pelada no ve.
    const uint16_t before = lut.checksum();
    const Lut::Eighths tmp = lut.entry[3];
    lut.entry[3] = lut.entry[4];
    lut.entry[4] = tmp;
    check(lut.checksum() != before,
          "el checksum cambia si se intercambian dos entradas");

    // --------------------------------------------------------- ángulo desenrollado

    Tracker ang;

    check_eq(Tracker::wrapped_error(10, 4090), 16,
             "el camino corto cruza el cero en lugar de dar la vuelta");
    check_eq(Tracker::wrapped_error(4090, 10), -16, "y lo mismo para el otro lado");

    // Tres vueltas enteras hacia adelante, de a 100 cuentas: el desenrollado tiene
    // que sumar 3 * 4096 sin un solo salto, y con el mismo signo que la cuenta. El
    // signo no se da vuelta en la placa: se elige en la computadora.
    for (int k = 1; k <= 3 * 4096 / 100; k++) { ang.update((Tracker::Counts)((k * 100) % 4096)); }
    check(ang.y_uw > 3 * 4096 - 200 && ang.y_uw <= 3 * 4096,
          "tres vueltas desenrolladas dan tres vueltas, hacia arriba");

    Tracker atras;
    for (int k = 1; k <= 4096 / 100; k++) { atras.update((Tracker::Counts)((4096 - (k * 100) % 4096) % 4096)); }
    check(atras.y_uw < -4096 + 200 && atras.y_uw >= -4096,
          "y una vuelta para el otro lado da una vuelta negativa");

    // --------------------------------------------------------------- corriente

    CurrentSense cur(2048);

    cur.update(2048);
    check_eq(cur.i, 0, "la cuenta del cero da corriente cero");

    cur.update(2148);
    check_eq(cur.i, 100, "cien cuentas por encima del cero dan cien");

    cur.update(1948);
    check_eq(cur.i, -100, "y cien por debajo, menos cien");

    printf("\n%d falla(s)\n", fails);
    return fails ? 1 : 0;
}
