// Comprobaciones de escritorio para los módulos que son aritmética pura: la tabla
// de calibración, el seguimiento de ángulo, la medición de corriente y el notch.
// Las del lazo y el PID van con la biblioteca Control, que no forma parte del TP2.
//
// Poder probarlos acá es una de las razones para tenerlos separados. Ninguno
// accede a un registro ni espera respuesta de otro componente: reciben números por update(), por
// corrected() o por step() y devuelven números, así que un error de signo o de
// redondeo se encuentra en un segundo en lugar de en un banco con un motor girando.
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
#include <cstdlib>

#include "AngleLut.h"
#include "AngleTracker.h"
#include "CurrentSense.h"
#include "WindowMean.h"
#include "SupplyRatio.h"
#include "MainsNotch.h"

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
    check(flat, "una tabla vacía no corrige en ninguna cuenta");

    // Una entrada sola, en octavos, sale en cuentas donde tiene que salir.
    lut.entry[0] = 8;                       // una cuenta entera
    check_eq(lut.correction(0), 1, "una entrada de 8 octavos corrige una cuenta");

    // A mitad de camino de una entrada en cero la interpolación da media cuenta, y el
    // redondeo es al medio hacia arriba: 1, no 0. Vale la pena fijarlo, porque es la
    // regla que la computadora tiene que reproducir para que las dos coincidan.
    check_eq(lut.correction(32), 1, "media cuenta redondea al medio hacia arriba");

    // Y del lado negativo la misma regla, que es donde el `e < 0 ? -4 : 4`, que
    // parece la opción natural, se equivoca: -3/8 tiene que dar 0 y no -1.
    lut.entry[0] = -3;
    lut.entry[1] = -3;
    check_eq(lut.correction(0), 0, "menos tres octavos redondean a cero, no a -1");

    // El recorte, y el caso que motivó que las entradas sean int16.
    lut.entry[1] = 32000;
    check(!lut.write_packed(((uint32_t)64 << 16) | 5),
          "un índice que no existe se rechaza");
    check(lut.write_packed(((uint32_t)1 << 16) | (uint32_t)(uint16_t)32000),
          "una entrada válida se acepta");
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
    check(parity, "la interpolación coincide con la referencia en las 4096 cuentas");
    if (!parity) { printf("       peor desvío: %d cuentas\n", worst); }

    // El checksum distingue dos entradas intercambiadas, que es el error que se
    // comete cargando una tabla y que una suma simple no detecta. Se prueban todos
    // los pares y no uno solo: el par (3,4) lo detectaba igual la versión de bytes
    // módulo 256, y los que se le escapaban eran los que están a distancia 32.
    const uint16_t before = lut.checksum();
    unsigned swaps = 0;
    unsigned blind = 0;
    for (unsigned i = 0; i < Lut::SIZE; i++)
    {
        for (unsigned j = i + 1; j < Lut::SIZE; j++)
        {
            if (lut.entry[i] == lut.entry[j]) { continue; }

            const Lut::Eighths tmp = lut.entry[i];
            lut.entry[i] = lut.entry[j];
            lut.entry[j] = tmp;

            swaps++;
            if (lut.checksum() == before) { blind++; }

            lut.entry[j] = lut.entry[i];
            lut.entry[i] = tmp;
        }
    }
    check(lut.checksum() == before, "la tabla quedó como estaba");
    check(blind == 0,
          "el checksum cambia con cualquier par de entradas intercambiadas");
    if (blind) { printf("       %u de %u intercambios no se detectan\n", blind, swaps); }

    // La media de la ventana con una suma cerca de 2^32/16: el camino corto
    // multiplica por 16 y suma medio divisor para redondear, y la guarda tiene que
    // descontar las dos cosas. Sin eso hay una ventana de 4080 valores en la que la
    // media sale 0 y la corriente publicada salta a -2048 cuentas.
    {
        SupplyRatio r;
        r.div_e4 = 0;
        r.apply();

        const uint32_t n = 32;
        bool bien = true;
        for (uint32_t s = 0xFFFFFFFFUL / 16UL - 4200UL; s <= 0xFFFFFFFFUL / 16UL + 8UL; s++)
        {
            // Sin divisor: la media de A0 llevada a la escala de AVCC, que es
            // monótona en la suma. Un salto a cero es el desborde.
            const uint16_t q4 = r.counts_q4(s, n, 0, 0);
            if (q4 == 0) { bien = false; break; }
        }
        check(bien, "la media no desborda con la suma pegada a 2^32/16");
    }

    // --------------------------------------------------------- ángulo desenrollado

    Tracker ang;

    check_eq(Tracker::wrapped_error(10, 4090), 16,
             "el camino corto cruza el cero en lugar de dar la vuelta");
    check_eq(Tracker::wrapped_error(4090, 10), -16, "y lo mismo para el otro lado");

    // Tres vueltas enteras hacia adelante, de a 100 cuentas: el desenrollado tiene
    // que sumar 3 * 4096 sin un solo salto, y con el mismo signo que la cuenta. El
    // signo no se invierte en la placa: se elige en la computadora.
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

    // La misma resta con la cuenta en dieciseisavos: el cero sigue en cuentas.
    cur.update_q4(2048L * 16);
    check_eq(cur.i, 0, "en dieciseisavos, la cuenta del cero da cero");

    cur.update_q4(2048L * 16 + 7);
    check_eq(cur.i, 7, "y siete dieciseisavos por encima dan siete");

    cur.update_q4(0);
    check_eq(cur.i, -32767 - 1, "una cuenta en cero satura en lugar de dar la vuelta");

    // ------------------------------------------------- el promedio de la corriente

    WindowMean win(3);

    // Una fila sola: el promedio de sus conversiones, redondeado. push() ya no lo
    // devuelve --lo calculaba en cada fila y nadie lo leía, y eran dos divisiones de
    // 32 bits-- así que se pide con mean().
    win.push(3 * 100 + 2, 3);
    check_eq(win.mean(), 101, "una fila da el promedio de sus conversiones");

    // Promedia conversiones, no filas: 10 conversiones en 200 pesan más que 2 en 0.
    WindowMean pesos(2);
    pesos.push(10 * 200, 10);
    pesos.push(0, 2);
    check_eq(pesos.mean(), 167, "una fila con más conversiones pesa más");

    // Las filas más antiguas salen de la ventana.
    win.push(3 * 100, 3);
    win.push(3 * 100, 3);
    win.push(3 * 400, 3);
    win.push(3 * 400, 3);
    win.push(3 * 400, 3);
    check_eq(win.mean(), 400, "después de rows filas la más antigua ya no cuenta");

    // Una ventana sin ninguna conversión --el ADC no corrió-- no tiene promedio, y
    // devuelve cero en lugar del último. Antes `mean` era un campo pegajoso que
    // guardaba el valor viejo; ahora se calcula cuando se pide, así que decir «no
    // hay» es más honesto que devolver algo de hace rato. Quien necesite distinguir
    // las dos cosas mira count(), que es lo que hace SupplyRatio.
    WindowMean vacia(1);
    vacia.push(4 * 250, 4);
    check_eq(vacia.mean(), 250, "una fila da su promedio");
    vacia.push(0, 0);
    check_eq((long)vacia.count(), 0L, "y una ventana sin conversiones se ve en count()");
    check_eq(vacia.mean(), 0, "y no tiene promedio");

    // Cambiar rows empieza de nuevo y recorta al rango.
    win.rows = 0;
    win.apply();
    check_eq(win.rows, 1, "rows en cero se recorta a una fila");
    win.push(2 * 7, 2);
    check_eq(win.mean(), 7, "y la ventana empieza de nuevo");
    win.rows = 200;
    win.apply();
    check_eq(win.rows, WindowMean::MAX_ROWS, "y por arriba a MAX_ROWS");

    // La ventana llena con el máximo de conversiones de la placa más rápida no
    // desborda: 32 filas de 91 conversiones de 4095.
    WindowMean llena(WindowMean::MAX_ROWS);
    for (int k = 0; k < 40; k++) { llena.push(91UL * 4095UL, 91); }
    check_eq(llena.mean(), 4095, "la ventana llena a fondo de escala no desborda");

    // Los totales de la ventana, que usa SupplyRatio.
    WindowMean tot(2);
    tot.push(1000UL, 10);
    tot.push(3000UL, 20);
    check_eq((long)tot.total(), 4000L, "la ventana suma las conversiones");
    check_eq((long)tot.count(), 30L, "y las cuenta");

    // ----------------------------------------- contra la alimentación del sensor

    SupplyRatio ratio;
    ratio.apply();
    // Sin divisor la escala la fija AVCC, que es el único camino en el que no se
    // cancela: 3150 cuentas contra AVCC son 3150 * 5006/5120 cuentas equivalentes.
    check_eq(ratio.counts(315000UL, 100UL, 0UL, 0UL), 3080,
             "sin divisor, la media de A0 llevada a la escala de AVCC");
    check_eq(ratio.counts(0UL, 0UL, 0UL, 0UL), 0, "sin conversiones, cero");

    ratio.div_e4 = 2817;        // 2 k / (5,1 k + 2 k)
    ratio.apply();
    check(ratio.active(), "con divisor, activo");
    check_eq(ratio.counts(314820UL, 100UL, 179220UL, 100UL), 1979,
             "A0/A1 medidos en el banco: 1979 cuentas equivalentes");
    check_eq(ratio.supply, 1792, "y A1 queda como lectura");
    check_eq(ratio.counts(200000UL, 100UL, 112700UL, 100UL), 2000,
             "el sensor en la mitad de su alimentación da 2000 cualquiera sea el divisor");
    check_eq(ratio.counts(200000UL, 100UL, 0UL, 100UL), 4095,
             "A1 en cero (divisor suelto) satura contra el fondo de escala");
    check_eq(ratio.counts(315000UL, 100UL, 0UL, 0UL), 3080,
             "sin conversiones de A1, la media de A0 contra AVCC");

    // Lo mismo en dieciseisavos, que es lo que publica el canal: la resolución que
    // counts() pierde al redondear.
    check_eq((long)ratio.counts_q4(314820UL, 100UL, 179220UL, 100UL), 31670L,
             "counts_q4 da los mismos 1979 con cuatro bits más abajo");
    check_eq(ratio.counts(314820UL, 100UL, 179220UL, 100UL),
             (int16_t)((ratio.counts_q4(314820UL, 100UL, 179220UL, 100UL) + 8) >> 4),
             "y counts() es counts_q4() redondeado");

    ratio.div_e4 = 20000;
    ratio.apply();
    check_eq(ratio.div_e4, 10000, "una relación mayor que 1 se acota");

    // ------------------------------------------------------------ el notch de la red

    MainsNotch notch;
    notch.apply(500.0f);
    check_eq(notch.active(), 0, "apagado por omisión");
    notch.harmonics = 1;
    notch.apply(500.0f);
    check_eq(notch.active(), 2, "dos notch por armónico");
    notch.harmonics = 7;
    notch.apply(500.0f);
    check_eq(notch.active(), 6, "tres armónicos, seis notch");
    notch.harmonics = 5;
    notch.apply(500.0f);
    check_eq(notch.active(), 4, "50 y 150 Hz sin el de 100: cuatro notch");
    notch.harmonics = 7;
    notch.apply(100.0f);
    check_eq(notch.active(), 1, "a 100 Hz de filas sólo entra el de 49,5");
    notch.nyquist = 1;
    notch.apply(500.0f);
    check_eq(notch.active(), 7, "y el del Nyquist es uno más");
    notch.nyquist = 0;

    notch.harmonics = 1;
    notch.apply(500.0f);
    int16_t dc = 0;
    for (int k = 0; k < 2000; k++) { dc = notch.step(1000); }
    check(dc >= 999 && dc <= 1001, "la continua pasa entera");

    // La red vista desde el clon, 49,68 Hz, con 400 cuentas de amplitud alrededor del
    // cero: los dos notch la bajan unos 40 dB sin haberla calibrado.
    notch.apply(250.0f);
    notch.apply(500.0f);
    int16_t pico = 0;
    for (int k = 0; k < 4000; k++)
    {
        const int16_t y = notch.step((int16_t)lround(400.0 * sin(2.0 * M_PI * 49.68 * k / 500.0)));
        if (k > 2000 && abs(y) > pico) { pico = (int16_t)abs(y); }
    }
    check(pico < 20, "49,68 Hz baja a menos de 20 cuentas de 400");

    // El rango: un escalón de -1500 a 1500 cuentas, el doble del fondo del ACS712 de
    // 5 A, sale entero sin saturar adentro.
    notch.harmonics = 7;
    notch.apply(500.0f);
    for (int k = 0; k < 1000; k++) { dc = notch.step(-1500); }
    for (int k = 0; k < 2000; k++) { dc = notch.step(1500); }
    check_eq(dc, 1500, "un escalón de 3000 cuentas llega entero, con la continua exacta");

    // Lo que agrega el redondeo adentro del filtro, sobre ruido blanco de ±2 cuentas.
    // Los seis notch sacan ~15 % de la varianza; en cuartos de cuenta el redondeo
    // devolvía casi todo eso (la salida quedaba en ~96 % de la entrada).
    srand(3);
    notch.apply(250.0f);
    notch.apply(500.0f);
    double var_in = 0.0, var_out = 0.0;
    for (int k = 0; k < 20000; k++)
    {
        const int16_t x = (int16_t)(rand() % 5 - 2);
        const int16_t y = notch.step(x);
        if (k >= 2000) { var_in += (double)x * x; var_out += (double)y * y; }
    }
    check(var_out < 0.9 * var_in, "sobre ruido blanco, el redondeo interno no devuelve lo que el notch saca");

    // El notch del Nyquist, solo: una fila sí y otra no --250 Hz con filas a 500 Hz--
    // desaparece, y la continua pasa exacta.
    MainsNotch nyq;
    nyq.nyquist = 1;
    nyq.apply(500.0f);
    int16_t alterna = 0;
    for (int k = 0; k < 400; k++)
    {
        const int16_t y = nyq.step((k & 1) ? 300 : -300);
        if (k > 100 && abs(y) > alterna) { alterna = (int16_t)abs(y); }
    }
    check(alterna <= 1, "el notch del Nyquist saca una fila sí y otra no");
    for (int k = 0; k < 200; k++) { dc = nyq.step(1500); }
    check_eq(dc, 1500, "y deja pasar la continua exacta");

    // En dieciseisavos, que es la unidad del canal: el lazo resuelve por debajo de la
    // cuenta, así que una alternancia de dos cuentas y media no sale en cero.
    MainsNotch fino;
    fino.nyquist = 1;
    fino.apply(500.0f);
    int16_t chico = 0;
    for (int k = 0; k < 400; k++)
    {
        const int16_t y = fino.step_q4((int16_t)((k & 1) ? 40 : -40));
        if (k > 200 && abs(y) > chico) { chico = (int16_t)abs(y); }
    }
    check(chico <= 4, "en dieciseisavos también saca la alternancia, sin trabarse");

    // Y a fondo de escala con las siete secciones, que es donde más grande se hace el
    // acumulador: la continua pasa sin dar la vuelta. Ver GUARD_BITS.
    MainsNotch tope;
    tope.harmonics = 7;
    tope.nyquist   = 1;
    tope.apply(500.0f);
    int16_t lleno = 0;
    for (int k = 0; k < 3000; k++) { lleno = tope.step_q4(INT16_MAX); }
    check(lleno >= INT16_MAX - 2,
          "a fondo de escala y con siete secciones la continua pasa sin desbordar");

    printf("\n%d falla(s)\n", fails);
    return fails ? 1 : 0;
}
