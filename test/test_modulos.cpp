// Comprobaciones de escritorio para los módulos que son aritmética pura: la tabla
// de calibración, el seguimiento de ángulo, la medición de corriente y el PID.
//
// Que se puedan probar acá es la mitad del punto de haberlos separado. Ninguno
// toca un registro ni pregunta nada a nadie: reciben números por update(), por
// corrected() o por step() y devuelven números, así que un error de signo o de
// redondeo se encuentra en un segundo en lugar de en un banco con un motor girando.
//
//   g++ -std=c++11 -O2 -Wall -Wextra \
//       -I ../libraries/ControlMath/src -I ../libraries/Calibracion/src \
//       -I ../libraries/AngleSensor/src -I ../libraries/Sense/src \
//       -I ../libraries/Control/src \
//       test_modulos.cpp -o test_modulos && ./test_modulos
//
// Se mantiene compilando bajo C++11, que es con lo que compila el core del AVR.

#include <cstdio>
#include <cstdint>
#include <cmath>

#include "FixedPoint.h"
#include "FirstOrderFilter.h"
#include "AngleLut.h"
#include "AngleTracker.h"
#include "CurrentSense.h"
#include "WindowMean.h"
#include "SupplySag.h"
#include "LoopAngle.h"
#include "LoopCurrent.h"
#include "Pid.h"

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

    // Lo que lee un lazo: referido a `offset` y con el signo del eje, que es el
    // contrario al del imán.
    LoopAngle<4096> lazo;
    lazo.offset = 1000;
    lazo.update(1000);
    check_eq(lazo.track.y, 0, "la cuenta de offset se lee como cero");
    lazo.update(1100);
    check_eq(lazo.track.y_uw, -100, "y cien cuentas mas del iman son cien menos del eje");
    check_eq(lazo.y_uwf, -100, "sin set_alpha() el filtro deja pasar la posicion");

    // Con el ángulo ya desenrollado, un período del lazo puede abarcar varias vueltas
    // sin que se pierdan: update() desenrollaría media vuelta como mucho.
    LoopAngle<4096> rapido;
    rapido.offset = 1000;
    rapido.update_unwrapped(1000);
    check_eq(rapido.track.y_uw, 0, "desenrollado afuera: la cuenta de offset es cero");
    rapido.update_unwrapped(1000 + 3 * 4096 + 100);
    check_eq(rapido.track.y_uw, -(3L * 4096 + 100),
             "y tres vueltas en un periodo son tres vueltas, con el signo del eje");
    check_eq(rapido.track.y, -100, "adentro de la vuelta, las mismas cien cuentas");

    lazo.set_alpha(LoopAngle<4096>::Alpha::from_float(0.1f));
    lazo.update(1000);
    check(lazo.y_uwf < -50, "y con alpha chico la posicion filtrada llega despues");

    // --------------------------------------------------------------- corriente

    CurrentSense cur(2048);

    cur.update(2048);
    check_eq(cur.i, 0, "la cuenta del cero da corriente cero");

    cur.update(2148);
    check_eq(cur.i, 100, "cien cuentas por encima del cero dan cien");

    cur.update(1948);
    check_eq(cur.i, -100, "y cien por debajo, menos cien");

    // Lo que lee un lazo: el cero primero y el signo despues.
    LoopCurrent lcur(2048);
    lcur.update(2148);
    check_eq(lcur.i, 100, "el lazo lee la misma corriente sin invert");
    lcur.invert = 1;
    lcur.update(2148);
    check_eq(lcur.i, -100, "y con el sensor invertido, menos cien");
    check_eq(lcur.sense.i, 100, "sin tocar lo que mide el sensor");

    // ------------------------------------------------- el promedio de la corriente

    WindowMean win(3);

    // Una fila sola: el promedio de sus conversiones, redondeado.
    check_eq(win.push(3 * 100 + 2, 3), 101, "una fila da el promedio de sus conversiones");

    // Promedia conversiones, no filas: 10 conversiones en 200 pesan más que 2 en 0.
    WindowMean pesos(2);
    pesos.push(10 * 200, 10);
    check_eq(pesos.push(0, 2), 167, "una fila con mas conversiones pesa mas");

    // Las filas viejas salen de la ventana.
    win.push(3 * 100, 3);
    win.push(3 * 100, 3);
    win.push(3 * 400, 3);
    win.push(3 * 400, 3);
    check_eq(win.push(3 * 400, 3), 400, "despues de rows filas la vieja ya no cuenta");

    // Una fila sin conversiones --el ADC no corrió-- no inventa un cero.
    WindowMean vacia(1);
    vacia.push(4 * 250, 4);
    check_eq(vacia.push(0, 0), 250, "una fila vacia deja el ultimo promedio");

    // Cambiar rows empieza de nuevo y recorta al rango.
    win.rows = 0;
    win.apply();
    check_eq(win.rows, 1, "rows en cero se recorta a una fila");
    check_eq(win.push(2 * 7, 2), 7, "y la ventana empieza de nuevo");
    win.rows = 200;
    win.apply();
    check_eq(win.rows, WindowMean::MAX_ROWS, "y por arriba a MAX_ROWS");

    // La ventana llena con el máximo de conversiones de la placa más rápida no
    // desborda: 32 filas de 91 conversiones de 4095.
    WindowMean llena(WindowMean::MAX_ROWS);
    int16_t ultimo = 0;
    for (int k = 0; k < 40; k++) { ultimo = llena.push(91UL * 4095UL, 91); }
    check_eq(ultimo, 4095, "la ventana llena a fondo de escala no desborda");

    // ----------------------------------------------- la caída de la referencia

    SupplySag sag;
    check_eq(sag.correct(300000UL, 128, 255), 300000L, "sin calibrar no corrige nada");
    check_eq(sag.checksum(), 0, "la tabla vacia suma cero");

    // 100 partes por punto hasta duty 240, y 2000 con el pin siempre en alto.
    for (uint32_t k = 0; k < SupplySag::SIZE - 1; k++)
    {
        check(sag.apply((k << 16) | (100 * k)), "cada entrada se escribe");
    }
    check(sag.apply(((uint32_t)(SupplySag::SIZE - 1) << 16) | 2000UL), "y la del pin en alto");
    check(!sag.apply(((uint32_t)(SupplySag::SIZE - 1) << 16) | 2000UL),
          "la misma escritura dos veces no hace nada");
    check(!sag.write_packed((17UL << 16) | 5UL), "un indice que no existe se rechaza");
    check_eq(sag.checksum(), 0x64de, "la suma de Fletcher es la que calcula Python");

    check_eq(sag.error(0, 255), 0, "con el pin en bajo no hay caida");
    check_eq(sag.error(24, 255), 150, "entre dos puntos, interpola");
    check_eq(sag.error(240, 255), 1500, "en un punto, el punto");
    check_eq(sag.error(248, 255), 1767, "el ultimo tramo va de 240 a full");
    check_eq(sag.error(255, 255), 2000, "a fondo, el ultimo punto");
    check_eq(sag.error(300, 255), 2000, "por arriba de full, el ultimo punto");
    check_eq(sag.correct(100000UL, 24, 255), 100000L - 1500L, "corrige la suma entera");
    check(sag.correct(4000000000UL, 200, 255) < 4000000000UL,
          "una suma enorme no desborda");

    SupplySag baja;
    baja.write_packed((1UL << 16) | 50UL);
    check_eq(baja.error(8, 255), 25, "una tabla que sube redondea bien");
    baja.write_packed((2UL << 16) | 0UL);
    check_eq(baja.error(24, 255), 25, "y una que baja, tambien");
    check((uint16_t)baja.write_packed((3UL << 16) | 60000UL) && baja.entry[3] == SupplySag::MAX_ENTRY,
          "una entrada enorme se acota");

    // ------------------------------------------------------------------- el PID

    Pid pid;
    pid.kp = Pid::Kp::from_float(1.0f).raw();
    pid.ki = 0;
    pid.kd = 0;
    pid.set_alpha(Pid::Alpha::from_int(1));
    pid.refresh(255);

    check(pid.configure(1, 0, 0), "la primera configuracion reinicia");
    check(!pid.configure(1, 0, 0), "y repetirla no");

    check_eq(pid.step(10, -255, 255, 0), 10, "kp = 1 devuelve el error");
    check_eq(pid.step(1000, -255, 255, 0), 255, "y recorta en el techo del actuador");
    check_eq(pid.step(-1000, 0, 255, 0), 0,
             "con un puente de un solo cuadrante el piso es cero");

    // El defecto que este modulo existe para hacer imposible: cambiar la magnitud
    // realimentada sin cambiar el controlador tiene que olvidar el integrador.
    pid.ki = Pid::Ki::from_float(0.01f).raw();
    pid.refresh(255);
    pid.configure(1, 0, 0);
    for (int k = 0; k < 200; k++) { pid.step(100, -255, 255, 0); }
    check(pid.integral() != 0, "el integrador se carga con el error sostenido");

    check(pid.configure(1, 1, 0),
          "cambiar target sin cambiar mode tambien reinicia");
    check_eq(pid.integral(), 0, "y deja el integrador en cero");

    // La cota del integrador tiene que seguir a ki: con ki = 1 el termino integral
    // satura el actuador con una suma de 255, asi que ahi tiene que quedarse.
    //
    // Con kp en cero a proposito. Con kp = 1 y un error de 1000 el actuador satura
    // por el termino proporcional desde el primer periodo, asi que la integracion
    // condicional no carga nunca y esto no mediria la cota sino el anti-windup.
    pid.kp = 0;
    pid.ki = Pid::Ki::from_float(1.0f).raw();
    pid.refresh(255);
    pid.configure(2, 0, 0);
    for (int k = 0; k < 2000; k++) { pid.step(1000, -255, 255, 0); }
    check_eq(pid.integral(), 255, "la cota del integrador sigue a ki");

    // Y con ki chico manda el limite del propio tipo, no ki: la acumulacion tiene
    // que quedar en rango le importe o no a la ganancia.
    pid.ki = Pid::Ki::from_float(1e-6f).raw();
    pid.refresh(255);
    pid.configure(3, 0, 0);
    check(pid.integral() == 0, "y reconfigurar vuelve a dejarlo en cero");
    for (int k = 0; k < 3000; k++) { pid.step(30000, -255, 255, 0); }
    check(pid.integral() == 3000L * 30000L,
          "con ki chico la suma crece libre sin desbordar");

    printf("\n%d falla(s)\n", fails);
    return fails ? 1 : 0;
}
