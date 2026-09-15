// Comprobaciones de escritorio para los lazos y el PID de Control. Estaban en
// test/test_modulos.cpp; viven acá porque Control no va en el zip del TP2, y
// test_modulos.cpp sí.
//
//   g++ -std=c++11 -O2 -Wall -Wextra \
//       -I ../../ControlMath/src -I ../../AngleSensor/src -I ../../Sense/src \
//       -I ../src \
//       test_control.cpp -o test && ./test
//
// Se mantiene compilando bajo C++11, que es con lo que compila el core del AVR.

#include <cstdio>
#include <cstdint>
#include <cmath>

#include "FixedPoint.h"
#include "FirstOrderFilter.h"
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

int main()
{
    // ------------------------------------------------------------ el ángulo

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

    // Lo que lee un lazo: el cero primero y el signo despues.
    LoopCurrent lcur(2048);
    lcur.update(2148);
    check_eq(lcur.i, 100, "el lazo lee la misma corriente sin invert");
    lcur.invert = 1;
    lcur.update(2148);
    check_eq(lcur.i, -100, "y con el sensor invertido, menos cien");
    check_eq(lcur.sense.i, 100, "sin tocar lo que mide el sensor");

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
