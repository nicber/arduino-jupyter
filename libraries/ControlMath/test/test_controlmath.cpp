// Host-side checks for FixedPoint.h, FirstOrderFilter.h and MovingAverage.h. All
// three are pure arithmetic, so they are far easier to get right here than on the
// target.
//
//   g++ -std=c++11 -O2 -Wall -Wextra -I ../src test_controlmath.cpp -o test && ./test
//
// Keep this compiling under C++11: that is what the AVR core builds with.

#include <cstdio>
#include <cstdint>
#include <cmath>
#include "FixedPoint.h"
#include "FirstOrderFilter.h"
#include "MovingAverage.h"

// La media de referencia: al más cercano con los empates hacia más infinito, que
// es dividir hacia abajo el total más medio N. Escrita a lo bruto a propósito.
static long media_ref(long total, long n)
{
    const long x = total + n / 2;
    return x >= 0 ? x / n : -((-x + n - 1) / n);
}

static int fails = 0;
static void check(bool ok, const char* what, double got, double want)
{
    if (!ok) { printf("FAIL %-34s got %g want %g\n", what, got, want); fails++; }
}
#define CHECK_NEAR(expr, want, tol) do { double g=(expr); check(fabs(g-(want))<=(tol), #expr, g, (want)); } while(0)

int main()
{
    typedef Fixed<int32_t, 22> Kp;
    typedef Fixed<int32_t, 30> KiDt;
    typedef Fixed<int32_t, 16> Q16;

    // round trip through the chosen scales
    CHECK_NEAR(Kp::from_float(0.5f).to_float(),      0.5,     1e-6);
    CHECK_NEAR(Kp::from_float(-0.001f).to_float(),  -0.001,   1e-6);
    CHECK_NEAR(Kp::from_float(511.0f).to_float(),    511.0,   1e-4);
    CHECK_NEAR(KiDt::from_float(5e-8f).to_float(),   5e-8,    1e-9);

    // scale(): gain in engineering units meets a signal in raw counts
    CHECK_NEAR(Kp::from_float(0.5f).scale(1000),     500.0,   0.5);
    CHECK_NEAR(Kp::from_float(0.5f).scale(-1000),   -500.0,   0.5);
    CHECK_NEAR(Kp::from_float(0.001f).scale(123456), 123.456, 1.0);
    CHECK_NEAR(KiDt::from_float(5e-8f).scale(100000000L), 5.0, 0.1);

    // rounding is to nearest away from zero, not toward minus infinity
    check(Q16::from_float(0.5f).scale(1) == 1,  "round +0.5 -> 1", Q16::from_float(0.5f).scale(1), 1);
    check(Q16::from_float(0.5f).scale(-1) == 0, "tie -0.5 -> 0",  Q16::from_float(0.5f).scale(-1), 0);
    check(Q16::from_float(0.6f).scale(-1) == -1,"round -0.6 -> -1",Q16::from_float(0.6f).scale(-1), -1);
    check(Q16::from_float(0.4f).scale(1) == 0,  "round +0.4 -> 0", Q16::from_float(0.4f).scale(1), 0);

    // no half-LSB bias on a long run of a proportional term
    long sum = 0;
    for (int e = -500; e <= 500; e++) sum += Q16::from_float(0.3f).scale(e);
    check(sum == 0, "P term is unbiased over +/-500", (double)sum, 0);

    // operator* and rescale
    CHECK_NEAR((Q16::from_float(1.5f) * Q16::from_float(2.5f)).to_float(), 3.75, 1e-4);
    CHECK_NEAR(Kp::from_float(0.25f).rescale<16>().to_float(),             0.25, 1e-4);
    CHECK_NEAR(Q16::from_float(0.25f).rescale<22>().to_float(),            0.25, 1e-6);
    check(Q16::from_float(2.6f).to_int() == 3, "to_int rounds", Q16::from_float(2.6f).to_int(), 3);
    check(Q16::from_float(-2.6f).to_int() == -3, "to_int rounds negatives", Q16::from_float(-2.6f).to_int(), -3);
    check(Q16::from_float(-2.5f).to_int() == -2, "to_int ties go up", Q16::from_float(-2.5f).to_int(), -2);

    // filter: step response reaches the input and does not stall short of it
    {
        FirstOrderFilter<8> f(FirstOrderFilter<8>::alpha_for(0.005f, 0.001f));
        int32_t y = 0;
        for (int n = 0; n < 200; n++) y = f.update(100);
        check(y == 100, "filter settles exactly on a step", (double)y, 100);

        // one time constant of a 5 ms tau at 1 ms: 63% of the way
        FirstOrderFilter<8> g(FirstOrderFilter<8>::alpha_for(0.005f, 0.001f));
        int32_t v = 0;
        for (int n = 0; n < 5; n++) v = g.update(1000);
        check(v > 590 && v < 680, "filter hits ~63% after one tau", (double)v, 632);
    }

    // the small-signal case the old +/-1 nudge existed for: alpha * diff < 1 LSB
    {
        FirstOrderFilter<8> f(FirstOrderFilter<8>::alpha_for(1.0f, 0.001f)); // alpha ~ 1e-3
        int32_t y = 0;
        for (int n = 0; n < 20000; n++) y = f.update(5);
        check(y == 5, "no stall when alpha*diff < 1 output LSB", (double)y, 5);
    }

    // pass-through when tau is zero
    {
        FirstOrderFilter<4> f(FirstOrderFilter<4>::alpha_for(0.0f, 0.001f));
        check(f.update(123456) == 123456, "tau = 0 is a pass-through", f.update(123456), 123456);
    }

    // alpha from the host is clamped to [0, 1]: alpha > 2 would diverge
    {
        typedef FirstOrderFilter<8> F;
        F f;
        f.set_alpha(F::Alpha::from_raw(5L << 16));
        int32_t y = 0;
        for (int n = 0; n < 50; n++) y = f.update(1000);
        check(y == 1000, "alpha > 1 is clamped to a pass-through", (double)y, 1000);
        f.set_alpha(F::Alpha::from_raw(-100));
        for (int n = 0; n < 50; n++) y = f.update(-1000);
        check(y == 1000, "alpha < 0 is clamped to a frozen filter", (double)y, 1000);
    }

    // headroom: guard 4 must survive a free-running position counter
    {
        FirstOrderFilter<4> f(FirstOrderFilter<4>::alpha_for(0.05f, 0.001f));
        int32_t y = 0;
        for (int32_t x = 0; x < 100000000L; x += 4096) y = f.update(x);
        check(y > 99000000L && y <= 100000000L, "guard 4 holds 1e8 counts", (double)y, 1e8);
    }

    // past the headroom the filter must saturate, never wrap: a filtered position
    // that jumps to the opposite extreme is the worst thing it could report
    {
        typedef FirstOrderFilter<4> F;
        F f(F::alpha_for(0.0f, 0.001f));        // alpha = 1: follows the input exactly

        check(f.update(F::limit()) == F::limit(), "guard 4 reaches its limit",
              f.update(F::limit()), F::limit());
        check(f.update(2000000000L) == F::limit(), "and saturates past it instead of wrapping",
              f.update(2000000000L), F::limit());
        check(f.update(-2000000000L) == -F::limit(), "on the negative side too",
              f.update(-2000000000L), -F::limit());
    }

    // a free-running counter run far past the limit keeps its sign
    {
        typedef FirstOrderFilter<4> F;
        F f(F::alpha_for(0.05f, 0.001f));
        int32_t y = 0;
        bool signo = true;
        for (int64_t x = 0; x < 3000000000LL; x += 65536)
        {
            y = f.update((int32_t)(x > F::limit() ? F::limit() : x));
            if (y < 0) signo = false;
        }
        check(signo && y == F::limit(), "32768 turns in one direction never flip the sign",
              (double)y, (double)F::limit());
    }

    // a very long time constant: the carry has to do all the work here
    {
        FirstOrderFilter<8> f(FirstOrderFilter<8>::alpha_for(10.0f, 0.001f));
        int32_t y = 0;
        for (int n = 0; n < 400000; n++) y = f.update(-7);
        check(y == -7, "converges exactly with tau/dt = 10000", (double)y, -7);
    }

    // and it must converge from either side
    {
        FirstOrderFilter<8> f(FirstOrderFilter<8>::alpha_for(0.05f, 0.001f), 1000);
        int32_t y = 0;
        for (int n = 0; n < 5000; n++) y = f.update(-1000);
        check(y == -1000, "converges downwards exactly", (double)y, -1000);
    }

    // ------------------------------------------------------- MovingAverage
    //
    // N es un parámetro de plantilla, así que la división que cierra la media es
    // por una constante y el compilador no emite ninguna. Lo que se comprueba acá
    // es que la media sea la correcta para cualquier N, potencia de dos o no.
    {
        typedef MovingAverage<int16_t, 4> MA4;
        check(MA4::SIZE == 4, "window of 4", (double)MA4::SIZE, 4);

        // Against a brute-force window, every step of the way, for a power of two
        // and for one that is not.
        {
            MA4 ma;
            int16_t v[4] = {0, 0, 0, 0};
            bool media = true, suma = true, q2 = true;
            for (int k = 1; k <= 400; k++)
            {
                const int16_t x = (int16_t)((k * 37) % 1000 - 500);
                v[3] = v[2]; v[2] = v[1]; v[1] = v[0]; v[0] = x;
                const long s4 = (long)v[0] + v[1] + v[2] + v[3];
                const long m4 = media_ref(s4, 4);

                if (ma.push(x) != m4) { media = false; }
                if (ma.total() != s4) { suma  = false; }
                // With N a power of two the running sum IS the mean in Q log2(N).
                if (ma.mean_fixed<2>().raw() != s4) { q2 = false; }
            }
            check(media, "N = 4 matches a brute-force mean", media, 1);
            check(suma,  "and total() is the running sum", suma, 1);
            check(q2,    "and mean_fixed<2>() is that sum untouched", q2, 1);
        }
        {
            MovingAverage<int16_t, 10> ma;      // not a power of two
            int16_t v[10] = {0};
            bool media = true;
            for (int k = 1; k <= 400; k++)
            {
                const int16_t x = (int16_t)((k * 53) % 1400 - 700);
                for (int j = 9; j > 0; j--) { v[j] = v[j - 1]; }
                v[0] = x;
                long s10 = 0;
                for (int j = 0; j < 10; j++) { s10 += v[j]; }
                const long m10 = media_ref(s10, 10);
                if (ma.push(x) != m10) { media = false; }
            }
            check(media, "N = 10 matches a brute-force mean too", media, 1);
        }

        // The fraction is the point: a mean of 11.5 is 11.5, not 11 or 12.
        MA4 medio;
        medio.push(10); medio.push(11); medio.push(12); medio.push(13);
        CHECK_NEAR(medio.mean_fixed<2>().to_float(), 11.5, 1e-9);
        check(medio.mean() == 12, "and mean() rounds to the nearest",
              (double)medio.mean(), 12);

        // Rounding is symmetric about zero: truncation would put half an LSB of DC
        // on a signal centred at zero.
        MA4 neg;
        neg.push(-10); neg.push(-11); neg.push(-12); neg.push(-13);
        check(neg.mean() == -11, "and ties go towards +inf, as FixedPoint does",
              (double)neg.mean(), -11);
        check(neg.total() == -46, "a negative window is exact",
              (double)neg.total(), -46);

        // prime() fills the window so the mean starts there instead of climbing.
        MA4 cebado;
        cebado.prime(100);
        check(cebado.mean() == 100, "prime() starts the window full",
              (double)cebado.mean(), 100);
        check(cebado.push(100) == 100, "and stays there", (double)cebado.push(100), 100);
        cebado.reset();
        check(cebado.total() == 0, "reset() empties it", (double)cebado.total(), 0);

        // The division that closes the mean is exact for every value the sum type
        // can hold, power of two or not. Exhaustive over all 65536 of an int16.
        {
            bool p2 = true, impar = true, par = true;
            for (long t = -32768; t <= 32767; t++)
            {
                const int16_t x = (int16_t)t;
                if (MaDividir<int16_t, 4>::de(x)   != media_ref(t, 4))   { p2 = false; }
                if (MaDividir<int16_t, 45>::de(x)  != media_ref(t, 45))  { impar = false; }
                if (MaDividir<int16_t, 10>::de(x)  != media_ref(t, 10))  { par = false; }
            }
            check(p2,    "N = 4 is exact over the whole int16 range", p2, 1);
            check(impar, "and N = 45 too", impar, 1);
            check(par,   "and N = 10 too", par, 1);
        }

        // A window of 1 is a pass-through, and it has to compile.
        MovingAverage<int16_t, 1> uno;
        check(uno.push(77) == 77, "a window of 1 passes the sample through",
              (double)uno.push(77), 77);

        // Sixteen, with a sum that would not fit in the sample type.
        MovingAverage<int16_t, 16> ma16;
        ma16.prime(30000);
        check(ma16.total() == 480000L, "a window of 16 sums past the sample type",
              (double)ma16.total(), 480000.0);
        check(ma16.mean() == 30000, "and its mean comes back",
              (double)ma16.mean(), 30000);
    }

    printf(fails ? "\n%d FAILED\n" : "\nall fixed-point, filter and moving-average checks passed\n", fails);
    return fails != 0;
}
