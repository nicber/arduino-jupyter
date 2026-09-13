// Host-side checks for FixedPoint.h and FirstOrderFilter.h. Both are pure
// arithmetic, so they are far easier to get right here than on the target.
//
//   g++ -std=c++11 -O2 -Wall -Wextra -I ../src test_controlmath.cpp -o test && ./test
//
// Keep this compiling under C++11: that is what the AVR core builds with.

#include <cstdio>
#include <cstdint>
#include <cmath>
#include "FixedPoint.h"
#include "FirstOrderFilter.h"

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

    // headroom: guard 4 must survive a free-running position counter
    {
        FirstOrderFilter<4> f(FirstOrderFilter<4>::alpha_for(0.05f, 0.001f));
        int32_t y = 0;
        for (int32_t x = 0; x < 100000000L; x += 4096) y = f.update(x);
        check(y > 99000000L && y <= 100000000L, "guard 4 holds 1e8 counts", (double)y, 1e8);
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

    printf(fails ? "\n%d FAILED\n" : "\nall fixed-point and filter checks passed\n", fails);
    return fails != 0;
}
