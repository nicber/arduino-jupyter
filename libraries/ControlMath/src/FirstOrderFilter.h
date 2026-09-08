// One-pole low pass, run entirely in integers.
//
//   y[n] = y[n-1] + alpha * (x[n] - y[n-1]),   alpha = dt / (tau + dt)
//
// which is the exact pole of an RC section sampled at `dt`. Cascade two of
// them for a two-pole response; the corner moves down by a factor of about
// 1.55 when you do, which is usually what you want anyway.
//
// The thing that breaks a naive integer IIR is that `alpha * (x - y)` rounds
// to zero long before y reaches x: with a long time constant the output stalls
// several counts short and stays there. Nudging the state by one count whenever
// the difference is non-zero -- the usual patch -- only trades the stall for a
// limit cycle and a slew-rate floor.
//
// This one keeps the remainder instead, through Fixed::scale_carry(). Whatever
// part of `alpha * (x - y)` does not fit in the state is carried into the next
// sample, so a step too small to move the state now moves it a few samples
// later and the output converges exactly, for any alpha and any input. Nothing
// is discarded, so there is no bias and no dead zone.
//
// The state is a Fixed with `Guard` fractional bits -- guard bits are a scale,
// so they are spelled as one. Sizing it: the state is an int32_t, so the input
// must stay inside +/-2^(31 - Guard). Guard buys smoothness during a transient,
// not steady-state accuracy -- the carry already gives that -- so a
// free-running counter can afford Guard = 4 (+/-2^27 counts) and a bounded ADC
// channel can spend 8 on it.

#ifndef CONTROLMATH_FIRSTORDERFILTER_H
#define CONTROLMATH_FIRSTORDERFILTER_H

#include <stdint.h>

#include "FixedPoint.h"

template <unsigned Guard = 8>
class FirstOrderFilter
{
    public:

    // alpha is a fraction in [0, 1]; Q16 resolves it to 1.5e-5, which at a 1 ms
    // sample is a time constant of up to about a minute.
    typedef Fixed<int32_t, 16> Alpha;

    // The filter's own state: the input with `Guard` fractional bits under it.
    typedef Fixed<int32_t, Guard> State;

    // tau and dt in seconds. tau <= 0 gives alpha = 1: a pass-through, which is
    // the natural way for a host to switch the filter off.
    static constexpr Alpha alpha_for(float tau, float dt)
    {
        return tau > 0.0f ? Alpha::from_float(dt / (tau + dt)) : Alpha::from_int(1);
    }

    // constexpr so that filter instances at file scope are initialised by the
    // loader rather than by a global constructor that runs before setup().
    constexpr explicit FirstOrderFilter(Alpha alpha = Alpha::from_int(1),
                                        int32_t initial = 0)
        : m_alpha(alpha)
        , m_state(State::from_int(initial))
        , m_carry(0)
    {
    }

    // Safe to call while running: the state is in output units, so the output
    // does not jump when the corner moves.
    void set_alpha(Alpha alpha) { m_alpha = alpha; }

    void reset(int32_t x)
    {
        m_state = State::from_int(x);
        m_carry = 0;
    }

    int32_t update(int32_t x)
    {
        int32_t step = m_alpha.scale_carry(State::from_int(x).raw() - m_state.raw(),
                                           m_carry);

        m_state = m_state + State::from_raw(step);
        return value();
    }

    int32_t value(void) const { return m_state.to_int(); }

    private:

    Alpha   m_alpha;
    State   m_state;
    int32_t m_carry;    // remainder of alpha * diff, in Alpha's fraction
};

#endif  // CONTROLMATH_FIRSTORDERFILTER_H
