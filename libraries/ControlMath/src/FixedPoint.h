// Compile-time-scaled fixed point, for control loops that must not touch float.
//
// A `Fixed<Raw, Frac>` is an integer of type `Raw` holding a value scaled by
// 2^Frac. The scale is a template parameter, so choosing it is free: every
// shift is folded at compile time and a conversion between scales that a
// float version would have done at runtime does not exist at all.
//
// Why not an existing library: fpm and CNL are the good ones, and both need
// <type_traits> and <limits>, which avr-g++ does not ship. The single trait
// that is actually needed here is four lines, so it is spelled out below.
//
// Picking a scale. Frac buys resolution at the cost of range: a value needs
// `Frac` bits below the point and enough above it for its largest magnitude,
// and the two must fit in `Raw`. A gain of 0.5 and a gain of 0.00005 want very
// different scales, which is exactly why this is a parameter and not 16.
//
//   Fixed<int32_t, 22> kp;   // +/-511,     resolution 2.4e-7
//   Fixed<int32_t, 30> ki;   // +/-1.99,    resolution 9.3e-10
//
// Cost on an AVR. Products of two `int32_t` go through a 64-bit intermediate
// (__muldi3, of the order of 200 cycles); products of two `int16_t` compile to
// a handful of MUL instructions. Prefer the narrowest `Raw` that holds the
// range, and keep multiplies out of the inner loop where the operands are
// known to be small.
//
// Every operation that discards bits rounds to nearest, ties towards positive
// infinity. An arithmetic right shift on its own rounds towards minus infinity
// instead, which puts a half-LSB DC offset on a proportional term -- a steady
// pull to one side that a control loop then has to integrate away.

#ifndef CONTROLMATH_FIXEDPOINT_H
#define CONTROLMATH_FIXEDPOINT_H

#include <stdint.h>

// The one trait needed: the type a product of two `T` is computed in.
template <typename T> struct FixedWider;
template <> struct FixedWider<int8_t>  { typedef int16_t type; };
template <> struct FixedWider<int16_t> { typedef int32_t type; };
template <> struct FixedWider<int32_t> { typedef int64_t type; };

template <typename Raw, unsigned Frac>
class Fixed
{
    public:

    typedef Raw                            raw_type;
    typedef typename FixedWider<Raw>::type wide_type;

    static const unsigned FRAC = Frac;
    static const Raw      ONE  = (Raw)1 << Frac;

    // Half an LSB, used to round rather than truncate. Frac == 0 is a plain
    // integer and has nothing to round, and the shift below must still be a
    // legal one for the compiler to accept the dead branch.
    static const Raw      HALF = Frac ? ((Raw)1 << (Frac ? Frac - 1 : 0)) : 0;

    constexpr Fixed(void) : m_raw(0) {}

    // Tagged so that a bare integer cannot silently become a Fixed with a
    // scale it was never in.
    struct FromRaw {};
    constexpr Fixed(Raw value, FromRaw) : m_raw(value) {}

    static constexpr Fixed from_raw(Raw value)
    {
        return Fixed(value, FromRaw());
    }

    // The only place a float is allowed. Call it on a literal and it folds to a
    // constant; call it when a host-supplied gain changes and it costs one
    // float multiply, outside the control step.
    static constexpr Fixed from_float(float value)
    {
        return from_raw((Raw)(value * (float)ONE + (value >= 0.0f ? 0.5f : -0.5f)));
    }

    static constexpr Fixed from_int(Raw value)
    {
        return from_raw((Raw)(value << Frac));
    }

    constexpr Raw   raw(void)      const { return m_raw; }
    constexpr float to_float(void) const { return (float)m_raw / (float)ONE; }
    constexpr Raw   to_int(void)   const { return (Raw)((m_raw + HALF) >> Frac); }

    // Multiply a plain integer by this value and get a plain integer back --
    // the workhorse of a control law, where a gain in engineering units meets a
    // signal in raw counts. The product is formed in the wider type, so the
    // only overflow to worry about is of the result itself.
    template <typename Int>
    constexpr Raw scale(Int x) const
    {
        return round_shift((wide_type)m_raw * (wide_type)x);
    }

    // scale(), but exact over a sequence of calls. The bits below the result's
    // LSB are handed back in `carry` instead of being rounded away, so feeding
    // the same carry to the next call recovers them. A running sum of these is
    // exact where a sum of scale() drifts by up to half an LSB per term -- and,
    // worse, stalls completely once a term rounds to zero.
    //
    // `carry` must start at zero and belongs to the sequence, not to the value:
    // one accumulator per running total.
    template <typename Int>
    Raw scale_carry(Int x, Raw& carry) const
    {
        wide_type product = (wide_type)m_raw * (wide_type)x + carry;

        // The shift floors, so the remainder is never negative and the split
        // is exact rather than merely close.
        carry = (Raw)product & (ONE - 1);
        return (Raw)(product >> Frac);
    }

    // Same scale in, same scale out.
    constexpr Fixed operator+(Fixed o) const { return from_raw((Raw)(m_raw + o.m_raw)); }
    constexpr Fixed operator-(Fixed o) const { return from_raw((Raw)(m_raw - o.m_raw)); }
    constexpr Fixed operator-(void)    const { return from_raw((Raw)(-m_raw)); }

    constexpr Fixed operator*(Fixed o) const
    {
        return from_raw(round_shift((wide_type)m_raw * (wide_type)o.m_raw));
    }

    // Re-scale to a different number of fractional bits. Both shift counts are
    // clamped because both branches of the ternary are compiled, even though
    // only one can ever be taken for a given pair of scales.
    template <unsigned Frac2>
    constexpr Fixed<Raw, Frac2> rescale(void) const
    {
        return Fixed<Raw, Frac2>::from_raw(
            Frac2 >= Frac ? (Raw)(m_raw << (Frac2 >= Frac ? Frac2 - Frac : 0))
                          : (Raw)(m_raw >> (Frac >= Frac2 ? Frac - Frac2 : 0)));
    }

    constexpr bool operator==(Fixed o) const { return m_raw == o.m_raw; }
    constexpr bool operator!=(Fixed o) const { return m_raw != o.m_raw; }
    constexpr bool operator< (Fixed o) const { return m_raw <  o.m_raw; }
    constexpr bool operator> (Fixed o) const { return m_raw >  o.m_raw; }
    constexpr bool operator<=(Fixed o) const { return m_raw <= o.m_raw; }
    constexpr bool operator>=(Fixed o) const { return m_raw >= o.m_raw; }

    private:

    // Shift `product` back down by Frac bits, rounding to nearest. The shift
    // itself floors, so adding half an LSB first turns it into round-to-nearest
    // with ties going up -- which is symmetric about zero, unlike the floor.
    static constexpr Raw round_shift(wide_type product)
    {
        return (Raw)((product + (wide_type)HALF) >> Frac);
    }

    Raw m_raw;
};

#endif  // CONTROLMATH_FIXEDPOINT_H
