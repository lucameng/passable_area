#ifndef DR_MATH_H
#define DR_MATH_H

#include <math.h>
#include <cstdint>

namespace dr_math {

const double kEps = 1e-8;

inline bool equal(double a, double b)
{
    //	return a == b;
    if (isinf(a) && isinf(b))
    {
        return true;
    }
    return fabs(a - b) <= kEps;
}

inline bool lessThan(double a, double b)
{
    //	return a < b;
    return (a - b) < -(kEps);
}

inline bool greaterThan(double a, double b)
{
    //	return a > b;
    return (a - b) > kEps;
}

inline bool lessEqual(double a, double b)
{
    //	return a <= b;
    if (isinf(a) && isinf(b))
    {
        return true;
    }
    return (a - b) < kEps;
}

inline bool greaterEqual(double a, double b)
{
    //	return a >= b;
    if (isinf(a) && isinf(b))
    {
        return true;
    }
    return (a - b) > -(kEps);
}

// float compare
inline bool equal(float a, float b)
{
    if (isinf(a) && isinf(b))
    {
        return true;
    }
    return fabs(a - b) <= 1e-6;
}

inline bool lessThan(float a, float b) { return (a - b) < -(1e-6); }

inline bool greaterThan(float a, float b) { return (a - b) > 1e-6; }

inline bool lessEqual(float a, float b)
{
    if (isinf(a) && isinf(b))
    {
        return true;
    }
    return (a - b) < 1e-6;
}

inline bool greaterEqual(float a, float b)
{
    if (isinf(a) && isinf(b))
    {
        return true;
    }
    return (a - b) > -(1e-6);
}
} // namespace dr_utils

#endif // DR_MATH_H
