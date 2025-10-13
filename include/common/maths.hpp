#ifndef DR_MATHS_H
#define DR_MATHS_H

#include <cmath>
#include <cstdint>

namespace dr {

const double kEps = 1e-8;

inline bool equal(double a, double b)
{
    //	return a == b;
    if (std::isinf(a) && std::isinf(b))
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
    if (std::isinf(a) && std::isinf(b))
    {
        return true;
    }
    return (a - b) < kEps;
}

inline bool greaterEqual(double a, double b)
{
    //	return a >= b;
    if (std::isinf(a) && std::isinf(b))
    {
        return true;
    }
    return (a - b) > -(kEps);
}

// float compare
inline bool equal(float a, float b)
{
    if (std::isinf(a) && std::isinf(b))
    {
        return true;
    }
    return fabs(a - b) <= 1e-6;
}

inline bool lessThan(float a, float b) { return (a - b) < -(1e-6); }

inline bool greaterThan(float a, float b) { return (a - b) > 1e-6; }

inline bool lessEqual(float a, float b)
{
    if (std::isinf(a) && std::isinf(b))
    {
        return true;
    }
    return (a - b) < 1e-6;
}

inline bool greaterEqual(float a, float b)
{
    if (std::isinf(a) && std::isinf(b))
    {
        return true;
    }
    return (a - b) > -(1e-6);
}

inline double rangedRadian(double radian, double radian_min, double radian_max)
{
    while (greaterThan(radian, radian_max) || lessThan(radian, radian_min))
    {
        if (greaterThan(radian, radian_max))
            radian -= M_PI * 2;
        else if (lessThan(radian, radian_min))
            radian += M_PI * 2;
    }
    return radian;
}

inline double rangedRadian(double radian)
{
    double radian_min = -M_PI;
    double radian_max = M_PI;
    auto ret_radian = rangedRadian(radian, radian_min, radian_max);
    return ret_radian;
}

inline double rangedDegree(double degree, double degree_min, double degree_max)
{
    // BAY_ASSERT("robotctl",degree_max > degree_min && degree_max - degree_min == 360);

    while (greaterThan(degree, degree_max) || lessThan(degree, degree_min))
    {
        if (greaterThan(degree, degree_max))
            degree -= 360;
        else if (lessThan(degree, degree_min))
            degree += 360;
    }
    return degree;
}

inline double rangedDegree(double degree)
{
    double degree_min = -180;
    double degree_max = 180;
    auto ret_degree = rangedDegree(degree, degree_min, degree_max);

    return ret_degree;
}

inline double degreeToRadian(double deg) { return deg * M_PI / 180.0; }

inline double radianToDegree(double rad) { return rad * 180.0 / M_PI; }

} // namespace dr

#endif // DR_MATHS_H
