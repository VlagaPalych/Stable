#include "AP_Math.h"
#include <float.h>

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0f;
    }
    if (v >= 1.0f) {
        return M_PI/2;
    }
    if (v <= -1.0f) {
        return -M_PI/2;
    }
    return asinf(v);
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

/*
  linear interpolation based on a variable in a range
 */
float linear_interpolate(float low_output, float high_output,
                         float var_value,
                         float var_low, float var_high)
{
    if (var_value <= var_low) {
        return low_output;
    }
    if (var_value >= var_high) {
        return high_output;
    }
    float p = (var_value - var_low) / (var_high - var_low);
    return low_output + p * (high_output - low_output);
}

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
float wrap_PI(float angle_in_radians)
{
    if (angle_in_radians > 10*M_PI || angle_in_radians < -10*M_PI) {
        // for very large numbers use modulus
        angle_in_radians = fmodf(angle_in_radians, 2*M_PI);
    }
    while (angle_in_radians > M_PI) angle_in_radians -= 2*M_PI;
    while (angle_in_radians < -M_PI) angle_in_radians += 2*M_PI;
    return angle_in_radians;
}

/*
 * wrap an angle in radians to 0..2PI
 */
float wrap_2PI(float angle)
{
    if (angle > 10*M_PI || angle < -10*M_PI) {
        // for very large numbers use modulus
        angle = fmodf(angle, 2*M_PI);
    }
    while (angle > 2*M_PI) angle -= 2*M_PI;
    while (angle < 0) angle += 2*M_PI;
    return angle;
}

