#include "filter.h"

float low_pass_filter(float input, float last_output, float alpha)
{
    return alpha * input + (1.0 - alpha) * last_output;
}

float kalman_filter_std(float input, float r, float q)
{
    static float z;
    static float p = 1;
    float g = 0;
    p = p + q;
    g = p / (p + r);
    z = z + g * (input - z);
    p = (1 - g) * p;
    return z;
}
