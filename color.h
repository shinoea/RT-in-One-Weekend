#pragma once
#ifndef COLOR_H
#define COLOR_H

#include "interval.h"
#include "vec3.h"

using color = vec3;

//线性空间转gamma空间
inline double linear_to_gamma(double linear_component)
{
    if (linear_component > 0)
        return std::sqrt(linear_component);

    return 0;
}

void write_color(FILE* fp,const color& pixel_color,unsigned char* color)
{
    auto r = pixel_color.x();
    auto g = pixel_color.y();
    auto b = pixel_color.z();

    if (r != r) r = 0.0;
    if (g != g) g = 0.0;
    if (b != b) b = 0.0;

    //线性空间转gamma空间
    r = linear_to_gamma(r);
    g = linear_to_gamma(g);
    b = linear_to_gamma(b);

    static const interval intensity(0.000, 0.999);
    int rbyte = int(256 * intensity.clamp(r));
    int gbyte = int(256 * intensity.clamp(g));
    int bbyte = int(256 * intensity.clamp(b));

    color[0] = (unsigned char)rbyte;
    color[1] = (unsigned char)gbyte;
    color[2] = (unsigned char)bbyte;
}


#endif
