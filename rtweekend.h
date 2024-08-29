#pragma once
#ifndef RTWEEKEND_H
#define RTWEEKEND_H

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>

using std::make_shared;
using std::shared_ptr;

//无穷大和圆周率
const double infinity = std::numeric_limits<double>::infinity();
const double pi = 3.1415926535897932385;

//角度转弧度
inline double degrees_to_radians(double degrees) 
{
    return degrees * pi / 180.0;
}

//生成一个[0,1)范围内的数
inline double random_double() {
    static std::uniform_real_distribution<double> distribution(0.0, 1.0);
    static std::mt19937 generator;
    return distribution(generator);
}

//生成一个[min,max)范围内的数
inline double random_double(double min, double max)
{
    return min + (max - min) * random_double();
}

inline int random_int(int min, int max) {
    return int(random_double(min, max + 1));
}

#include "color.h"
#include "interval.h"
#include "ray.h"
#include "vec3.h"

#endif
