#include "Curve1d.h"
#include <iostream>
#include <algorithm>

Curve1d::Curve1d()
{
    x0 = 0.0;
    x1 = 0.0;
    x2 = 0.0;
    x3 = 0.0;
    x4 = 0.0;
}

Curve1d::Curve1d(std::vector<double>& s, std::vector<double>& d)
{
    init_s = s;
    init_d = d;
}

void Curve1d::calculate_s(double v_0, double v_1, double v_2)
{
    x0 = v_0;
    x1 = 2 * v_1 - 0.5 * v_2 - 1.5 * v_0;
    x2 = 0.5 * v_2 - v_1 + 0.5 * v_0;
    x3 = 0;
    x4 = 0;
}

void Curve1d::calculate_d(double d_0, double v_0, double a_0, double t_0, double offset)
{
    // when t=t0, speed=0, d=0
    x4 = 0.0;
    x3 = 0.0;
    x2 = 3.0 * offset / pow(t_0, 3.0);
    x1 = -6.0 * offset / pow(t_0, 2.0);
    x0 = 3.0 * offset / t_0;
}

double Curve1d::s(double t)
{
    double ret = 0.0;;
    ret += 0.25 * x3 * pow(t, 4.0);
    ret += 0.33 * x2 * pow(t, 3.0);
    ret += 0.5 * x1 * pow(t, 2.0);
    ret += x0 * pow(t, 1.0);
    ret += init_s[0];
    return ret;
}

double Curve1d::d(double t)
{
    double ret = 0.0;
    ret += 0.25 * x3 * pow(t, 4.0);
    ret += 0.33 * x2 * pow(t, 3.0);
    ret += 0.5 * x1 * pow(t, 2.0);
    ret += x0 * pow(t, 1.0);
    ret += init_d[0];
    return ret;
}

double Curve1d::v(double t)
{
    double ret = 0.0;
    ret += x4 * pow(t, 4.0);
    ret += x3 * pow(t, 3.0);
    ret += x2 * pow(t, 2.0);
    ret += x1 * pow(t, 1.0);
    ret += x0;
    ret = std::abs(ret);
    return ret;
}

double Curve1d::a(double t)
{
    double ret = 0.0;
    ret += 4 * x4 * pow(t, 3.0);
    ret += 3 * x3 * pow(t, 2.0);
    ret += 2 * x2 * pow(t, 1.0);
    ret += x1;
    ret = std::abs(ret);
    return ret;
    
}

double Curve1d::max_speed()
{
    double ret = 0.0;
    for (double t = 0.0; t <= 1.0; t += 0.1)
    {
        double velocity = x0 + x1 * t + x2 * pow(t, 2.0) + x3 * pow(t, 3.0) + x4 * pow(t, 4.0);
        velocity = std::abs(velocity);
        ret = std::max(velocity, ret);
    }
    return ret;
}

double Curve1d::min_speed()
{
    double ret = 100.0;
    for (double t = 0.0; t <= 1.0; t += 0.1)
    {
        double velocity = x0 + x1 * t + x2 * pow(t, 2.0) + x3 * pow(t, 3.0) + x4 * pow(t, 4.0);
        ret = std::abs(velocity);
        ret = std::min(velocity, ret);
    }
    return ret;
}

double Curve1d::forward()
{
    double ret = 0.0;
    ret += 0.2 * x4 + 0.25 * x3 + 0.33 * x2 + 0.5 * x1 + x0;
    return ret;
}

double Curve1d::avg_speed()
{
    double ret = 0.0;
    for (double t = 0.0; t <= 1.0; t += 0.2)
    {
        double velocity = x1 + 2.0 * x2 * t + 3.0 * x3 * pow(t, 2) + 4.0 * x4 * pow(t, 3);
        velocity = std::abs(velocity);
        ret = std::min(velocity, ret);
    }
    ret /= 11;
    return ret;
}

void Curve1d::print()
{
    for (double t = 0; t <= 1.0; t += 0.1)
    {
        std::cout << s(t) << " ";
    }
    std::cout << std::endl;
}