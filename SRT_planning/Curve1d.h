#ifndef CURVE1D
#define CURVE1D

#include <vector>

// using a quartic polynomial to describe the speed, s means lon, d means lat
class Curve1d
{
public:
    double x0;
    double x1;
    double x2;
    double x3;
    double x4;

    std::vector<double> init_s;
    std::vector<double> init_d;

    Curve1d();
    Curve1d(std::vector<double>& s, std::vector<double>& d);
    void calculate_s(double v_0, double v_1, double v_2);
    void calculate_d(double d_0, double v_0, double a_0, double t_0, double offset);
    double s(double t);
    double d(double t);
    double v(double t);
    double a(double t);
    double max_speed();
    double min_speed();
    // the forward distance in the following 1 second
    double forward();
    double avg_speed();
    void print();
};

#endif // !CURVE1D


