#ifndef DEMO
#define DEMO

#include <vector>
#include "Curve1d.h"
#include "PathPoint.h"

// One single trajectory, Cost is being calculated here, initialized with lon & lat curve
class TrajectoryDemo
{
public:
    TrajectoryDemo();
    TrajectoryDemo(std::vector<double>& s, std::vector<double>& d,
        Curve1d lon, Curve1d lat, std::vector<PathPoint>& ref);
    double cost();
    Curve1d lon();
    Curve1d lat();

private:
    std::vector<double> init_s;
    std::vector<double> init_d;
    Curve1d lon_curve;
    Curve1d lat_curve;
    std::vector<PathPoint> ref_points;
    double Cost = 0.0;

};

#endif // !DEMO

