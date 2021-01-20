#ifndef DEMO
#define DEMO

#include <vector>
#include "Curve1d.h"
#include "PathPoint.h"

// 单条规划路径，需要计算cost，初始化根据纵横的曲线直接得到
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

