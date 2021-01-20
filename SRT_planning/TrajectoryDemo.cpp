#include "TrajectoryDemo.h"

TrajectoryDemo::TrajectoryDemo() {}

TrajectoryDemo::TrajectoryDemo(std::vector<double>& s, std::vector<double>& d,
    Curve1d lon, Curve1d lat, std::vector<PathPoint>& ref)
{
    init_s = s;
    init_d = d;
    lon_curve = lon;
    lat_curve = lat;
    ref_points = ref;
    // 在这里写cost的计算!
    double lon_s_cost = 100 / lon.forward();            // 纵向前进距离的cost
    double avg_speed_cost = 300 - 10 * lon.avg_speed(); // 纵向平均速度的cost
    double offset_cost = 0.0;                           // 横向偏离的cost，越偏cost越大
    for (double t = 0.0; t <= 0.7; t += 0.1)
    {
        double offset = init_d[0];
        offset += 0.2 * lat_curve.x4 * pow(t, 5);
        offset += 0.25 * lat_curve.x3 * pow(t, 4);
        offset += 0.33 * lat_curve.x2 * pow(t, 3);
        offset += 0.5 * lat_curve.x1 * pow(t, 2);
        offset += lat_curve.x0 * pow(t, 1);
        offset_cost += t * std::abs(offset);
    }
    Cost = lon_s_cost + avg_speed_cost + 10 * offset_cost;
}

double TrajectoryDemo::cost() 
{ 
    return Cost;
}

Curve1d TrajectoryDemo::lon() 
{ 
    return lon_curve;
}

Curve1d TrajectoryDemo::lat() 
{ 
    return lat_curve;
}
