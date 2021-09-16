#ifndef  GENERATOR
#define	 GENERATOR

#include <vector>
#include <iostream>

#include "Curve1d.h"
 // sampling & generating bundles
class Trajectory1dGenerator
{
public:
    Trajectory1dGenerator(double max_speed, std::vector<double>& s, std::vector<double>& d);
    // sampling to generate lon & lat path, record them in the two empty vectors
    void GenerateTrajectoryBundles(bool loss, std::vector<Curve1d>& lon_trajectory1d_bundle,
        std::vector<Curve1d>& lat_trajectory1d_bundle);
private:
    double max_speed = 10;
    double min_speed = 0.0;
    double dec_a = -2.0;
    double acc_a = 2.0;
    std::vector<double> init_s;
    std::vector<double> init_d;
};

#endif // ! GENERATOR

