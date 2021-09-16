#ifndef COMBINER
#define COMBINER

#include <vector>
#include "PathPoint.h"
#include "TrajectoryDemo.h"
#include "TrajectoryPoint.h"

// examine the trajectory
class TrajectoryCombiner
{
public:
    double PATH =  3.5;
    double VEHICLE = 1.5;
    double SECURE = 0.3;
    double SPEEDLIMIT = 11;
    double CURVERATE = 40;
    TrajectoryCombiner();
    TrajectoryCombiner(double speedlimit, double curverate);
    void Combine(std::vector<PathPoint>& ptr_reference_line, TrajectoryDemo trajectory_pair,
        std::vector<double> s, std::vector<double> d, bool loss);
    bool valid();
    int type();
    // generate path using Frenet->cartesian
    void generate();
    std::vector<TrajectoryPoint> ans();
private:
    std::vector<PathPoint> ref_line;
    TrajectoryDemo Trajectory_final;
    // If this is valid, if not, record error typr
    bool Valid;
    int Type = 0;

    // information of final trajectory points
    double velocity;
    double acceleration;
    double ptr_x;
    double ptr_y;
    double ptr_theta;
    double ptr_kappa;
    std::vector<TrajectoryPoint> Points;
};

#endif // !COMBINER

