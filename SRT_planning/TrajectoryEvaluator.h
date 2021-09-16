#ifndef EVALUATOR
#define EVALUATOR

#include <vector>
#include <algorithm>
#include "PathPoint.h"
#include "Curve1d.h"
#include "TrajectoryDemo.h"

// permutation and combination
class TrajectoryEvaluator
{
public:
    TrajectoryEvaluator(std::vector<double>& s, std::vector<double>& d,
        std::vector<Curve1d>& lon_trajectory1d_bundle,
        std::vector<Curve1d>& lat_trajectory1d_bundle,
        std::vector<PathPoint>& ptr_reference_line);
    // if there are more pairs left
    bool has_more_trajectory_pairs();
    // total number of pairs
    int num_of_trajectory_pairs();
    // one pair with min Cost value in the remaining pairs
    TrajectoryDemo next_top_trajectory_pair();
private:
    std::vector<double> init_s;
    std::vector<double> init_d;
    std::vector<Curve1d> lon_bundle;
    std::vector<Curve1d> lat_bundle;
    std::vector<PathPoint> ref_points;
    std::vector<TrajectoryDemo> rank;
};

#endif // !EVALUATOR

