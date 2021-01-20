#ifndef EVALUATOR
#define EVALUATOR

#include <vector>
#include <algorithm>
#include "PathPoint.h"
#include "Curve1d.h"
#include "TrajectoryDemo.h"

// 进行轨迹的组合和排序
class TrajectoryEvaluator
{
public:
    TrajectoryEvaluator(std::vector<double>& s, std::vector<double>& d,
        std::vector<Curve1d>& lon_trajectory1d_bundle,
        std::vector<Curve1d>& lat_trajectory1d_bundle,
        std::vector<PathPoint>& ptr_reference_line);
    // 是否还有轨迹
    bool has_more_trajectory_pairs();
    // 规划的总线路数
    int num_of_trajectory_pairs();
    // 下一个最佳匹配方案
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

