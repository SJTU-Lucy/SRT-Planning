#ifndef COMBINER
#define COMBINER

#define PATH 4.0
#define VEHICLE 2.0
#define SECURE 0.3
#define SPEEDLIMIT 10
#define CURVERATE 10

#include <vector>
#include "PathPoint.h"
#include "TrajectoryDemo.h"
#include "TrajectoryPoint.h"

// 对曲线进行一些检验
class TrajectoryCombiner
{
public:
    TrajectoryCombiner();
    void Combine(std::vector<PathPoint>& ptr_reference_line, TrajectoryDemo trajectory_pair,
        std::vector<double> s, std::vector<double> d);
    bool valid();
    int type();
    // 使用二维的frenet反向生成绝对坐标系中的坐标
    void generate();
    std::vector<TrajectoryPoint> ans();
private:
    std::vector<PathPoint> ref_line;
    TrajectoryDemo Trajectory_final;
    // 轨迹是否可行/错误种类
    bool Valid;
    int Type = 0;
    // 最终轨迹的信息
    double velocity;
    double acceleration;
    double ptr_x;
    double ptr_y;
    double ptr_theta;
    double ptr_kappa;

    std::vector<TrajectoryPoint> Points;
};

#endif // !COMBINER

