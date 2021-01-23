#ifndef  TRAJECTORYPOINT
#define  TRAJECTORYPOINT

#include "PathPoint.h"

// 规划点信息，包含轨迹信息以及对应的速度、加速度
class TrajectoryPoint
{
public:
    PathPoint point;
    double v = 0.0;
    double a = 0.0;
    double relative_time = 0.0;
};

#endif // ! TRAJECTORYPOINT

