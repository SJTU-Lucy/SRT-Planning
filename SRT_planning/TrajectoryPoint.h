#ifndef  TRAJECTORYPOINT
#define  TRAJECTORYPOINT

#include "PathPoint.h"

class TrajectoryPoint
{
public:
    PathPoint point;
    double v = 0.0;
    double a = 0.0;
    double relative_time = 0.0;
};

#endif // ! TRAJECTORYPOINT

