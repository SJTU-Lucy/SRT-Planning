#ifndef  TRAJECTORYPOINT
#define  TRAJECTORYPOINT

#include "PathPoint.h"

// �滮����Ϣ�������켣��Ϣ�Լ���Ӧ���ٶȡ����ٶ�
class TrajectoryPoint
{
public:
    PathPoint point;
    double v = 0.0;
    double a = 0.0;
    double relative_time = 0.0;
};

#endif // ! TRAJECTORYPOINT

