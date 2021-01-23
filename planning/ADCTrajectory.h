#ifndef ADC
#define ADC

#include <vector>
#include "TrajectoryPoint.h"

// 最终获得的规划结果
class ADCTrajectory
{
public:
    std::vector<TrajectoryPoint> points;
};

#endif // !ADC

