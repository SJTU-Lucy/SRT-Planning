#ifndef ADC
#define ADC

#include <vector>
#include "TrajectoryPoint.h"

// final result being sent out in a msg
class ADCTrajectory
{
public:
    std::vector<TrajectoryPoint> points;
    int timestamp = 0;
};

#endif // !ADC

