#ifndef REFERENCEPOINT
#define REFERENCEPOINT

// 参考线上点信息，不含有路程信息
class ReferencePoint
{
public:
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double dkappa = 0.0;
};

#endif // !REFERENCEPOINT
