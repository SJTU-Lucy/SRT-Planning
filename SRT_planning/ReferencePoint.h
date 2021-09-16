#ifndef REFERENCEPOINT
#define REFERENCEPOINT

class ReferencePoint
{
public:
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double dkappa = 0.0;
    ReferencePoint() {};
    ReferencePoint(double X, double Y, double Theta, double Kappa)
    {
        x = X;
        y = Y;
        theta = Theta;
        kappa = Kappa;
    }
};

#endif // !REFERENCEPOINT
