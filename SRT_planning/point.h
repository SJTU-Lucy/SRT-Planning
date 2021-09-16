#ifndef POINT
#define POINT

class Point
{
public:
	Point()
	{
		x = 0;
		y = 0;
	}
	Point(double X, double Y)
	{
		x = X;
		y = Y;
	}
	double x;
	double y;
};

#endif // !POINT

