#ifndef CURVE1D
#define CURVE1D

#include <vector>

// 曲线信息，用一个四次多项式表示，为速度关于时间函数，其他通过微分和积分再得到
class Curve1d
{
public:
    double x0;
    double x1;
    double x2;
    double x3;
    double x4;

    std::vector<double> init_s;
    std::vector<double> init_d;

    Curve1d();
    Curve1d(std::vector<double>& s, std::vector<double>& d);
    // 使用四次函数拟合曲线轨迹
    void calculate_s(double v_0, double a_0, double v_0p5, double v_1, double v_2);
    void calculate_d(double d_0, double v_0, double a_0, double t_0, double offset);
    double s(double t);
    double d(double t);
    double v(double t);
    double a(double t);
    // 计算曲线的最大速度和最小速度
    double max_speed();
    double min_speed();
    // 该轨迹在1s内前进的距离
    double forward();
    double avg_speed();
    void print();
};

#endif // !CURVE1D


