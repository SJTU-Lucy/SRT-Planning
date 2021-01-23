#ifndef  GENERATOR
#define	 GENERATOR

#include <vector>
#include <iostream>

#include "Curve1d.h"

// 取样得到多种纵横轨迹
class Trajectory1dGenerator
{
public:
    Trajectory1dGenerator(std::vector<double>& s, std::vector<double>& d);
    void set_max(double target);
    void set_min(double target);
    void set_dec(double target);
    void set_acc(double target);
    // 根据初始信息，采样分别得到纵横曲线（此处的传入参数是用来记录数据的，非初始条件）
    void GenerateTrajectoryBundles(std::vector<Curve1d>& lon_trajectory1d_bundle,
        std::vector<Curve1d>& lat_trajectory1d_bundle);
private:
    double max_speed = 7.0;
    double min_speed = 0.0;
    double dec_a = -4.0;
    double acc_a = 4.0;
    std::vector<double> init_s;
    std::vector<double> init_d;
};

#endif // ! GENERATOR

