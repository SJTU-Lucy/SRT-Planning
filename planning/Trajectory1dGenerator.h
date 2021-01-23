#ifndef  GENERATOR
#define	 GENERATOR

#include <vector>
#include <iostream>

#include "Curve1d.h"

// ȡ���õ������ݺ�켣
class Trajectory1dGenerator
{
public:
    Trajectory1dGenerator(std::vector<double>& s, std::vector<double>& d);
    void set_max(double target);
    void set_min(double target);
    void set_dec(double target);
    void set_acc(double target);
    // ���ݳ�ʼ��Ϣ�������ֱ�õ��ݺ����ߣ��˴��Ĵ��������������¼���ݵģ��ǳ�ʼ������
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

