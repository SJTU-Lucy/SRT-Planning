#include "Trajectory1dGenerator.h"
#include <algorithm>

Trajectory1dGenerator::Trajectory1dGenerator(std::vector<double>& s,
    std::vector<double>& d)
{
    init_s = s;
    init_d = d;
}

// ���������������/���ټ��ٶ�
void Trajectory1dGenerator::set_dec(double target)
{ 
    dec_a = target;
}

void Trajectory1dGenerator::set_acc(double target)
{
    acc_a = target;
}

void Trajectory1dGenerator::set_max(double target)
{
    max_speed = target;
}

void Trajectory1dGenerator::set_min(double target)
{
    min_speed = target;
}

// ���ݳ�ʼ��Ϣ�������ֱ�õ��ݺ����ߣ��˴��Ĵ��������������¼���ݵģ��ǳ�ʼ������
void Trajectory1dGenerator::GenerateTrajectoryBundles(std::vector<Curve1d>& 
    lon_trajectory1d_bundle, std::vector<Curve1d>& lat_trajectory1d_bundle)
{
    // ��0,0.5,1,2�ĸ���õ�һ��ȷ�����Ĵκ������������Ϊ0ʱv,a�����Ϊv
    std::vector<std::vector<double>> s_sample;
    double start_speed = init_s[1];
    double start_acc = init_s[2];
    std::vector<double> tmp;

    double left;
    double right;

    // t = 0.5
    left = std::max(min_speed, start_speed + 0.5 * dec_a);
    right = std::min(max_speed, start_speed + 0.5 * acc_a);
    for (double i = left; i <= right; i += (right-left) / 3.0)
        tmp.push_back(i);
    s_sample.push_back(tmp);
    tmp.clear();
    // t = 1
    left = std::max(min_speed, start_speed + dec_a);
    right = std::min(max_speed, start_speed + acc_a);
    for (double i = left; i <= right; i += (right - left) / 3.0)
        tmp.push_back(i);
    s_sample.push_back(tmp);
    tmp.clear();
    // t = 2
    left = std::max(min_speed, start_speed + 2.0 * dec_a);
    right = std::min(max_speed, start_speed + 2.0 * acc_a);
    for (double i = left; i <= right; i += (right - left) / 3.0)
        tmp.push_back(i);
    s_sample.push_back(tmp);
    tmp.clear();

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            for (int k = 0; k < 4; ++k)
            {
                Curve1d ans(init_s, init_d);
                ans.calculate_s(start_speed, start_acc,
                    s_sample[0][i], s_sample[1][j], s_sample[2][k]);
                lon_trajectory1d_bundle.push_back(ans);
            }
        }
    }

    // ����滮
    double offset = -init_d[0];                  //��ʱ���ڵ�ƫ����
    for (double t = 5.0; t < 10.0; t++)
    {
        Curve1d ans(init_s, init_d);
        ans.calculate_d(init_d[0], init_d[1], init_d[2], t, offset);
        lat_trajectory1d_bundle.push_back(ans);
        
        // debug
        std::cout << "t0 = " << t << " Equation : ";
        if (ans.x2 > 0) std::cout << "+" << 0.33 * ans.x2 << "t^3";
        else if (ans.x2 < 0) std::cout << 0.333 * ans.x2 << "t^3";
        if (ans.x1 > 0) std::cout << "+" << 0.5 * ans.x1 << "t^2";
        else if (ans.x1 < 0) std::cout << 0.5 * ans.x1 << "t^2";
        if (ans.x0 > 0) std::cout << "+" << ans.x0 << "t";
        else if (ans.x0 < 0) std::cout << ans.x0 << "t";
        if (init_d[0] > 0) std::cout << "+" << init_d[0] << std::endl;
        else std::cout << init_d[0] << std::endl;
    }
}