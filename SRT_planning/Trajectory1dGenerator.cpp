#include "Trajectory1dGenerator.h"
#include <algorithm>

Trajectory1dGenerator::Trajectory1dGenerator(double max_speed, std::vector<double>& s,
    std::vector<double>& d)
{
    this->max_speed = max_speed;
    init_s = s;
    init_d = d;
}

void Trajectory1dGenerator::GenerateTrajectoryBundles(bool loss, std::vector<Curve1d>& 
    lon_trajectory1d_bundle, std::vector<Curve1d>& lat_trajectory1d_bundle)
{
    // lon-bundle generation
    // take t=0, t=1, t=2 for Curve1d generation
    std::vector<std::vector<double>> s_sample;
    double start_speed = init_s[1];
    double start_acc = init_s[2];
    std::vector<double> tmp;

    double left;
    double right;
    // t = 1s
    left = std::max(min_speed, start_speed + dec_a);
    right = std::min(max_speed, start_speed + acc_a);
    for (double i = left; i <= right; i += (right - left) / 5.1)
        tmp.push_back(i);
    s_sample.push_back(tmp);
    tmp.clear();
    // t = 2s
    left = std::max(min_speed, start_speed + 2.0 * dec_a);
    right = std::min(max_speed, start_speed + 2.0 * acc_a);
    for (double i = left; i <= right; i += (right - left) / 5.1)
        tmp.push_back(i);   
     
    s_sample.push_back(tmp);
    tmp.clear();

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            Curve1d ans(init_s, init_d);
            ans.calculate_s(start_speed, s_sample[0][i], s_sample[1][j]);
            lon_trajectory1d_bundle.push_back(ans);
        };
    }
    // lat-bundle generation
    // when t=t0, the offset is cleared
    double offset;                 
    if (loss) offset = 0;
    else offset = -init_d[0];
    for (double t = 3.0; t < 7.0; t++)
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