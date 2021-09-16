#include "TrajectoryCombiner.h"
#include <cmath>
#include <iostream>

TrajectoryCombiner::TrajectoryCombiner() 
{
    Valid = true;
    Type = 0;
}

TrajectoryCombiner::TrajectoryCombiner(double speedlimit, double curverate)
{
    SPEEDLIMIT = speedlimit;
    CURVERATE = curverate;
}

void TrajectoryCombiner::Combine(std::vector<PathPoint>& ptr_reference_line, TrajectoryDemo trajectory_pair,
    std::vector<double> s, std::vector<double> d, bool loss)
{
    ref_line = ptr_reference_line;
    Trajectory_final = trajectory_pair;
    Curve1d lon_curve = Trajectory_final.lon();
    Curve1d lat_curve = Trajectory_final.lat();
    double offset = 0.33 * lat_curve.x2 + 0.5 * lat_curve.x1 + lat_curve.x0 + d[0];
    // debug
    std::cout << "max speed = " << lon_curve.max_speed() << std::endl;
    std::cout << "min speed = " << lon_curve.min_speed() << std::endl;
    std::cout << "d0 =  " << d[0] << std::endl;
    std::cout << "Chosen " << "Lon equation : ";
    if (lon_curve.x2 > 0) std::cout << "+" << 0.333 * lon_curve.x2 << "t^3";
    else if (lon_curve.x2 < 0) std::cout << 0.333 * lon_curve.x2 << "t^3";
    if (lon_curve.x1 > 0) std::cout << "+" << 0.5 * lon_curve.x1 << "t^2";
    else if (lon_curve.x1 < 0) std::cout << 0.5 * lon_curve.x1 << "t^2";
    if (lon_curve.x0 > 0) std::cout << "+" << lon_curve.x0 << "t";
    else if (lon_curve.x0 < 0) std::cout << lon_curve.x0 << "t";
    if (s[0] > 0) std::cout << "+" << s[0] << std::endl;
    else std::cout << s[0] << std::endl;
    std::cout << "Chosen " << "Lat equation : ";
    if (lat_curve.x2 > 0) std::cout << "+" << 0.333 * lat_curve.x2 << "t^3";
    else if (lat_curve.x2 < 0) std::cout << 0.333 * lat_curve.x2 << "t^3";
    if (lat_curve.x1 > 0) std::cout << "+" << 0.5 * lat_curve.x1 << "t^2";
    else if (lat_curve.x1 < 0) std::cout << 0.5 * lat_curve.x1 << "t^2";
    if (lat_curve.x0 > 0) std::cout << "+" << lat_curve.x0 << "t";
    else if (lat_curve.x0 < 0) std::cout << lat_curve.x0 << "t";
    if (d[0] > 0) std::cout << "+" << d[0] << std::endl;
    else std::cout << d[0] << std::endl;
    std::cout << "offset " << offset << std::endl << std::endl;
    // check min & max speed
    if (lon_curve.max_speed() >= SPEEDLIMIT * 1.1 || lon_curve.min_speed() <= 0)
    {
        Valid = false;
        Type = 1;
        return;
    }
    // examine whether it's too close to the buckets after 1 seconds
    if (!loss)
    {
        if (offset + 0.5 * VEHICLE >= 0.5 * PATH - SECURE || offset - 0.5 * VEHICLE <= -0.5 * PATH + SECURE)
        {
            Valid = false;
            Type = 2;
            return;
        }
    }

    generate();

    // examine if it's turning too fast
    for (unsigned int i = 0; i < Points.size(); ++i)
    {
        double v = Points[i].v;
        double kappa = Points[i].point.kappa;
        if (v * v * kappa > CURVERATE)
        {
            Valid = false;
            Type = 3;
            return;
        }
    }
    Valid = true;
    Type = 0;
}

bool TrajectoryCombiner::valid()
{
    return Valid;
}
int TrajectoryCombiner::type()
{
    return Type;
}

// Frenet->Cartesian
void TrajectoryCombiner::generate()
{
    std::vector<TrajectoryPoint> ret;
    unsigned int j = 0;
    for (double t = 0; t < 1.01; t += 0.05)
    {
        double S = Trajectory_final.lon().s(t);
        while (ref_line[j].s + 0.05 - S < 0 && j < ref_line.size())  j++;
        TrajectoryPoint tmp_Tra;
        PathPoint tmp_point;
        double mat_x = ref_line[j].x;
        double mat_y = ref_line[j].y;
        double mat_theta = ref_line[j].theta;
        double mat_kappa = ref_line[j].kappa;
        double mat_dkappa = ref_line[j].dkappa;
        double mat_s= ref_line[j].s;
        double d = Trajectory_final.lat().d(t);
        double d_ = Trajectory_final.lat().v(t);
        double d__ = Trajectory_final.lat().a(t);
        double s = Trajectory_final.lon().s(t);
        double s_ = Trajectory_final.lon().v(t);
        double s__ = Trajectory_final.lon().a(t);
        double l_ = d_ / s_;
        double l__ = (d__ * s_ - d_ * s__) / pow(s_, 3.0);

        std::cout << "t = " << t << std::endl
            << "s = " << Trajectory_final.lon().s(t)
            << " s' = " << Trajectory_final.lon().v(t)
            << " s\" = " << Trajectory_final.lon().a(t)
            << " d = " << Trajectory_final.lat().d(t)
            << " d' = " << Trajectory_final.lat().v(t)
            << " d\" = " << Trajectory_final.lat().a(t)
            << " l' = " << l_
            << " l\" = " << l__
            << std::endl;

        ptr_x = mat_x - d * std::sin(mat_theta) + (S - ref_line[j].s) * std::cos(mat_theta);
        ptr_y = mat_y + d * std::cos(mat_theta) + (S - ref_line[j].s) * std::sin(mat_theta);
        ptr_theta = std::atan(l_ / (1.0 - mat_kappa * d)) + mat_theta;
        while (ptr_theta < -3.14159)    ptr_theta += 2.0 * 3.14159;
        while (ptr_theta > 3.14159)     ptr_theta -= 2.0 * 3.14159;
        ptr_kappa = (std::abs(l__ + (mat_dkappa * d + mat_kappa * l_) * std::tan(ptr_theta - mat_theta))
            * (pow(std::cos(ptr_theta - mat_theta), 2.0) / (1 - mat_kappa * d)) + mat_kappa)
            * std::cos(ptr_theta - mat_theta) / (1.0 - mat_kappa * d);
        velocity = std::sqrt(pow(s_ * (1 - mat_kappa * d), 2.0) + pow(s_ * l_, 2.0));
        acceleration = s__ * ((1.0 - mat_kappa * d) / std::cos(ptr_theta - mat_theta))
            + pow(s_, 2.0) / std::cos(ptr_theta - mat_theta)
            * (l_ * (ptr_kappa * (1.0 - mat_kappa * d) / std::cos(ptr_theta - mat_theta) - mat_kappa)
                - (mat_dkappa * d + mat_kappa * l_));

        // debug
        std::cout << "match_x: " << mat_x << "  match_y: " << mat_y << "  match_theta: "
            << mat_theta << "  match_kappa: " << mat_kappa << " match_s: " 
            << mat_s << " dkappa:" << ptr_kappa << std::endl
            << "x: " << ptr_x << "  y: " << ptr_y
            << "  theta: " << ptr_theta << "  kappa: " << ptr_kappa 
            << "  v: " << velocity << "  a: " << acceleration
            << std::endl << std::endl;

        // record and return
        tmp_Tra.v = velocity;
        tmp_Tra.a = acceleration;
        tmp_point.s = Trajectory_final.lon().s(t);
        tmp_point.x = ptr_x;
        tmp_point.y = ptr_y;
        tmp_point.theta = ptr_theta;
        tmp_point.kappa = ptr_kappa;
        tmp_Tra.relative_time = t;
        tmp_Tra.point = tmp_point;
        ret.push_back(tmp_Tra);
        j++;
    }
    Points = ret;
}

std::vector<TrajectoryPoint> TrajectoryCombiner::ans()
{
    return Points;
}