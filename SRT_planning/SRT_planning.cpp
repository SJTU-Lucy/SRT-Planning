#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include <queue>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <algorithm>
#include <cmath>

#include "PathPoint.h"
#include "ReferencePoint.h"
#include "TrajectoryPoint.h"
#include "Curve1d.h"
#include "Trajectory1dGenerator.h"
#include "ADCTrajectory.h"
#include "TrajectoryDemo.h"
#include "TrajectoryEvaluator.h"
#include "TrajectoryCombiner.h"

#define DONE  1
#define PROCESSING  2
#define ERROR  3


// 获取当前系统时间
double NowInSeconds()
{
    time_t tt = time(NULL);
    tm* t = new tm;
    localtime_s(t, &tt);
    int ans = t->tm_sec + 60 * t->tm_min + t->tm_hour * 3600;
    // std::cout << t->tm_hour << " " << t->tm_min << " " << t->tm_sec << std::endl;
    return double(ans);
}

// 笛卡尔坐标系转换到frenet坐标系
void cartesian_to_frenet(
    double rs, double rx, double ry, double rtheta, double rkappa, const double rdkappa,
    double x, double y, double v, double a, double theta, double kappa,
    std::vector<double>& ptr_s_condition, std::vector<double>& ptr_d_condition){

    double dx = x - rx;
    double dy = y - ry;

    double cos_theta_r = std::cos(rtheta);
    double sin_theta_r = std::sin(rtheta);

    double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    ptr_d_condition[0] =
        std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);


    double delta_theta = theta - rtheta;

    std::cout << "delta theta = " << delta_theta << std::endl;

    double tan_delta_theta = std::tan(delta_theta);
    double cos_delta_theta = std::cos(delta_theta);

    double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition[0];
    ptr_d_condition[1] = one_minus_kappa_r_d * tan_delta_theta;

    const double kappa_r_d_prime =
        rdkappa * ptr_d_condition[0] + rkappa * ptr_d_condition[1];

    ptr_d_condition[2] =
        -kappa_r_d_prime * tan_delta_theta +
        one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
        (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

    ptr_s_condition[0] = rs;

    ptr_s_condition[1] = v * cos_delta_theta / one_minus_kappa_r_d;

    const double delta_theta_prime =
        one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
    ptr_s_condition[2] =
        (a * cos_delta_theta -
            ptr_s_condition[1] * ptr_s_condition[1] *
            (ptr_d_condition[1] * delta_theta_prime - kappa_r_d_prime)) /
        one_minus_kappa_r_d;
    // std::cout << "in here" << std::endl;
}

// 将非标准角度的范围规定至-pi~pi
double NormalizeAngle(double angle) {
    double a = angle;
    while (a < -3.14159)    a += 2.0 * 3.14159;
    while (a > 3.14159)     a -= 2.0 * 3.14159;
    return a; 
}

double slerp(double a0, double t0, double a1, double t1, double t) {
    double a0_n = NormalizeAngle(a0);
    double a1_n = NormalizeAngle(a1);
    double d = a1_n - a0_n;
    if (d > 3.14159)  d = d - 2.0 * 3.14159;
    else if (d < -3.14159) d = d + 2.0 * 3.14159;

    const double r = (t - t0) / (t1 - t0);
    const double a = a0_n + d * r;
    return NormalizeAngle(a);
}

// 插值法寻找对应点的估计位置
PathPoint InterpolateUsingLinearApproximation(PathPoint p0, PathPoint p1, double s) {
    double s0 = p0.s;
    double s1 = p1.s;

    PathPoint path_point;
    double weight = (s - s0) / (s1 - s0);
    double X = (1 - weight) * p0.x + weight * p1.x;
    double Y = (1 - weight) * p0.y + weight * p1.y;
    double Theta = slerp(p0.theta, p0.s, p1.theta, p1.s, s);
    double Kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
    double Dkappa = (1 - weight) * p0.dkappa + weight * p1.dkappa;
    path_point.x = X;
    path_point.y = Y;
    path_point.theta = Theta;
    path_point.kappa = Kappa;
    path_point.dkappa = Dkappa;
    path_point.s = s;
    return path_point;
}

// 找到映射点
PathPoint FindProjectionPoint(PathPoint p0, PathPoint p1, double x, double y) {
    double v0x = x - p0.x;
    double v0y = y - p0.y;

    double v1x = p1.x - p0.x;
    double v1y = p1.y - p0.y;

    double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
    double dot = v0x * v1x + v0y * v1y;

    double delta_s = dot / v1_norm;
    return InterpolateUsingLinearApproximation(p0, p1, p0.s + delta_s);
}

// 将目前坐标为x,y的点和参考线上的点进行匹配，返回参考线上的pathpoint
PathPoint MatchToPath(std::vector<PathPoint> reference_line, double x, double y)
{
    auto func_distance_square = [](PathPoint& point, double x, double y) {
            double dx = point.x - x;
            double dy = point.y - y;
            return dx * dx + dy * dy;
    };
    // 先寻找轨迹点中距离出发点最近的两个点
    double distance_min = func_distance_square(reference_line.front(), x, y);
    std::size_t index_min = 0;

    for (std::size_t i = 1; i < reference_line.size(); ++i) {
        double distance_temp = func_distance_square(reference_line[i], x, y);
        if (distance_temp < distance_min) {
            distance_min = distance_temp;
            index_min = i;
        }
    }

    std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
    std::size_t index_end =
        (index_min + 1 == reference_line.size()) ? index_min : index_min + 1;

    if (index_start == index_end) {
        return reference_line[index_start];
    }
    std::cout << "Info of matching points:" << std::endl;
    std::cout << reference_line[index_start].x << " " << reference_line[index_start].y << std::endl;
    std::cout << reference_line[index_end].x << " " << reference_line[index_end].y << std::endl;
    // 再根据这两个点去寻找匹配点的坐标
    return FindProjectionPoint(reference_line[index_start],
        reference_line[index_end], x, y);
}

// 采样，将参考曲线的散点转化为路径点，并且标注距离出发点的距离
std::vector<PathPoint> ToDiscretizedReferenceLine(
    std::vector<ReferencePoint>& ref_points) {
    double S = 0.0;
    std::vector<PathPoint> path_points;
    int count = 0;
    for (auto ref_point : ref_points) {
        PathPoint path_point;
        path_point.x = ref_point.x;
        path_point.y = ref_point.y;
        path_point.theta = ref_point.theta;
        path_point.kappa = ref_point.kappa;
        path_point.dkappa = ref_point.dkappa;

        if (!path_points.empty()) {
            double dx = path_point.x - path_points.back().x;
            double dy = path_point.y - path_points.back().y;
            S += sqrt(dx * dx + dy * dy);
        }
        path_point.s = S;
        path_points.push_back(std::move(path_point));
        count++;
    }

    return path_points;
}

// 根据规划起点和它对应的匹配点,推断得到frenet坐标系信息
// 即s,ds,dds;l,dl,ddl六个信息

void ComputeInitFrenetState(PathPoint& matched_point,
    TrajectoryPoint& planning_start_point,
    std::vector<double>& ptr_s,
    std::vector<double>& ptr_d){
    cartesian_to_frenet(
        matched_point.s, matched_point.x, matched_point.y,
        matched_point.theta, matched_point.kappa, matched_point.dkappa,
        planning_start_point.point.x, planning_start_point.point.y,
        planning_start_point.v, planning_start_point.a,
        planning_start_point.point.theta, planning_start_point.point.kappa,
        ptr_s, ptr_d);

};

// planning_init_point：起点
// frame: 其中这一帧的内容
// reference_line_info: 参考线信息
int PlanOnReferenceLine(
    TrajectoryPoint& planning_init_point, 
    std::vector<ReferencePoint>& ref_points,
    ADCTrajectory& ptr_computed_trajectory) {
    static int num_planning_cycles = 0;                 // 进行规划次数计数
    static int num_planning_succeeded_cycles = 0;       // 成功规划次数计数

    double start_time = NowInSeconds();
    double current_time = start_time;

     std::cout << "Number of planning cycles: " 
        << num_planning_cycles << " "
        << num_planning_succeeded_cycles << std::endl;

    ++num_planning_cycles;

    // 1.获得当前ReferencePoint形式的参考线，通过离散化，获得PathPoint形式
    std::vector<PathPoint> ptr_reference_line = ToDiscretizedReferenceLine(ref_points);
    std::cout << "Stage 1 : Discretizing finished" << std::endl << std::endl;

    // 2.使用规划原点匹配第一步离散化后的轨迹，找出匹配点matched_point
    PathPoint matched_point = MatchToPath( ptr_reference_line,
        planning_init_point.point.x, planning_init_point.point.y);
    std::cout << "init_point x = " << planning_init_point.point.x
        << " y = " << planning_init_point.point.y << std::endl;
    std::cout << "match point: x = " << matched_point.x 
        << " y = " << matched_point.y
        << " s =" << matched_point.s
        << " theta = " << matched_point.theta
        << std::endl;

    std::cout << "Stage 2 : Matching finished" << std::endl << std::endl;

    // 3.根据对应点坐标，计算Frenet坐标系下,出发点的s和d值信息，包含速度和加速度
    std::vector<double> init_s(3);
    std::vector<double> init_d(3);
    ComputeInitFrenetState(matched_point, planning_init_point, init_s, init_d);

    std::cout << "vector s: " << init_s[0] << " " << init_s[1] << " " << init_s[2] << std::endl;
    std::cout << "vector d: " << init_d[0] << " " << init_d[1] << " " << init_d[2] << std::endl;

    std::cout << "Stage 3 : Transverting finished" << std::endl;

    std::cout << "ReferenceLine and Frenet Conversion Time = "
        << (NowInSeconds() - current_time) * 1000 << std::endl << std::endl;
    current_time = NowInSeconds();

    // 4.生成横纵向轨迹：lon代表纵向，lat表示横向，生成纵横速度关于时间的四次函数
    Trajectory1dGenerator trajectory1d_generator(init_s, init_d);
    std::vector<Curve1d> lon_trajectory1d_bundle;
    std::vector<Curve1d> lat_trajectory1d_bundle;
    // 在此类中产生多种轨迹，传递到这两个vector中
    trajectory1d_generator.GenerateTrajectoryBundles(
       lon_trajectory1d_bundle, lat_trajectory1d_bundle);

    std::cout << "Stage 4 : Bundle Generation finished" << std::endl;

    std::cout << "Trajectory_Generation_Time = "
        << (NowInSeconds() - current_time) * 1000 << std::endl << std::endl;
    current_time = NowInSeconds();

    // 5.首先进行排列组合，之后评价轨迹，根据cost值排序，挑选cost最小的
    TrajectoryEvaluator trajectory_evaluator(init_s, init_d,
        lon_trajectory1d_bundle, lat_trajectory1d_bundle, ptr_reference_line);

    std::cout << "Trajectory_Evaluator_Construction_Time = "
        << (NowInSeconds() - current_time) * 1000 << std::endl;
    current_time = NowInSeconds();

    std::cout << "number of trajectory pairs = "
        << trajectory_evaluator.num_of_trajectory_pairs()
        << "  number_lon_traj = " << lon_trajectory1d_bundle.size()
        << "  number_lat_traj = " << lat_trajectory1d_bundle.size() << std::endl;

    std::cout << "Stage 5 : Evaluation finished" << std::endl << std::endl;
    
    // 6.选取符合条件的作为最终结果，并且记录不符合条件的cost较小总数
    int lon_vel_failure_count = 0;                  // 速度太大
    int lat_offset_failure_count = 0;               // 横向碰桩桶
    int turn_quick_failure_count = 0;
    int combined_constraint_failure_count = 0;      // 不符合的总数
    int num_lattice_traj = 0;

    // 若上一组不符合标准，持续选取
    while (trajectory_evaluator.has_more_trajectory_pairs()) {
        TrajectoryDemo trajectory_demo = trajectory_evaluator.next_top_trajectory_pair();
        double trajectory_pair_cost = trajectory_demo.cost();

        // 将横纵向的轨迹结合得到一个二维坐标系，在这里做一些检验看是否合格
        TrajectoryCombiner combined_trajectory;
        // 在combiner里面进行检验，并debug一些关键参数
        combined_trajectory.Combine(ptr_reference_line, trajectory_demo, init_s, init_d);

        // 若不合格，记录一下情况，然后重复循环
        if (!combined_trajectory.valid()) {
            ++combined_constraint_failure_count;
            switch (combined_trajectory.type()) {
            case 1: // 前进速度太大
                lon_vel_failure_count++;
                break;                                                       
            case 2: // 横向碰桩桶
                lat_offset_failure_count++;
                break;
            case 3: // 转弯速度过大
                turn_quick_failure_count++;
                break;
            }
            continue;
        }
        else
        {
            num_lattice_traj++;
            // 将得到的规划点放入到ptr_computed_trajectory中
            std::vector<TrajectoryPoint> ans = combined_trajectory.ans();
            ptr_computed_trajectory.points = ans;
            break;
        }
    }
    std::cout << "total failure count = " << combined_constraint_failure_count << std::endl;
    std::cout << "velocity failure count = " << lon_vel_failure_count << std::endl;
    std::cout << "collision failure count = " << lat_offset_failure_count << std::endl;
    std::cout<< "turning speed failure count = " << turn_quick_failure_count << std::endl;

    std::cout << "Trajectory_Generation_Time = "
        << (NowInSeconds() - current_time) * 1000 << std::endl;
    current_time = NowInSeconds();

    std::cout << "Stage 6 : Generation finished" << std::endl << std::endl;

    if (num_lattice_traj > 0) {
        std::cout << "Planning succeeded" << std::endl;
        num_planning_succeeded_cycles += 1;
        return DONE;
    }
    else {
        std::cout << "Planning failed"<< std::endl;
        return ERROR;
    }
}

// 在使用plan函数之前，需要先得出planning_start_point（根据上一次得到的轨迹路线）
// ref_points简单取道路中线，根据内外圈的桩桶坐标拟合曲线
// ptr_computed_trajectory为一个承载结果的值，不需要事先赋值

// 主函数，用来进行操作，输入参数为开始状态信息和此帧信息 
// 函数入口：通过for循环进入规划，执行函数PlanOnReferenceLine
// 在frame中给出ref_points即中线，在此处进行目标轨迹的生成
int Plan(TrajectoryPoint& planning_start_point,
    std::vector<ReferencePoint>& ref_points,
    ADCTrajectory& ptr_computed_trajectory) {

    // 调用规划函数，传入参考线，此帧信息以及开始点信息，进行处理
    int status = PlanOnReferenceLine(planning_start_point,
        ref_points, ptr_computed_trajectory);

    return status;
}

int main()
{
    TrajectoryPoint start;
    PathPoint start_point;
    // line planning
    /*
    start_point.x = 0.4;
    start_point.y = 2;
    start_point.theta = 1.57;
    start.path_point = start_point;
    start.V = 5;
    start.A = 1;
    start.relative_time = NowInSeconds();

    std::vector<ReferencePoint> ref;
    for (double i = 0; i < 200; ++i)
    {
        ReferencePoint tmp;
        double angle = i / 100.0;
        tmp.x = 0;
        tmp.y = i * 0.1;
        tmp.theta = 1.57;
        tmp.kappa = 0;
        tmp.dkappa = 0;
        ref.push_back(tmp);
    }
    */

    start_point.x = 19.5;
    start_point.y = 0;
    start_point.theta = 1.4;
    start.point = start_point;
    start.v = 4;
    start.a = 1;
    start.relative_time = NowInSeconds();
    std::vector<ReferencePoint> ref;
    for (double i = 0; i < 157; ++i)
    {
        ReferencePoint tmp;
        double angle = i / 100.0;
        tmp.x = 20 * std::cos(angle);
        tmp.y = 20 * std::sin(angle);
        tmp.theta = 1.57 + angle;
        tmp.kappa = 0.05;
        tmp.dkappa = 0;
        ref.push_back(tmp);
    }

    ADCTrajectory ans;
    Plan(start, ref, ans);

    return 0;
}
