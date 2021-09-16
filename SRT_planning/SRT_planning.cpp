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

#include <graphics.h>
#include <conio.h>

#include "PathPoint.h"
#include "ReferencePoint.h"
#include "TrajectoryPoint.h"
#include "Curve1d.h"
#include "Trajectory1dGenerator.h"
#include "ADCTrajectory.h"
#include "TrajectoryDemo.h"
#include "TrajectoryEvaluator.h"
#include "TrajectoryCombiner.h"
#include "point.h"
#include "ControlMsg.h"

#define DONE 1
#define PROCESSING 2
#define ERROR 3
#define MAXSPEED 5
// look forward rate: t = speed * LFK 
// e.g. 10m/s -> t = 1s
#define LFK 0.1

// whether is a serious loss of buckets
bool loss = false;

// System Time (modify in the Linux environment, achieving accuracy of ms)
double NowInSeconds()
{
    time_t tt = time(NULL);
    tm* t = new tm;
    localtime_s(t, &tt);
    int ans = t->tm_sec + 60 * t->tm_min + t->tm_hour * 3600;
    // std::cout << t->tm_hour << " " << t->tm_min << " " << t->tm_sec << std::endl;
    return double(ans);
}

// Cartesian->frenet
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

// a helper function for cartesian_to_frenet
void ComputeInitFrenetState(PathPoint& matched_point,
    TrajectoryPoint& planning_start_point,
    std::vector<double>& ptr_s,
    std::vector<double>& ptr_d) {
    cartesian_to_frenet(
        matched_point.s, matched_point.x, matched_point.y,
        matched_point.theta, matched_point.kappa, matched_point.dkappa,
        planning_start_point.point.x, planning_start_point.point.y,
        planning_start_point.v, planning_start_point.a,
        planning_start_point.point.theta, planning_start_point.point.kappa,
        ptr_s, ptr_d);

};

// range an angle to [-pi,pi]
double NormalizeAngle(double angle) {
    double a = angle;
    while (a < -3.14159)    a += 2.0 * 3.14159;
    while (a > 3.14159)     a -= 2.0 * 3.14159;
    return a; 
}

// get the angle for matched_point
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

// weighted-average the two points, calculate information for PathPoint
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

// Using projection rule to find the point
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

// Find a matching point in the reference line for current point, return a pathpoint
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

// discrete the reference line, adding a parament of s(distance)
std::vector<PathPoint> ToDiscretizedReferenceLine(
    std::vector<ReferencePoint>& ref_points) {
    double S = 0.0;
    std::vector<PathPoint> path_points;
    int count = 0;
    
    for (int i = 0; i < ref_points.size(); i++) {
        ReferencePoint ref_point = ref_points[i];
        PathPoint path_point;
        path_point.x = ref_point.x;
        path_point.y = ref_point.y;
        path_point.theta = ref_point.theta;
        path_point.kappa = ref_point.kappa;
        if (!path_points.empty()) {
            double dx = path_point.x - path_points.back().x;
            double dy = path_point.y - path_points.back().y;
            S += sqrt(dx * dx + dy * dy);
        }
        path_point.s = S;
        path_points.push_back(path_point);
        count++;
    }
    // 验证dkappa的结果是否准确
    //for (int i = 0; i < path_points.size() - 1; i++)
    //{
    //    double ds = path_points[i + 1].s - path_points[i].s;
    //    double dk = path_points[i + 1].kappa - path_points[i].kappa;
    //    path_points[i].dkappa = dk / ds;
    //}
    //path_points[path_points.size() - 1] = path_points[path_points.size() - 2];

    return path_points;
}


int PlanOnReferenceLine(
    TrajectoryPoint& planning_init_point, 
    std::vector<ReferencePoint>& ref_points,
    ADCTrajectory& ptr_computed_trajectory) {
    static int num_planning_cycles = 0;                 // attempted planning counter
    static int num_planning_succeeded_cycles = 0;       // successful planning counter

    double start_time = NowInSeconds();
    double current_time = start_time;

     std::cout << "Number of planning cycles: " 
        << num_planning_cycles << " "
        << num_planning_succeeded_cycles << std::endl;

    ++num_planning_cycles;

    // 1. Discretize the reference line of the ReferencePoints to PathPoints
    std::vector<PathPoint> ptr_reference_line = ToDiscretizedReferenceLine(ref_points);
    std::cout << "Stage 1 : Discretizing finished" << std::endl << std::endl;

    // 2. Use the position of the vehicle, to match a point in the reference line
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

    // 3. performs coordinate system conversion, cartesian->Frenet
    std::vector<double> init_s(3);
    std::vector<double> init_d(3);
    ComputeInitFrenetState(matched_point, planning_init_point, init_s, init_d);

    std::cout << "vector s: " << init_s[0] << " " << init_s[1] << " " << init_s[2] << std::endl;
    std::cout << "vector d: " << init_d[0] << " " << init_d[1] << " " << init_d[2] << std::endl;

    std::cout << "Stage 3 : Transverting finished" << std::endl;

    std::cout << "ReferenceLine and Frenet Conversion Time = "
        << (NowInSeconds() - current_time) * 1000 << std::endl << std::endl;
    current_time = NowInSeconds();
    
    // 4. generat horizontal(lon) and vertical(lat) trajectories
    Trajectory1dGenerator trajectory1d_generator(MAXSPEED, init_s, init_d);
    std::vector<Curve1d> lon_trajectory1d_bundle;
    std::vector<Curve1d> lat_trajectory1d_bundle;
    // the two vectors receive generated trajectories
    trajectory1d_generator.GenerateTrajectoryBundles(loss,
       lon_trajectory1d_bundle, lat_trajectory1d_bundle);

    std::cout << "Stage 4 : Bundle Generation finished" << std::endl;

    std::cout << "Trajectory_Generation_Time = "
        << (NowInSeconds() - current_time) * 1000 << std::endl << std::endl;
    current_time = NowInSeconds();

    // 5. permute and combine the trajectories, evaluate all the trajectories and sort them
    TrajectoryEvaluator trajectory_evaluator(init_s, init_d,
        lon_trajectory1d_bundle, lat_trajectory1d_bundle, ptr_reference_line);

    std::cout << "Trajectory_Evaluator_Construction_Time = "
        << (NowInSeconds() - current_time) * 1000 << std::endl;
    current_time = NowInSeconds();

    std::cout << "Stage 5 : Evaluation finished" << std::endl << std::endl;
    
    // 6. Choose the best one till matching out demands, keeps counting
    int lon_vel_failure_count = 0;                  // 速度太大
    int lat_offset_failure_count = 0;               // 横向碰桩桶
    int turn_quick_failure_count = 0;
    int combined_constraint_failure_count = 0;      // 不符合的总数
    int num_lattice_traj = 0;

    // Choose till it's okay
    while (trajectory_evaluator.has_more_trajectory_pairs()) {
        TrajectoryDemo trajectory_demo = trajectory_evaluator.next_top_trajectory_pair();
        double trajectory_pair_cost = trajectory_demo.cost();

        // combine lon&lat trajectories and examine the path
        TrajectoryCombiner combined_trajectory(MAXSPEED, 20);
        combined_trajectory.Combine(ptr_reference_line, trajectory_demo, init_s, init_d, loss);

        // if not okay, record and move forward
        if (!combined_trajectory.valid()) {
            ++combined_constraint_failure_count;
            switch (combined_trajectory.type()) {
            case 1: // too quick
                lon_vel_failure_count++;
                std::cout << "too quick" << std::endl;
                break;                                                       
            case 2: // collision with buckets
                lat_offset_failure_count++;
                std::cout << "collision" << std::endl;
                break;
            case 3: // speed is too high for this corner
                turn_quick_failure_count++;
                std::cout << "fast turning" << std::endl;
                break;
            }
            continue;
        }
        else
        {
            num_lattice_traj++;
            // return the acquired trajectory points and return
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

/*
Input :
planning_init_point：initial position of the vehicle
ref_points: the reference points of the track
ptr_computed_trajectory: an empty vetor to receive the answer
*/
int Plan(TrajectoryPoint& planning_start_point,
    std::vector<ReferencePoint>& ref_points,
    ADCTrajectory& ptr_computed_trajectory) {

    // 调用规划函数，传入参考线，此帧信息以及开始点信息，进行处理
    int status = PlanOnReferenceLine(planning_start_point,
        ref_points, ptr_computed_trajectory);

    return status;
}

// a solution of urgent stopping, used when too few buckets are detected or other commands
void UrgentStop(TrajectoryPoint& planning_start_point,
    ADCTrajectory& ptr_computed_trajectory) {

    double X = planning_start_point.point.x;
    double Y = planning_start_point.point.y;
    double Theta = planning_start_point.point.theta;
    double V = planning_start_point.v;
    double dec_a = -8.0;
    std::vector<TrajectoryPoint> pts;
    for (double t = 0; t < 1.1; t += 0.1)
    {
        double s = V * t + 0.5 * dec_a * t * t;
        TrajectoryPoint tmp;
        tmp.a = dec_a;
        tmp.relative_time = t;
        tmp.v = V + dec_a * t;
        tmp.point.x = X + std::cos(Theta) * s;
        tmp.point.y = Y + std::sin(Theta) * s;
        tmp.point.theta = Theta;
        tmp.point.kappa = 0;
        tmp.point.dkappa = 0;
        tmp.point.s = s;
        pts.push_back(tmp);
    }
    ptr_computed_trajectory.points = pts;
}
     
// Bezier Curve: P = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3(1-t)*t^2*P2 + t^3*P3 

double bezier3funcX(double t, Point* controlP) {
    double part0 = controlP[0].x * (1 - t) * (1 - t) * (1 - t);
    double part1 = 3 * controlP[1].x * (1 - t) * (1 - t) * t;
    double part2 = 3 * controlP[2].x * (1 - t) * t * t;
    double part3 = controlP[3].x * t * t * t;
    return part0 + part1 + part2 + part3;
}

double bezier3funcY(double t, Point* controlP) {
    double part0 = controlP[0].y * (1 - t) * (1 - t) * (1 - t);
    double part1 = 3 * controlP[1].y * (1 - t) * (1 - t) * t;
    double part2 = 3 * controlP[2].y * (1 - t) * t * t;
    double part3 = controlP[3].y * t * t * t;
    return part0 + part1 + part2 + part3;
}

double dx(double t, Point* controlP) {
    double part0 = controlP[0].x * (-3 * t * t + 6 * t - 3);
    double part1 = 3 * controlP[1].x * (3 * t * t - 4 * t + 1);
    double part2 = 3 * controlP[2].x * (2 * t - 3 * t * t);
    double part3 = controlP[3].x * 3 * t * t;
    return part0 + part1 + part2 + part3;
}

double dy(double t, Point* controlP) {
    double part0 = controlP[0].y * (-3 * t * t + 6 * t - 3);
    double part1 = 3 * controlP[1].y * (3 * t * t - 4 * t + 1);
    double part2 = 3 * controlP[2].y * (2 * t - 3 * t * t);
    double part3 = controlP[3].y * 3 * t * t;
    return part0 + part1 + part2 + part3;
}

double ddx(double t, Point* controlP) {
    double part0 = controlP[0].x * (-6 * t + 6);
    double part1 = 3 * controlP[1].x * (6 * t - 4);
    double part2 = 3 * controlP[2].x * (2 - 6 * t);
    double part3 = controlP[3].x * 6 * t;
    return part0 + part1 + part2 + part3;
}

double ddy(double t, Point* controlP) {
    double part0 = controlP[0].y * (-6 * t + 6);
    double part1 = 3 * controlP[1].y * (6 * t - 4);
    double part2 = 3 * controlP[2].y * (2 - 6 * t);
    double part3 = controlP[3].y * 6 * t;
    return part0 + part1 + part2 + part3;
}  

double bezier3theta(double t, Point* controlP)
{
    double dX = dx(t, controlP);
    double dY = dy(t, controlP);
    double theta = std::atan2(dY, dX);
    // std::cout << "t = " << t << " ref theta = " << theta << std::endl;
    return theta;
}

double bezier3kappa(double t, Point* controlP)
{
    double dX = dx(t, controlP);
    double dY = dy(t, controlP);
    double ddX = ddx(t, controlP);
    double ddY = ddy(t, controlP);
    double ret = std::abs(dX * ddY - ddX * dY) / std::pow(dX * dX + dY * dY, 1.5);
    std::cout << "t = " << t << " ref kappa = " << ret << std::endl;
    return ret;
}

void createCurve(std::vector<Point> originPoint, int originCount, std::vector<ReferencePoint>& curvePoint) {
    // 控制点收缩系数
    double scale = 0.6;                                                                
    std::vector<Point> midpoints(originCount - 1);
    // 生成中点      
    for (int i = 0; i < originCount - 1; i++) {
        int nexti = i + 1;
        midpoints[i].x = (originPoint[i].x + originPoint[nexti].x) / 2.0;
        midpoints[i].y = (originPoint[i].y + originPoint[nexti].y) / 2.0;
    }
    // 平移中点 
    std::vector<Point> extrapoints(2 * originCount - 4);
    for (int i = 0; i < originCount - 2; i++) {
        int nexti = i + 1;                                   
        Point midinmid;
        midinmid.x = (midpoints[i].x + midpoints[nexti].x) / 2.0;
        midinmid.y = (midpoints[i].y + midpoints[nexti].y) / 2.0;
        double offsetx = originPoint[i + 1].x - midinmid.x;
        double offsety = originPoint[i + 1].y - midinmid.y;

        int extraindex = 2 * i;
        extrapoints[extraindex].x = midpoints[i].x + offsetx;
        extrapoints[extraindex].y = midpoints[i].y + offsety;
        // 朝 originPoint[i]方向收缩  
        double addx = (extrapoints[extraindex].x - originPoint[i + 1].x) * scale;
        double addy = (extrapoints[extraindex].y - originPoint[i + 1].y) * scale;
        extrapoints[extraindex].x = extrapoints[extraindex].x + addx;
        extrapoints[extraindex].y = extrapoints[extraindex].y + addy;

        int extranexti = extraindex + 1;
        extrapoints[extranexti].x = midpoints[nexti].x + offsetx;
        extrapoints[extranexti].y = midpoints[nexti].y + offsety;
        //朝 originPoint[i]方向收缩  
        addx = (extrapoints[extranexti].x - originPoint[i + 1].x) * scale;
        addy = (extrapoints[extranexti].y - originPoint[i + 1].y) * scale;
        extrapoints[extranexti].x = extrapoints[extranexti].x + addx;
        extrapoints[extranexti].y = extrapoints[extranexti].y + addy;
    }

    Point controlPoint[4];
    for (int i = 1; i < originCount - 2; i++) {
        controlPoint[0] = originPoint[i];
        int extraindex = 2 * i - 1;
        controlPoint[1] = extrapoints[extraindex];
        controlPoint[2] = extrapoints[extraindex + 1];
        int nexti = i + 1;
        controlPoint[3] = originPoint[nexti];
        double t = 0.25;
        double pre_kappa;   
        double pre_x;
        double pre_y;
        while (t <= 1) {
            double px = bezier3funcX(t, controlPoint);
            double py = bezier3funcY(t, controlPoint);
            double pkappa = bezier3kappa(t, controlPoint);
            double ptheta = bezier3theta(t, controlPoint);
            ReferencePoint tempP = ReferencePoint(px, py, ptheta, pkappa);
            if (t == 0) {
                pre_kappa = pkappa;
                pre_x = px;
                pre_y = py;
            }
            else{
                pre_kappa = pkappa;
                pre_x = px;
                pre_y = py;
            }
            t += 0.5;
            curvePoint.push_back(tempP);
        }
    }

}

// TODO
void edge_filter(std::vector<Point>& edge)
{
    // 对left&right进行处理，排除显著的识别错误
    // 并且根据车辆的当前位置对桩桶进行简单的排序
}
// TODO
void midpoint_filter(std::vector<Point>& ref)
{
    // 对reference进行过滤or调整，使得整体轨迹合理
    // 比如目前在一个左转的弯道，但是其中有一些偏离轨迹的点，需要进行矫正
    ref.clear();
    for (double i = 180; i >= 0; i -= 6)
    {
        double angle = i / 180 * 3.14;
        Point tmp;
        tmp.x = 28.875 + 9.125 * std::cos(angle);
        tmp.y = 20 + 9.125 * std::sin(angle);
        ref.push_back(tmp);
    }
}
// TODO
void ref_denser(std::vector<ReferencePoint>& ref)
{
    // 将得到的midpoint稠密化，暂时直接取直线
    int size = ref.size();
    std::vector<ReferencePoint> ret;
    for (int i = 0; i < size - 1; i++)
    {
        ReferencePoint tmp;
        ret.push_back(ref[i]);
        for (int j = 1; j < 5; j += 1)
        {
            tmp.x = (ref[i + 1].x - ref[i].x) * j / 5 + ref[i].x;
            tmp.y = (ref[i + 1].y - ref[i].y) * j / 5 + ref[i].y;
            tmp.theta = (ref[i + 1].theta - ref[i].theta) * j / 5 + ref[i].theta;
            tmp.kappa = (ref[i + 1].kappa - ref[i].kappa) * j / 5 + ref[i].kappa;
            tmp.dkappa = (ref[i + 1].dkappa - ref[i].dkappa) * j / 5 + ref[i].dkappa;
            ret.push_back(tmp);
        }
    }
    ret.push_back(ref[size - 1]);
    ref = ret;
}

// using left cones and right cones to generate the reference line
// if loss == true, then just consider one side containing more points as the reference line 
std::vector<ReferencePoint> reference(std::vector<Point> left, std::vector<Point> right, bool loss)
{
    int l = left.size();
    int r = right.size();
    int i = 0;
    int j = 0;
    std::vector<Point> points;
    if (loss)
    {
        if (left.size() <= 2)
        {
            for (int i = 1; i < right.size(); ++i)
            {
                points.push_back(right[i - 1]);
                Point tmp;
                tmp.x = (right[i - 1].x + right[i].x) / 2;
                tmp.y = (right[i - 1].y + right[i].y) / 2;
                points.push_back(tmp);
            }
            points.push_back(right[right.size() - 1]);
        }
        else
        {
            for (int i = 1; i < left.size(); ++i)
            {
                points.push_back(left[i - 1]);
                Point tmp;
                tmp.x = (left[i - 1].x + left[i].x) / 2;
                tmp.y = (left[i - 1].y + left[i].y) / 2;
                points.push_back(tmp);
            }
            points.push_back(left[left.size() - 1]);
        }
    }
    else
    {
        while (i < l && j < r)
        {
            if (i == j)
            {
                Point tmp;
                tmp.x = (left[i].x + right[j].x) / 2;
                tmp.y = (left[i].y + right[j].y) / 2;
                points.push_back(tmp);
                j++;
            }
            else
            {
                Point tmp;
                tmp.x = (left[i].x + right[j].x) / 2;
                tmp.y = (left[i].y + right[j].y) / 2;
                points.push_back(tmp);
                i++;
            }
        }
    }
    midpoint_filter(points);
    std::vector<ReferencePoint> curvePoint;
    createCurve(points, points.size(), curvePoint);
    return curvePoint;
}

int FindNearestTime(double t) {
    std::cout << "t = " << t << std::endl;
    double t1 = 0.0;
    double t2 = 0.05;
    int index;
    while (t2 <= 1.0)
    {
        if (t == t2) 
        {
            index = t * 20;
            break;
        }
        if (t > t1&& t < t2)
        {
            if (t - t1 <= t2 - t) index = t1 * 20;
            else t2 * 20;
            break;
        }
        else
        {
            t1 += 0.05;
            t2 += 0.05;
        }
    }
    std::cout << "Index = " << index << "\n";
    return index;
}

void PurePursuit(TrajectoryPoint& current_point, ADCTrajectory& route, ControlMsg& ctr_msg) {
    // angle = arctan(2Lsin(alpha)/ld)
    // L: distance between front and rear axle
    // alpha: angle of the target point for vehicle
    // ld: look forward distance decided by t(rear axle to target point)

    double L = 1.6;  // front axle -> rear axle
    double camera = 0.5;  // camera-> rear axle

    double current_angle = current_point.point.theta;
    double current_x = current_point.point.x - camera * sin(current_angle);
    double current_y = current_point.point.y - camera * cos(current_angle);
    double current_v = current_point.v;
    std::cout << "current:" << current_x << " " << current_y << std::endl;

    double index = FindNearestTime(current_v * LFK);
    double tar_x = route.points[index].point.x;
    double tar_y = route.points[index].point.y;
    double tar_v = route.points[index].v;
    std::cout << "target:" << tar_x << " " << tar_y << std::endl;

    double angle = atan2(tar_y - current_y, tar_x - current_x);
    double alpha = angle - current_angle;
    double ld = std::pow((tar_x - current_x) * (tar_x - current_x) 
        + (tar_y - current_y) * (tar_y - current_y), 0.5);

    std::cout << "alpha = " << alpha << '\n';
    std::cout << "ld = " << ld << '\n';

    ctr_msg.angle = atan2(2 * L * sin(alpha), ld);
    ctr_msg.speed = tar_v;
}

/*
remains to be done: 
1. detecting yellow cone
2. change some #Define in TrajectoryCombiner
3. adding a filter for input cones to prevent possible mistakes
4. try to use Delaunay Triangulation when generating reference line
5. finish ref_denser & midpoint_filter & edge_filter
*/

int main()
{
    TrajectoryPoint start;
    PathPoint start_point;
    ADCTrajectory ans;

    std::vector<Point> left;
    std::vector<Point> right;
    std::vector<Point> yellow;
    Point tmp;

    // 1-2圈
    for (double i = 180; i >= 0; i -= 30)
    {
        double angle = i / 180 * 3.14;
        Point tmp;
        tmp.x = 28.875 + 7.625 * std::cos(angle);
        tmp.y = 20 + 7.625 * std::sin(angle);
        right.push_back(tmp);
        tmp.x = 28.875 + 10.625 * std::cos(angle);
        tmp.y = 20 + 10.625 * std::sin(angle);
        left.push_back(tmp);
    }
    start_point.x = 28.875;
    start_point.y = 29.6;
    start_point.theta = 0;
    start.point = start_point;
    start.v = 4.6;
    start.a = 1;
    start.relative_time = 0;
    edge_filter(left);
    edge_filter(right);
    initgraph(900, 900);

    std::vector<TrajectoryPoint> ret;
    if (left.size() <= 2 && right.size() <= 2)
    {
        UrgentStop(start, ans);
        ret = ans.points;
        for (unsigned int i = 0; i < left.size(); ++i) circle(20.0 * left[i].x, 20.0 * left[i].y, 2);
        for (unsigned int i = 0; i < right.size(); ++i) circle(20.0 * right[i].x, 20.0 * right[i].y, 2);
        for (unsigned int i = 0; i < ret.size(); ++i) circle(20.0 * ret[i].point.x, 20.0 * ret[i].point.y, 1);
    }
    else
    {
        if (left.size() <= 2 || right.size() <= 2) 
            loss = true;
        std::vector<ReferencePoint> ref = reference(left, right, loss);
        ref_denser(ref);
        Plan(start, ref, ans);
        ret = ans.points;

        for (unsigned int i = 0; i < ref.size(); ++i) circle(20.0 * ref[i].x, 20.0 * ref[i].y, 1);
        for (unsigned int i = 0; i < left.size(); ++i) circle(20.0 * left[i].x, 20.0 * left[i].y, 2);
        for (unsigned int i = 0; i < right.size(); ++i) circle(20.0 * right[i].x, 20.0 * right[i].y, 2);
        for (unsigned int i = 0; i < ret.size(); ++i) circle(20.0 * ret[i].point.x, 20.0 * ret[i].point.y, 1);
    }

    ControlMsg ctr_msg;             
    PurePursuit(start, ans, ctr_msg);
    std::cout << "Pure Pursuit Test: \n";
    std::cout << "Speed = " << ctr_msg.speed << "\n";
    std::cout << "Angle = " << ctr_msg.angle * 90 / 1.57 << "\n";

    _getch();
    closegraph();
    return 0;
}