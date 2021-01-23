#ifndef COMBINER
#define COMBINER

#define PATH 4.0
#define VEHICLE 2.0
#define SECURE 0.3
#define SPEEDLIMIT 10
#define CURVERATE 10

#include <vector>
#include "PathPoint.h"
#include "TrajectoryDemo.h"
#include "TrajectoryPoint.h"

// �����߽���һЩ����
class TrajectoryCombiner
{
public:
    TrajectoryCombiner();
    void Combine(std::vector<PathPoint>& ptr_reference_line, TrajectoryDemo trajectory_pair,
        std::vector<double> s, std::vector<double> d);
    bool valid();
    int type();
    // ʹ�ö�ά��frenet�������ɾ�������ϵ�е�����
    void generate();
    std::vector<TrajectoryPoint> ans();
private:
    std::vector<PathPoint> ref_line;
    TrajectoryDemo Trajectory_final;
    // �켣�Ƿ����/��������
    bool Valid;
    int Type = 0;
    // ���չ켣����Ϣ
    double velocity;
    double acceleration;
    double ptr_x;
    double ptr_y;
    double ptr_theta;
    double ptr_kappa;

    std::vector<TrajectoryPoint> Points;
};

#endif // !COMBINER

