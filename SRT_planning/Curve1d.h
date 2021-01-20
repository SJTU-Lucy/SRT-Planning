#ifndef CURVE1D
#define CURVE1D

#include <vector>

// ������Ϣ����һ���Ĵζ���ʽ��ʾ��Ϊ�ٶȹ���ʱ�亯��������ͨ��΢�ֺͻ����ٵõ�
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
    // ʹ���Ĵκ���������߹켣
    void calculate_s(double v_0, double a_0, double v_0p5, double v_1, double v_2);
    void calculate_d(double d_0, double v_0, double a_0, double t_0, double offset);
    double s(double t);
    double d(double t);
    double v(double t);
    double a(double t);
    // �������ߵ�����ٶȺ���С�ٶ�
    double max_speed();
    double min_speed();
    // �ù켣��1s��ǰ���ľ���
    double forward();
    double avg_speed();
    void print();
};

#endif // !CURVE1D


