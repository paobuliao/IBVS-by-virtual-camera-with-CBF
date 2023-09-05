#include <vector>
#include <math.h>
#include <Eigen/Dense>

Eigen::Matrix3d skew(Eigen::Vector3d v)
{
    Eigen::Matrix3d result;
    result << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return result;
}

std::vector<int> calImageAverage(std::vector<int> u, std::vector<int> v)
{
    /*
    param:
        u: 横向像素坐标
        v: 纵向像素坐标,should be same size as u
    */
    int ug = 0;
    int vg = 0;
    for (int k = 0; k < u.size(); k++)
    {
        ug += u[k];
        vg += v[k];
    }
    ug /= u.size();
    vg /= v.size();
    std::vector<int> result;
    result.push_back(ug);
    result.push_back(vg);
    return result;
}

double calImageMoment(std::vector<int> u, std::vector<int> v, int i, int j)
{
    /*
    param:
        u: 横向像素坐标
        v: 纵向像素坐标,should be same size as u
        i: order of moment
        j: order of moment
    */
    int ug = 0;
    int vg = 0;
    auto ave_result = calImageAverage(u, v);
    ug = ave_result[0];
    vg = ave_result[1];

    double m = 0;
    for (int k = 0; k < u.size(); k++)
    {
        m += (u[k] - ug) ^ i * (v[k] - vg) ^ j;
    }
    return m;
}

Eigen::Vector4d calImageFeatures(std::vector<int> u, std::vector<int> v, double a_, double lambda)
{
    auto ave_result = calImageAverage(u, v);
    int ug = ave_result[0];
    int vg = ave_result[1];
    double m11 = calImageMoment(u, v, 0, 0);
    double m20 = calImageMoment(u, v, 2, 0);
    double m02 = calImageMoment(u, v, 0, 2);
    double a = m20 + m02;
    double qx, qy, qz, q4;
    qz = sqrt(a_ / a);
    qx = qz * ug / lambda;
    qy = qz * vg / lambda;
    q4 = 1 / 2 * atan(2 * m11 / (m20 - m02)); // 出来的是弧度吧
    Eigen::Vector4d result;
    result << qx, qy, qz, q4;
    return result;
}

Eigen::Vector4d cal_q1d(Eigen::Vector4d q1, Eigen::Vector3d v, double faid, double fai, double z_)
{
    Eigen::Vector3d q1d;
    Eigen::Vector3d e3 << 0, 0, 1;
    q1d = -skew(faid * e3) * q1 - 1 / z_ * v;
    double q4d = -fai;
    Eigen::Vector4d result;
    result << q1d, q4d;
    return result;
}

// 参数包含两个四维向量
Eigen::Vector4d Cal_ufd(double c1, double c2, double z_,
                        double faid, double fai, Eigen::Vector4d q1, Eigen::Vector4d q1d,
                        Eigen::Vector3d v, Eigen::Vector4d uf Eigen::Vector4d delta)
{
    // 定义一个四行四列矩阵
    Eigen::MatrixXd H(4, 4) << -1 / z_, 0, 0, q1(1),
        0, -1 / z_, 0, -q1(0), 0, 0, -1 / z_, 0,
        0, 0, 0, -1;
    Eigen::MatrixXd Hd(4, 4) << 0, 0, 0, q1d(1),
        0, 0, 0, -q1d(0),
        0, 0, 0, 0,
        0, 0, 0, 0;
    Eigen::MatrixXd ufd(4, 1);
    ufd = (c2 * H).inverse() * (-(c1 * H + c2 * Hd) * uf - c3 * delta); // q1_d and q1_dd are zero
    return ufd
}

// 积分ufd获取uf，通过四阶龙格库塔法
Eigen::Vector4d Cal_uf(Eigen::Vector4d uf, double dt,
                       double c1, double c2, double z_,
                       double faid, double fai, Eigen::Vector4d q1, Eigen::Vector4d q1d,
                       Eigen::Vector3d v, Eigen::Vector4d uf Eigen::Vector4d delta)
{
    Eigen::Vector4d uf1, uf2, uf3, uf4, uf1d, uf2d, uf3d, uf4d;
    uf1=uf;
    uf1d = Cal_ufd(c1, c2, z_, faid, fai, q1, q1d, v, uf, delta);
    uf2=uf1+dt/2*uf1d;
    uf2d = Cal_ufd(c1, c2, z_, faid, fai, q1, q1d, v, uf2, delta);
    uf3=uf1+dt/2*uf2d;
    uf3d = Cal_ufd(c1, c2, z_, faid, fai, q1, q1d, v, uf3, delta);
    uf4=uf1+dt*uf3d;
    uf4d = Cal_ufd(c1, c2, z_, faid, fai, q1, q1d, v, uf4, delta);
    uf=uf1+dt/6*(uf1d+2*uf2d+2*uf3d+uf4d);
    return uf;
}
//其实现在计算出了uf，和ufd，不如直接用lEE的几何控制器

// 速度观测器
