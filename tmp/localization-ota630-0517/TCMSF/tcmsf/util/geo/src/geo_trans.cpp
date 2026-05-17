#include "geo_trans.h"

namespace byd {
namespace geo {

template <class T>
void Trans::swap(T &a, T &b) {
    T temp = a;
    a      = b;
    b      = temp;
    return;
}

template <class T>
void Trans::transpose(T &a, int rows) {
    for (int i = 0; i < rows; i++) {
        for (int j = i + 1; j < rows; j++) {
            swap(a[i][j], a[j][i]);
        }
    }
    return;
}

Trans::M33 Trans::mat_x(double ang) {
    double a;
    double c;
    double s;
    M33    mat{};

    a         = ang * kPi180;
    c         = cos(a);
    s         = sin(a);
    mat[0][0] = 1.0;
    mat[1][1] = c;
    mat[1][2] = s;
    mat[2][1] = -s;
    mat[2][2] = c;

    return mat;
}

Trans::M33 Trans::mat_y(double ang) {
    double a;
    double c;
    double s;
    M33    mat{};

    a         = ang * kPi180;
    c         = cos(a);
    s         = sin(a);
    mat[0][0] = c;
    mat[0][2] = -s;
    mat[1][1] = 1.0;
    mat[2][0] = s;
    mat[2][2] = c;

    return mat;
}

Trans::M33 Trans::mat_z(double ang) {
    double a;
    double c;
    double s;
    M33    mat{};

    a         = ang * kPi180;
    c         = cos(a);
    s         = sin(a);
    mat[0][0] = c;
    mat[0][1] = s;
    mat[1][0] = -s;
    mat[1][1] = c;
    mat[2][2] = 1.0;

    return mat;
}

Trans::M33 Trans::mul_mat(const M33 &mat_a, const M33 &mat_b) {

    unsigned int i;
    unsigned int j;
    unsigned int k;
    M33          mat{};

    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            for (k = 0; k < 3; ++k) {
                mat[i][j] += mat_a[i][k] * mat_b[k][j];
            }
        }
    }

    return mat;
}

Trans::CoordE Trans::rotate(const M33 &mat_r, const CoordX &pt_0) {
    CoordE pt;

    pt.e = mat_r[0][0] * pt_0.x + mat_r[0][1] * pt_0.y + mat_r[0][2] * pt_0.z;
    pt.n = mat_r[1][0] * pt_0.x + mat_r[1][1] * pt_0.y + mat_r[1][2] * pt_0.z;
    pt.u = mat_r[2][0] * pt_0.x + mat_r[2][1] * pt_0.y + mat_r[2][2] * pt_0.z;

    return pt;
}

Trans::CoordX Trans::rotate(const M33 &mat_r, const CoordE &pt_0) {
    CoordX pt;

    pt.x = mat_r[0][0] * pt_0.e + mat_r[0][1] * pt_0.n + mat_r[0][2] * pt_0.u;
    pt.y = mat_r[1][0] * pt_0.e + mat_r[1][1] * pt_0.n + mat_r[1][2] * pt_0.u;
    pt.z = mat_r[2][0] * pt_0.e + mat_r[2][1] * pt_0.n + mat_r[2][2] * pt_0.u;

    return pt;
}

double Trans::n(double x) {
    double res;

    res = kA / sqrt(1.0 - kE2 * pow(sin(x * kPi180), 2));

    return res;
}

Trans::CoordX Trans::blh2ecef(const CoordB &c_src) {
    CoordX ecef;

    ecef.x = (n(c_src.b) + c_src.h) * cos(c_src.b * kPi180) * cos(c_src.l * kPi180);
    ecef.y = (n(c_src.b) + c_src.h) * cos(c_src.b * kPi180) * sin(c_src.l * kPi180);
    ecef.z = (n(c_src.b) * (1.0 - kE2) + c_src.h) * sin(c_src.b * kPi180);

    return ecef;
}

// 将纬-经-高转换为当地坐标
// 输入 c_0 为坐标原点（纬经高），c_1 为目标点（纬经高）
// 输出为：c_1 点在以 c_0 点为原点的本地坐标系下的坐标（东-北-天）
Trans::CoordE Trans::blh2enu(const CoordB &c_0, const CoordB &c_1) {
    CoordX ecef_x_0;
    CoordX ecef_x_1;
    CoordX ecef_x;
    M33    mat_0{};
    M33    mat_1{};
    M33    mat_2{};
    M33    mat{};
    CoordE enu;

    ecef_x_0 = blh2ecef(c_0);
    ecef_x_1 = blh2ecef(c_1);
    ecef_x   = {ecef_x_1.x - ecef_x_0.x, ecef_x_1.y - ecef_x_0.y, ecef_x_1.z - ecef_x_0.z};

    mat_0 = mat_z(90.0);
    mat_1 = mat_y(90.0 - c_0.b);
    mat_2 = mat_z(c_0.l);
    mat   = mul_mat(mul_mat(mat_0, mat_1), mat_2);
    enu   = rotate(mat, ecef_x);

    return enu;
}

/*
 * @brief      ECEF -> BLH
 *
 * @param[in]  ECEF 坐标 (CoordX)
 * @return     BLH  坐标 (CoordB)
 */
Trans::CoordB Trans::ecef2blh(const CoordX &c_src) {
    double p;
    double theta;
    CoordB c_res;

    p       = sqrt(c_src.x * c_src.x + c_src.y * c_src.y);
    theta   = atan2(c_src.z * kA, p * kB) / kPi180;
    c_res.b = atan2(c_src.z + kEd2 * kB * pow(sin(theta * kPi180), 3), p - kE2 * kA * pow(cos(theta * kPi180), 3)) / kPi180; // Beta(Latitude)
    c_res.l = atan2(c_src.y, c_src.x) / kPi180;                                                                              // Lambda(Longitude)
    c_res.h = (p / cos(c_res.b * kPi180)) - n(c_res.b);                                                                      // Height

    return c_res;
}

Trans::CoordX Trans::enu2ecef(const CoordB &c_0, const CoordE &enu_) {

    M33 mat_0{};
    M33 mat_1{};
    M33 mat_2{};
    M33 mat{};

    CoordX ecef, ecef_0;

    ecef_0 = blh2ecef(c_0);

    mat_0 = mat_z(90.0);
    mat_1 = mat_y(90.0 - c_0.b);
    mat_2 = mat_z(c_0.l);
    mat   = mul_mat(mul_mat(mat_0, mat_1), mat_2);
    transpose(mat, 3);
    ecef = rotate(mat, enu_);

    return ecef_0 + ecef;
}

Trans::CoordB Trans::enu2blh(const CoordB &c_0, const CoordE &enu_) {
    return ecef2blh(enu2ecef(c_0, enu_));
}

// 计算航向角和俯仰角
// 输入 本地东-北-天坐标
// 输出 航向角-俯仰角 北偏东 0~360
Trans::Orient Trans::orientation(const CoordE &enu) {
    double az = atan2(enu.e, enu.n) / kPi180;
    if (az < 0.0) {
        az += 360.0;
    }
    double el = atan2(enu.u, sqrt(enu.e * enu.e + enu.n * enu.n)) / kPi180;
    return {az, el};
}
} // namespace geo
} // namespace byd