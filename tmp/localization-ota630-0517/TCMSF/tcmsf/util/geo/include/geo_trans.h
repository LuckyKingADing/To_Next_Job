#pragma once
#include <array>
#include <cmath>

namespace byd {
namespace geo {

class Trans {

public:
    using M33 = std::array<std::array<double, 3>, 3>;
    struct CoordB { // WGS84坐标系
        double b;   // lat 纬 （度）
        double l;   // lon 经 （度）
        double h;   // 高 （米）
        CoordB(double c1, double c2, double c3) {
            b = c1;
            l = c2;
            h = c3;
        }
        CoordB() {
            b = 0.0;
            l = 0.0;
            h = 0.0;
        }
    };
    struct CoordX { // 米，地心地固坐标系
        double x;   // X
        double y;   // Y
        double z;   // Z
        CoordX() {
            x = 0.0;
            y = 0.0;
            z = 0.0;
        }
        CoordX(double x_, double y_, double z_) {
            x = x_;
            y = y_;
            z = z_;
        }
        CoordX operator+(const CoordX &ecef_) {
            return {x + ecef_.x, y + ecef_.y, z + ecef_.z};
        }
        CoordX operator-(const CoordX &ecef_) {
            return {x - ecef_.x, y - ecef_.y, z - ecef_.z};
        }
    };
    struct CoordE { // 米，导航坐标系
        double e;   // E(East)
        double n;   // N(North)
        double u;   // U(Up)
        CoordE() {
            e = 0.0;
            n = 0.0;
            u = 0.0;
        }
        CoordE(double e_, double n_, double u_) {
            e = e_;
            n = n_;
            u = u_;
        }
        CoordE operator+(const CoordE enu_) {
            return {e + enu_.e, n + enu_.n, u + enu_.u};
        }
        CoordE operator-(const CoordE enu_) {
            return {e - enu_.e, n - enu_.n, u - enu_.u};
        }
    };

    struct Orient { // 度，
        double az;  // 航向角
        double el;  // 俯仰角
        Orient() {
            az = 0.0;
            el = 0.0;
        }
        Orient(double az_, double el_) {
            az = az_;
            el = el_;
        }
    };

private:
    static constexpr double kPi    = M_PI;
    static constexpr double kPi180 = kPi / 180.0;
    static constexpr double kA     = 6378137.0;
    static constexpr double k1F    = 298.257223563;
    static constexpr double kB     = kA * (1.0 - 1.0 / k1F);
    static constexpr double kE2    = (1.0 / k1F) * (2.0 - (1.0 / k1F));
    static constexpr double kEd2   = kE2 * kA * kA / (kB * kB);

public:
    CoordX blh2ecef(const CoordB &c_src);

    CoordE blh2enu(const CoordB &c_0, const CoordB &c_1);

    CoordB ecef2blh(const CoordX &c_src);

    CoordX enu2ecef(const CoordB &c_0, const CoordE &enu_);

    CoordB enu2blh(const CoordB &c_0, const CoordE &enu_);

    Orient orientation(const CoordE &enu);

private:
    template <class T>
    inline void swap(T &a, T &b);
    template <class T>
    void transpose(T &a, int rows);

    M33 mat_x(double ang);
    M33 mat_y(double ang);
    M33 mat_z(double ang);
    M33 mul_mat(const M33 &mat_a, const M33 &mat_b);

    CoordE rotate(const M33 &mat_r, const CoordX &pt_0);
    CoordX rotate(const M33 &mat_r, const CoordE &pt_0);
    double n(double x);
};
} // namespace geo
} // namespace byd