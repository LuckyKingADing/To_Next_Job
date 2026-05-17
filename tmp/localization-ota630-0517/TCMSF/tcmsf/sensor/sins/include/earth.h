#pragma once

#include "Eigen/Eigen"
#include <cmath>
namespace INS {

const struct GLV {
    static constexpr double Re  = 6378137.0;
    static constexpr double f   = 1 / 298.257;
    static constexpr double wie = 7.2921151467e-5;
    static constexpr double Rp  = (1 - f) * Re;
    static constexpr double e   = std::sqrt(2 * f - f * f);
    static constexpr double e2  = e * e;
    static constexpr double ep  = std::sqrt(Re * Re - Rp * Rp) / Rp;
    static constexpr double g0  = 9.7803267714;

    static constexpr double m = Re * wie * wie / g0;

    static constexpr double beta  = 5.0 / 2.0 * m - f - 17.0 / 14.0 * m * f;
    static constexpr double beta1 = (5.0 * m * f - f * f) / 8.0;

    static constexpr double ws      = 1.0 / std::sqrt(Re / g0);
    static constexpr double ppm     = 1.0e-6;
    static constexpr double arc2deg = M_PI / 180.0;
    static constexpr double deg2arc = 180.0 / M_PI;
} glv;

class EARTH {
public:
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d wnie;
    Eigen::Vector3d wnen;
    Eigen::Vector3d wnin;
    Eigen::Vector3d wnien;
    Eigen::Vector3d g;
    Eigen::Vector3d g_cc;

    double RMh, RNh, clRNh;
    double sl, cl, tl;

public:
    double sl2, sl4, sq, sqs;
    double vE_RNh;

public:
    EARTH();
    EARTH(const Eigen::Vector3d &pos_, const Eigen::Vector3d &vel_);

    void update(const Eigen::Vector3d &pos_, const Eigen::Vector3d &vel_);
};
} // namespace INS