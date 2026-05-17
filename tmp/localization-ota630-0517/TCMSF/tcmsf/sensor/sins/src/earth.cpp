#include "earth.h"

namespace INS {

EARTH::EARTH() {
    pos << 0.0, 0.0, 0.0;
    vel << 0.0, 0.0, 0.0;
    update(pos, vel);
}

EARTH::EARTH(const Eigen::Vector3d &pos_, const Eigen::Vector3d &vel_) { update(pos_, vel_); }

void EARTH::update(const Eigen::Vector3d &pos_, const Eigen::Vector3d &vel_) {
    pos = pos_;
    vel = vel_;

    sl    = sin(pos(0));
    cl    = cos(pos(0));
    tl    = tan(pos(0));
    sl2   = sl * sl;
    sl4   = sl2 * sl2;
    sq    = 1 - glv.e2 * sl2;
    sqs   = sqrt(sq);
    RMh   = glv.Re * (1 - glv.e2) / sq / sqs + pos(2);
    RNh   = glv.Re / sqs + pos(2);
    clRNh = cl * RNh;
    wnie << 0.0, glv.wie * cl, glv.wie * sl;
    vE_RNh = vel(0) / RNh;
    wnen << -vel(1) / RMh, vE_RNh, vE_RNh * tl;
    wnin  = wnie + wnen;
    wnien = wnie + wnin;

    double g_L = glv.g0 * (1.0 + glv.beta * sl2 - glv.beta1 * (2.0 * sl * cl) * (2.0 * sl * cl));
    double h_R = pos(2) / (glv.Re * (1.0 - glv.f * sl2));

    g << 0.0, 0.0, -(g_L * (1.0 - 2.0 * h_R - 5.0 * h_R * h_R));
    g_cc = g - wnien.cross(vel);
}

} // namespace INS