#include "geoinfo_utils.h"

#include <iostream>

namespace byd {
namespace geoinfo {

EARTH::EARTH() {
  pos << 0.0, 0.0, 0.0;
  vel << 0.0, 0.0, 0.0;
  wnie << 0.0, 0.0, 0.0;
  wnen << 0.0, 0.0, 0.0;
  wnin << 0.0, 0.0, 0.0;
  wnien << 0.0, 0.0, 0.0;
  g << 0.0, 0.0, 0.0;
  g_cc << 0.0, 0.0, 0.0;
  update(pos, vel);
}

EARTH::EARTH(const Eigen::Vector3d &pos_, const Eigen::Vector3d &vel_) {
  update(pos_, vel_);
}

void EARTH::update(const Eigen::Vector3d &pos_, const Eigen::Vector3d &vel_) {
  pos = pos_;
  vel = vel_;

  double sl = sin(pos(0));
  double cl = cos(pos(0));
  double tl = tan(pos(0));
  double sl2 = sl * sl;
  double sl4 = sl2 * sl2;
  double sq = 1 - glv.e2 * sl2;
  double sqs = sqrt(sq);
  if (std::abs(sq) > std::numeric_limits<double>::epsilon() &&
      std::abs(sqs) > std::numeric_limits<double>::epsilon()) {
    RMh = glv.Re * (1 - glv.e2) / sq / sqs + pos(2);
    RNh = glv.Re / sqs + pos(2);
    clRNh = cl * RNh;
    wnie << 0.0, glv.wie * cl, glv.wie * sl;
    if (std::abs(RMh) > std::numeric_limits<double>::epsilon() &&
        std::abs(RNh) > std::numeric_limits<double>::epsilon()) {
      double vE_RNh = vel(0) / RNh;
      wnen << -vel(1) / RMh, vE_RNh, vE_RNh * tl;
    }
    wnin = wnie + wnen;
    wnien = wnie + wnin;
    g << 0.0, 0.0,
        -(glv.g0 * (1 + 5.27094e-3 * sl2 + 2.32718e-5 * sl4) -
          3.086e-6 * pos(2));
    g_cc = g - wnien.cross(vel);
  }
}

}  // namespace geoinfo
}  // namespace byd
