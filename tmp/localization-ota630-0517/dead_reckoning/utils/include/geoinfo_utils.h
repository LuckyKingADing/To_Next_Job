#pragma once
#include <Eigen/Geometry>

namespace byd {
namespace geoinfo {

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

  double RMh = 0.0, RNh = 0.0, clRNh = 0.0;

 public:
  EARTH();
  EARTH(const Eigen::Vector3d &pos_, const Eigen::Vector3d &vel_);

  void update(const Eigen::Vector3d &pos_, const Eigen::Vector3d &vel_);
};

struct GLV {
  double Re = 6378137.0;
  double f = 1 / 298.257;
  double wie = 7.2921151467e-5;
  double Rp = (1 - f) * Re;
  double e = sqrt(2 * f - f * f);
  double e2 = e * e;
  double ep = sqrt(Re * Re - Rp * Rp) / Rp;
  double g0 = 9.7803267714;
  double ws = 1.0 / sqrt(Re / g0);
  double ppm = 1.0e-6;
  double arc2deg = 180.0 / M_PI;
  double deg2arc = M_PI / 180.0;
} const glv;
}  // namespace geoinfo
}  // namespace byd