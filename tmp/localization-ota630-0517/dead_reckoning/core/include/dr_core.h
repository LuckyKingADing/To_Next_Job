#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>

#include <Eigen/Dense>

#include "analysis.h"
#include "cartesian_utils.h"
#include "geoinfo_utils.h"

#include "cyber/common/log.h"
namespace byd {
namespace dr_core {

using DR_CORE = class DEAD_RECKONING_CORE {
 public:
  DEAD_RECKONING_CORE();
  DEAD_RECKONING_CORE(const Eigen::Matrix3d C_n2b_,
                      const Eigen::Matrix3d &C_n2m_);

 public:
  typedef struct ADAPTION {
    Eigen::Vector3d bias_gyro;
    Eigen::Vector3d bias_acc;
    double veh_spd_factor;
    std::array<double, 4> whl_cnt_to_spd;
    double latitudinal_acc;
    double longitudinal_acc;
    double misalignment_yaw;
    double alpha_factor;
    ADAPTION()
        : bias_gyro{0, 0, 0},
          // : bias_gyro{-0.005 / 180 * M_PI, -0.005 / 180 * M_PI,
          //             0.0285 / 180 * M_PI},
          bias_acc{0, 0, 0},
          veh_spd_factor{1.0},
          whl_cnt_to_spd{0, 0, 0, 0},
          latitudinal_acc(0),
          longitudinal_acc(0),
          misalignment_yaw(0),
          alpha_factor(0.01) {}
  } Adaption;

 private:
  Adaption adaption;
  dr::CoordinateTransformation coor_trans;

 private:
  Eigen::Vector3d phi_b;
  Eigen::Matrix3d C_n2b;
  Eigen::Vector3d theta_b;
  Eigen::Matrix3d C_n2m;
  double alpha;
  double dt_imu, dt_veh;

 private:
  double measurement_time;
  Eigen::Vector3d acc_measurement;
  Eigen::Vector3d gyro_measurement;
  Eigen::Vector3d vel_measurement;

 private:
  Eigen::Vector3d position;
  Eigen::Vector3d eulerAngle;
  Eigen::Quaterniond Q_n2b, Q_b2n;
  Eigen::Matrix3d slip_R;

 private:
  void phi_b_update(const Eigen::Vector3d &acc_b);
  void theta_b_update(const Eigen::Vector3d &gyro_b);
  void alpha_update();

 public:
  void measurement_time_update(double timestamp_);
  void attitude_update(const Eigen::Vector3d &acc_b,
                       const Eigen::Vector3d &gyro_b);
  void position_update(const Eigen::Vector3d &vel_m);
  void dt_update(double dt_);
  void adaption_update(const Adaption &adaption);
  DrData get_dr_result();
};
}  // namespace dr_core
}  // namespace byd