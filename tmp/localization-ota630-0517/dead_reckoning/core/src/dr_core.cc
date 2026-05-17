#include "dr_core.h"

namespace byd {
namespace dr_core {

DEAD_RECKONING_CORE::DEAD_RECKONING_CORE() {
  phi_b << 0.0, 0.0, 0.0;
  theta_b << 0.0, 0.0, 0.0;
  position << 0.0, 0.0, 0.0;
  acc_measurement << 0.0, 0.0, 0.0;
  gyro_measurement << 0.0, 0.0, 0.0;
  vel_measurement << 0.0, 0.0, 0.0;
  measurement_time = 0;
  C_n2m = Eigen::Matrix3d::Identity();
  C_n2b = Eigen::Matrix3d::Identity();
  alpha = 0.1;
  dt_imu = 0.01;
  dt_veh = 0.01;
  slip_R = Eigen::Matrix3d::Identity();
  eulerAngle.setZero();
  Q_n2b.setIdentity();
  Q_b2n.setIdentity();
}

DEAD_RECKONING_CORE::DEAD_RECKONING_CORE(const Eigen::Matrix3d C_n2b_,
                                         const Eigen::Matrix3d &C_n2m_) {
  DEAD_RECKONING_CORE();
  C_n2b = C_n2b_;
  C_n2m = C_n2m_;
}

void DEAD_RECKONING_CORE::measurement_time_update(double timestamp_) {
  measurement_time = timestamp_;
}

void DEAD_RECKONING_CORE::dt_update(double dt_) {
  dt_imu = dt_;
  dt_veh = dt_;
}

void DEAD_RECKONING_CORE::phi_b_update(const Eigen::Vector3d &acc_b) {
  Eigen::Vector3d C3 = C_n2b.col(2);
  phi_b = acc_b.cross(C3);

  // std::cout << "acc_b: \n"
  //           << acc_b << " C3\n"
  //           << C3 << "phi_b: \n"
  //           << phi_b << "\n";
}

void DEAD_RECKONING_CORE::theta_b_update(const Eigen::Vector3d &gyro_b) {
  theta_b = gyro_b * dt_imu;  // orientation is not smooth enough, so use the
                              // simplest method to update theta_b
  // Eigen::AngleAxisd AA_n2b(C_n2b); // this method is not suitable here
  // theta_b = (gyro_b + AA_n2b.axis().cross(gyro_b) / 2.0 +
  //            AA_n2b.axis().cross(AA_n2b.axis().cross(gyro_b)) / 12.0) *
  //           dt_imu;
}

void DEAD_RECKONING_CORE::attitude_update(const Eigen::Vector3d &acc_b,
                                          const Eigen::Vector3d &gyro_b) {
  auto EPS = std::numeric_limits<double>::epsilon();

  {  // store current measurement
    acc_measurement = acc_b;
    gyro_measurement = gyro_b;
  }
  // std::cout << "acc_compesation: \n"
  //           << adaption.latitudinal_acc << "\n"
  //           << adaption.longitudinal_acc << "\n"
  //           << "bias_acc:\n"
  //           << adaption.bias_acc.x() << "\n"
  //           << adaption.bias_acc.y() << "\n"
  //           << adaption.bias_acc.z() << "\n";
  Eigen::Vector3d acc_b_compensated{
      acc_b.x() - adaption.latitudinal_acc - adaption.bias_acc.x(),
      acc_b.y() - adaption.longitudinal_acc - adaption.bias_acc.y(),
      acc_b.z() - adaption.bias_acc.z()};
  Eigen::Vector3d gyro_b_compensated{gyro_b.x() - adaption.bias_gyro.x(),
                                     gyro_b.y() - adaption.bias_gyro.y(),
                                     gyro_b.z() - adaption.bias_gyro.z()};
  phi_b_update(acc_b_compensated);
  theta_b_update(gyro_b_compensated);

  Q_n2b = C_n2b;
  Q_b2n = C_n2b.transpose();

  Eigen::Quaterniond Q_step = Eigen::Quaterniond::Identity();

  Eigen::Vector3d theta = theta_b + adaption.alpha_factor * alpha * phi_b;

  double theta_norm = theta.norm();
  // std::cout << theta_norm << "\n";
  // Q_step = Eigen::AngleAxisd(theta_norm, theta);
  auto q_xyz = theta / (theta_norm + EPS) * sin(theta_norm / 2.0);
  Q_step.w() = cos(theta_norm / 2.0);
  Q_step.x() = q_xyz.x();
  Q_step.y() = q_xyz.y();
  Q_step.z() = q_xyz.z();
  // std::cout << Q_step.x() << "\t" << Q_step.y() << "\t" << Q_step.z() << "\t"
  // << Q_step.w() << "\n";
  Q_b2n = Q_b2n * Q_step;
  Q_n2b = Q_b2n.conjugate();
  // std::cout << Q_b2n.x() << ", " << Q_b2n.y() << ", " << Q_b2n.z() << ", "
  // << Q_b2n.w() << ", " << "\n"
  // << Q_n2b.x() << ", " << Q_n2b.y() << ", " << Q_n2b.z() << ", "
  // << Q_n2b.w() << std::endl;
  Q_n2b.normalize();
  C_n2b = Q_n2b;

  eulerAngle = coor_trans.quaternion2euler(Q_n2b);
}

void DEAD_RECKONING_CORE::position_update(const Eigen::Vector3d &vel_m) {
  {  // store current measurement
    vel_measurement = vel_m;
  }

  if (adaption.misalignment_yaw >
      0.02) {  // bound alignment paras in case of idiots
    adaption.misalignment_yaw = 0.02;
  }
  if (adaption.misalignment_yaw < -0.02) {
    adaption.misalignment_yaw = -0.02;
  }
  Eigen::AngleAxisd Alignment(
      Eigen::AngleAxisd(-adaption.misalignment_yaw, Eigen::Vector3d::UnitZ()));

  // 使用经验公式，额外补偿下侧滑角
  constexpr bool enable_slip_compesation_ = true;
  if (enable_slip_compesation_) {
      double vel_norm_ = vel_measurement.norm();
      if (vel_measurement.y() < 0.0) {
          vel_norm_ = -vel_norm_;
      }
      double slip_angle_ = -1.0 * ((gyro_measurement.z() - adaption.bias_gyro.z()) * vel_norm_) / 3.2 / 180.0 * M_PI;
      {
          // 做个限制，侧滑角补偿最大量控制在4度以内，以免出现数值异常情况
          slip_angle_ = slip_angle_ > 4.0 / 180.0 * M_PI ? 4.0 / 180.0 * M_PI : slip_angle_;
          slip_angle_ = slip_angle_ < -4.0 / 180.0 * M_PI ? -4.0 / 180.0 * M_PI : slip_angle_;
      }
      if (vel_norm_ > 5.0 && std::fabs(slip_angle_) > 0.10 / 180.0 * M_PI) {
          // 速度为正，且大于一定值的时候，启用侧滑角补偿
          slip_R = Eigen::AngleAxisd(slip_angle_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      } else {
          slip_R = Eigen::Matrix3d::Identity();
      }
  }

  Eigen::Vector3d dp =
      slip_R * Alignment * C_n2b.transpose() * vel_m * dt_veh * adaption.veh_spd_factor;
  // std::cout << dp.x() << "," << dp.y() << "," << dp.z() << "\n";
  position += dp;
}

void DEAD_RECKONING_CORE::adaption_update(const Adaption &adaption) {
  this->adaption = adaption;
}

DrData DEAD_RECKONING_CORE::get_dr_result() {
  DrData dr_data;

  auto pos = coor_trans.FLU2RFU.transpose() * position;

  Eigen::Matrix3d C_b2n = Q_b2n.toRotationMatrix();
  Eigen::Quaterniond att{coor_trans.FLU2RFU.transpose() * C_b2n *
                         coor_trans.FLU2RFU};
  double heading = coor_trans.quaternion2euler(att).z() * 180.0 / M_PI;

  auto ang_vel_FLU = coor_trans.FLU2RFU.transpose() * gyro_measurement;
  auto accel_FLU = coor_trans.FLU2RFU.transpose() * acc_measurement;
  Eigen::Vector3d vel_ego;
  vel_ego << vel_measurement.y() * adaption.veh_spd_factor, 0.0, 0.0;
  Eigen::Vector3d vel_world = att.toRotationMatrix() * vel_ego;

  dr_data.timestamp = measurement_time;
  dr_data.pos_x = pos.x();
  dr_data.pos_y = pos.y();
  dr_data.pos_z = pos.z();
  dr_data.ori_x = att.x();
  dr_data.ori_y = att.y();
  dr_data.ori_z = att.z();
  dr_data.ori_w = att.w();
  dr_data.acc_x = accel_FLU.x();
  dr_data.acc_y = accel_FLU.y();
  dr_data.acc_z = accel_FLU.z();
  dr_data.gyro_x = ang_vel_FLU.x();
  dr_data.gyro_y = ang_vel_FLU.y();
  dr_data.gyro_z = ang_vel_FLU.z();
  dr_data.vel_x = vel_world.x();
  dr_data.vel_y = vel_world.y();
  dr_data.vel_z = vel_world.z();
  dr_data.heading = heading;

  // dr_data = {measurement_time, pos.x(),         pos.y(),       pos.z(),
  //            att.x(),          att.y(),         att.z(),       att.w(),
  //            accel_FLU.x(),    accel_FLU.y(),   accel_FLU.z(),
  //            ang_vel_FLU.x(), ang_vel_FLU.y(),  ang_vel_FLU.z(),
  //            vel_world.x(), vel_world.y(), vel_world.z(),    heading};

  return dr_data;
}  // namespace dr_core

}  // namespace dr_core
}  // namespace byd