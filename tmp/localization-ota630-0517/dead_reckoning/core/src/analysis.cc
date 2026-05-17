#include "analysis.h"

void VDSA::statics() {
  max_ =
      -Eigen::Matrix<double, 9, 1>::Ones() * std::numeric_limits<double>::max();
  min_ =
      Eigen::Matrix<double, 9, 1>::Ones() * std::numeric_limits<double>::max();
  // mean_ = Eigen::Matrix<double, 9, 1>::Zero();
  gyro_mean_ = Eigen::Matrix<double, 3, 1>::Zero();
  gyro_mean_left_ = Eigen::Matrix<double, 3, 1>::Zero();
  gyro_mean_middle_ = Eigen::Matrix<double, 3, 1>::Zero();
  gyro_mean_right_ = Eigen::Matrix<double, 3, 1>::Zero();

  for (size_t i = 0; i < BUFFER_SIZE; i++) {
    max_(0) = max_(0) > vel[i](0) ? max_(0) : vel[i](0);
    max_(1) = max_(1) > vel[i](1) ? max_(1) : vel[i](1);
    max_(2) = max_(2) > vel[i](2) ? max_(2) : vel[i](2);
    max_(3) = max_(3) > acc[i](0) ? max_(3) : acc[i](0);
    max_(4) = max_(4) > acc[i](1) ? max_(4) : acc[i](1);
    max_(5) = max_(5) > acc[i](2) ? max_(5) : acc[i](2);
    max_(6) = max_(6) > gyro[i](0) ? max_(6) : gyro[i](0);
    max_(7) = max_(7) > gyro[i](1) ? max_(7) : gyro[i](1);
    max_(8) = max_(8) > gyro[i](2) ? max_(8) : gyro[i](2);
    min_(0) = min_(0) < vel[i](0) ? min_(0) : vel[i](0);
    min_(1) = min_(1) < vel[i](1) ? min_(1) : vel[i](1);
    min_(2) = min_(2) < vel[i](2) ? min_(2) : vel[i](2);
    min_(3) = min_(3) < acc[i](0) ? min_(3) : acc[i](0);
    min_(4) = min_(4) < acc[i](1) ? min_(4) : acc[i](1);
    min_(5) = min_(5) < acc[i](2) ? min_(5) : acc[i](2);
    min_(6) = min_(6) < gyro[i](0) ? min_(6) : gyro[i](0);
    min_(7) = min_(7) < gyro[i](1) ? min_(7) : gyro[i](1);
    min_(8) = min_(8) < gyro[i](2) ? min_(8) : gyro[i](2);
    // mean_(0) += vel[i](0);
    // mean_(1) += vel[i](1);
    // mean_(2) += vel[i](2);
    // mean_(3) += acc[i](0);
    // mean_(4) += acc[i](1);
    // mean_(5) += acc[i](2);
    // mean_(6) += gyro[i](0);
    // mean_(7) += gyro[i](1);
    // mean_(8) += gyro[i](2);
    gyro_mean_(0) += gyro[i](0);
    gyro_mean_(1) += gyro[i](1);
    gyro_mean_(2) += gyro[i](2);
    if (i < BUFFER_SIZE / 3) {
      gyro_mean_left_(0) += gyro[i](0);
      gyro_mean_left_(1) += gyro[i](1);
      gyro_mean_left_(2) += gyro[i](2);
    } else if (i >= BUFFER_SIZE / 3 && i < BUFFER_SIZE * 2 / 3) {
      gyro_mean_middle_(0) += gyro[i](0);
      gyro_mean_middle_(1) += gyro[i](1);
      gyro_mean_middle_(2) += gyro[i](2);
    } else {
      gyro_mean_right_(0) += gyro[i](0);
      gyro_mean_right_(1) += gyro[i](1);
      gyro_mean_right_(2) += gyro[i](2);
    }
  }
  gyro_mean_left_ /= (BUFFER_SIZE / 3);
  gyro_mean_middle_ /= (BUFFER_SIZE * 2 / 3 - BUFFER_SIZE / 3);
  gyro_mean_right_ /= (BUFFER_SIZE - BUFFER_SIZE * 2 / 3);
  // mean_ /= BUFFER_SIZE;
  gyro_mean_ /= BUFFER_SIZE;

  // auto dif = (max_ - min_);
  // std::cout << dif(0) << "," << dif(1) << "," << dif(2) << "," << dif(3) <<
  // ","
  //           << dif(4) << "," << dif(5) << "," << dif(6) << "," << dif(7) <<
  //           ","
  //           << dif(8) << "\n";
}

bool VDSA::is_static() {
    statics();
    // bool is_zero_speed = (max_.block(3, 1, 0, 0).array().abs() >
    //                       cfg.static_bound.block(3, 1, 0, 0).array())
    //                          .sum() == 0;
    bool is_imu_static =
        ((max_ - min_).array().abs() > cfg.static_bound.array()).sum() == 0;
    bool is_gyro_mean_valid =
        ((gyro_mean_left_ - gyro_mean_middle_).array().abs() >
         cfg.gyro_bias_valid_mean_bound.array())
                .sum() == 0 &&
        ((gyro_mean_right_ - gyro_mean_middle_).array().abs() >
         cfg.gyro_bias_valid_mean_bound.array())
                .sum() == 0;
    // return is_zero_speed && is_imu_static && is_gyro_mean_valid;
    return is_imu_static && is_gyro_mean_valid;
}

VDSA::RESULT VDSA::operator()(const Eigen::Vector3d& vel_,
                              const Eigen::Vector3d& acc_,
                              const Eigen::Vector3d& gyro_, double dt,
                              double acc_lon_sm, double acc_lat_sm) {
  count++;
  VDSA::RESULT result;
  auto EPS = std::numeric_limits<double>::epsilon();

  // Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();

  // size_t pre_idx = idx;
  idx++;
  if (idx >= BUFFER_SIZE) idx = 0;
  size_t fwd_idx = idx + 1;
  if (fwd_idx >= BUFFER_SIZE) fwd_idx = 0;

  vel[idx] = vel_;
  acc[idx] = acc_;
  gyro[idx] = gyro_;

  {
    auto vn = vel_.norm();
    auto wz = gyro_.z();

    latitudinal_acc = -vn * wz / (earth.g.norm() + EPS);
    // if (std::abs(latitudinal_acc) > 0.05)
    // std::cout << "vn: " << vn << " wz: " << wz
    // << " lateral_acc: " << latitudinal_acc << "\n";
    result.lat_ac = 1;
  }

  {
    if (count >= BUFFER_SIZE) {
      size_t idx_pre_half = idx + BUFFER_SIZE / 2;
      if (idx_pre_half >= BUFFER_SIZE) {
        idx_pre_half -= BUFFER_SIZE;
      }

      longitudinal_acc = (vel[idx].y() - vel[idx_pre_half].y()) /
                         (std::abs(dt) * ((float)BUFFER_SIZE / 2) + EPS) /
                         (earth.g.norm() + EPS);
      //   if (std::abs(longitudinal_acc) > 1)
      //     std::cout << "vk: " << vel[idx].y() << " vk1: " << vel[fwd_idx].y()
      //               << " longitudinal_acc: " << longitudinal_acc << "\n";
      result.lon_ac = 1;
    }
  }
  // Eigen::Matrix3d gyro_s = Vec2Skew(gyro_) * dt;
  // Eigen::Matrix3d C_step = (2 * I3x3 + gyro_s) * (2 * I3x3 -
  // gyro_s).inverse(); ori[idx] = ori[pre_idx] * C_step;

  if (count >= BUFFER_SIZE && vel_.norm() < 1e-2 && is_static()) {
    // Eigen::Matrix3d C = ori[fwd_idx].inverse() * ori[idx];
    // gyro_bias = Skew2Vec((C + I3x3).inverse() * (C - I3x3) * 2) /
    //             (dt * (BUFFER_SIZE - 1));
    // gyro_bias = mean_.block<3, 1>(6, 0);
    gyro_bias = gyro_mean_;
    double gyro_bias_bound = 0.2 / 180.0 * M_PI;
    gyro_bias.x() =
        gyro_bias.x() > gyro_bias_bound ? gyro_bias_bound : gyro_bias.x();
    gyro_bias.x() =
        gyro_bias.x() < -gyro_bias_bound ? -gyro_bias_bound : gyro_bias.x();
    gyro_bias.y() =
        gyro_bias.y() > gyro_bias_bound ? gyro_bias_bound : gyro_bias.y();
    gyro_bias.y() =
        gyro_bias.y() < -gyro_bias_bound ? -gyro_bias_bound : gyro_bias.y();
    gyro_bias.z() =
        gyro_bias.z() > gyro_bias_bound ? gyro_bias_bound : gyro_bias.z();
    gyro_bias.z() =
        gyro_bias.z() < -gyro_bias_bound ? -gyro_bias_bound : gyro_bias.z();
    result.zupt = 1;
  }

  {
    double alpha_trick = 6.0;

    double f_lat = std::abs(latitudinal_acc) / 0.1;
    double f_lon = std::abs(longitudinal_acc) / 0.1;

    if (f_lat > 1 || f_lon > 1) {
      alpha_factor = 0.01;
    } else if (f_lat > 1 / 20.0 || f_lon > 1 / 20.0) {
      alpha_factor =
          9.0 / 95.0 + 0.01 - (9.0 / 95.0) * (f_lat > f_lon ? f_lat : f_lon);
    } else {
      alpha_factor = 0.1;
    }

    alpha_factor /= alpha_trick;

    {
      double acc_factor = 1.0;
      if (std::abs(acc_lon_sm) > 0.15 || std::abs(acc_lat_sm) > 0.15) {
        acc_factor = 3.0;
      } else if (std::abs(acc_lon_sm) > 0.05 || std::abs(acc_lat_sm) > 0.05) {
        acc_factor = 20 * (std::abs(acc_lon_sm) > std::abs(acc_lat_sm)
                               ? std::abs(acc_lon_sm)
                               : std::abs(acc_lat_sm));
      } else {
        acc_factor = 1.0;
      }
      // std::cout << acc_factor << std::endl;
      alpha_factor /= (std::abs(acc_factor) + EPS);
    }

    {
      if (acc_lonlat_count % ACC_LONLAT_SKIP == 0) {
        dacc_lon_sm = (acc_lon_sm - acc_lon_sm_pre) /
                      (std::abs(dt) * ACC_LONLAT_SKIP + EPS);
        dacc_lat_sm = (acc_lat_sm - acc_lat_sm_pre) /
                      (std::abs(dt) * ACC_LONLAT_SKIP + EPS);
        acc_lon_sm_pre = acc_lon_sm;
        acc_lat_sm_pre = acc_lat_sm;
      }
      double dacc_factor = 1.0;
      if (std::abs(dacc_lon_sm) > 0.15 || std::abs(dacc_lat_sm) > 0.15) {
        dacc_factor = 3.0;
      } else if (std::abs(dacc_lon_sm) > 0.05 || std::abs(dacc_lat_sm) > 0.05) {
        dacc_factor = 20 * (std::abs(dacc_lon_sm) > std::abs(dacc_lat_sm)
                                ? std::abs(dacc_lon_sm)
                                : std::abs(dacc_lat_sm));
      } else {
        dacc_factor = 1.0;
      }
      if (std::abs(dacc_factor) < 1.0 + EPS) {
        dacc_factor = 1.0;
      }
      alpha_factor /= (std::abs(dacc_factor) + EPS);
      acc_lonlat_count++;
    }

    if (alpha_factor < 0.002) {
      alpha_factor = 0.002;
    }
    if (alpha_factor > 0.016) {
      alpha_factor = 0.016;
    }

    // alpha_factor = 0.002;
    result.alp_a = 1;
  }

  return result;
}

DA::RESULT DA::operator()(const DrData& dr_data, const MsfData& msf_data,
                          double whl_spd, double dt, double rtk_fix_time) {
  DA::RESULT result;

  auto EPS = std::numeric_limits<double>::epsilon();

  count++;
  // size_t pre_idx = idx;
  idx++;
  if (idx >= BUFFER_SIZE) idx = 0;
  size_t fwd_idx = idx + 1;
  if (fwd_idx >= BUFFER_SIZE) fwd_idx = 0;
  // yaw_ref[idx] = msf_data.heading;
  // yaw_dr[idx] = dr_data.heading / 180 * M_PI;
  v_ref[idx] =
      sqrt(msf_data.vel_x * msf_data.vel_x + msf_data.vel_y * msf_data.vel_y +
           msf_data.vel_z * msf_data.vel_z);
  v_whl[idx] = std::abs(whl_spd);

  // if (true) {  // method: gyro bias estimate by yaw
  //   // it seems this method does not get higher accuracy than ZUPT
  //   // not enable by default
  //   yaw_bias = 0;
  //   // std::cout<<"rtk fix time: "<<rtk_fix_time<<std::endl;
  //   if (count >= BUFFER_SIZE && rtk_fix_time > 2 * dt * BUFFER_SIZE) {
  //     // if (count >= BUFFER_SIZE) {
  //     for (size_t i = 0; i < BUFFER_SIZE; i++) {
  //       size_t pre_idx_ = pre_idx + i;
  //       if (pre_idx_ >= BUFFER_SIZE) {
  //         pre_idx_ = pre_idx_ - BUFFER_SIZE;
  //       }
  //       size_t idx_ = idx + i;
  //       if (idx_ >= BUFFER_SIZE) {
  //         idx_ = idx_ - BUFFER_SIZE;
  //       }
  //       // std::cout << yaw_ref[idx_] << " " << yaw_dr[idx_] << std::endl;
  //       double dyaw_ref = yaw_ref[idx_] - yaw_ref[pre_idx_];
  //       double dyaw_dr = yaw_dr[idx_] - yaw_dr[pre_idx_];
  //       // std::cout << yaw_ref[idx_] << " " << yaw_dr[idx_] << std::endl;
  //       if (dyaw_ref > M_PI) {
  //         dyaw_ref -= M_PI * 2;
  //       }
  //       if (dyaw_ref < -M_PI) {
  //         dyaw_ref += M_PI * 2;
  //       }
  //       if (dyaw_dr > M_PI) {
  //         dyaw_dr -= M_PI * 2;
  //       }
  //       if (dyaw_dr < -M_PI) {
  //         dyaw_dr += M_PI * 2;
  //       }
  //       double d_yaw = dyaw_dr - dyaw_ref;
  //       // std::cout << dyaw_ref * 180 / M_PI << " " << dyaw_dr * 180 / M_PI
  //       <<
  //       // " "
  //       // << d_yaw << std::endl;
  //       if (std::abs(d_yaw) < 0.01 / 180.0 * M_PI && std::abs(dyaw_ref) > EPS
  //       &&
  //           std::abs(dyaw_dr) / (std::abs(dt) + EPS) < 0.05 / 180 * M_PI) {
  //         yaw_bias = yaw_bias + (d_yaw) / (BUFFER_SIZE - 1);
  //       }
  //     }
  //     yaw_bias /= (std::abs(dt) + EPS);
  //     // std::cout << "yaw bias: " << yaw_bias * 180 / M_PI << std::endl;
  //     result.yaw_ba = 1;
  //   }
  // }

  double min_spd_whl = std::numeric_limits<double>::max();
  for (auto& whl_spd_ : v_whl) {
    if (min_spd_whl > std::abs(whl_spd_)) {
      min_spd_whl = std::abs(whl_spd_);
    }
  }

  if (std::abs(min_spd_whl) > 5.0 && count >= BUFFER_SIZE) {
    result.spd_sa = 1;
    double v_ref_sum = 0.0, v_whl_sum = 0.0;
    for (auto& v_ref_ : v_ref) {
      v_ref_sum += v_ref_;
    }
    for (auto& v_whl_ : v_whl) {
      v_whl_sum += v_whl_;
    }
    // std::cout << "min spd: " << min_spd_whl
    // << " whl spd factor: " << whl_spd_factor << "\n";
    // std::cout << "velocity: " << v_ref_sum << "," << v_whl_sum << std::endl;
    whl_spd_factor = v_ref_sum / (v_whl_sum + EPS);
    if (whl_spd_factor > 1.015) {
      whl_spd_factor = 1.015;
    }
    if (whl_spd_factor < 0.985) {
      whl_spd_factor = 0.985;
    }
  } else {
    result.spd_sa = 0;
  }

  if (rtk_fix_time > 20.0 && std::abs(min_spd_whl) > 3.0 && msf_data.zupt_count > 0) {
    gyro_bias << msf_data.bias_gyro_x, msf_data.bias_gyro_y,
        msf_data.bias_gyro_z;
    // acc_bias << msf_data.bias_acc_x / earth.g.norm(),
    // msf_data.bias_acc_y / earth.g.norm(),
    // msf_data.bias_acc_z / earth.g.norm();
    result.imu_bias = 1;
  } else {
    result.imu_bias = 0;
  }

  return result;
}
