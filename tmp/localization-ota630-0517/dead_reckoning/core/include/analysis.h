#pragma once

#include <array>
#include <chrono>
#include <iostream>
#include <limits>

#include <Eigen/Dense>

#include "data_type.h"
// #include "eigen_utils.h"
#include "geoinfo_utils.h"

// using namespace byd::eigen_util;
using namespace byd::dr_data;
using CFG = class CONFIG {
 public:
  Eigen::Matrix<double, 9, 1> static_bound =
      Eigen::Matrix<double, 9, 1>::Ones() * 1e-5;
  Eigen::Vector3d gyro_bias_valid_mean_bound{0.0005, 0.0005, 0.0005};

 public:
  void set_static_bound(const Eigen::Matrix<double, 9, 1>& data) {
    static_bound = data;
  }
  void set_gyro_bias_valid_mean_bound(const Eigen::Matrix<double, 3, 1> data) {
    gyro_bias_valid_mean_bound = std::move(data);
  }
};

using VDSA = class VEHICEL_DRIVING_STATUS_ANALYSIS {
 public:
  CFG cfg;

  const size_t ACC_LONLAT_SKIP = 5;
  size_t acc_lonlat_count = 0;

 public:
  struct RESULT {
    size_t zupt : 1;    // zero update
    size_t lat_ac : 1;  // Latitudinal acceleration compensation
    size_t lon_ac : 1;  // Longitudinal acceleration compensation
    size_t alp_a : 1;   // alpha adaption
    RESULT() { *this = 0ul; }
    RESULT& operator=(size_t r) noexcept {
      *(size_t*)this = r;
      return *this;
    }
    explicit operator bool() const noexcept { return *(bool*)this; }
  };

 private:
  byd::geoinfo::EARTH earth;

 private:
  static const size_t BUFFER_SIZE = 42;
  size_t idx = BUFFER_SIZE - 1;
  size_t count = 0;
  std::array<Eigen::Vector3d, BUFFER_SIZE> vel;
  std::array<Eigen::Vector3d, BUFFER_SIZE> acc;
  std::array<Eigen::Vector3d, BUFFER_SIZE> gyro;
  // std::array<Eigen::Matrix3d, BUFFER_SIZE> ori;

  // VEL ACC GYRO
  Eigen::Matrix<double, 9, 1> max_;
  Eigen::Matrix<double, 9, 1> min_;
  // Eigen::Matrix<double, 9, 1> mean_;
  Eigen::Matrix<double, 3, 1> gyro_mean_;
  Eigen::Matrix<double, 3, 1> gyro_mean_left_;
  Eigen::Matrix<double, 3, 1> gyro_mean_middle_;
  Eigen::Matrix<double, 3, 1> gyro_mean_right_;

 private:
  double acc_lon_sm_pre = 0.0;
  double acc_lat_sm_pre = 0.0;

 public:
  double dacc_lon_sm = 0.0;
  double dacc_lat_sm = 0.0;

 public:
  VEHICEL_DRIVING_STATUS_ANALYSIS() {
    for (size_t i = 0; i < BUFFER_SIZE; i++) {
      vel[i] = Eigen::Vector3d::Zero();
      acc[i] = Eigen::Vector3d::Zero();
      gyro[i] = Eigen::Vector3d::Zero();
      // ori[i] = Eigen::Matrix3d::Identity();
    }
    max_.setZero();
    min_.setZero();
    gyro_mean_.setZero();
    gyro_mean_left_.setZero();
    gyro_mean_middle_.setZero();
    gyro_mean_right_.setZero();
    gyro_bias.setZero();
  }
  RESULT operator()(const Eigen::Vector3d& vel_, const Eigen::Vector3d& acc_,
                    const Eigen::Vector3d& gyro_, double dt, double acc_lon_sm,
                    double acc_lat_sm);

 public:
  Eigen::Vector3d gyro_bias;
  double latitudinal_acc = 0;
  double longitudinal_acc = 0;
  double alpha_factor = 0.01;

 private:
  bool is_static();
  void statics();
};

using DA = class DR_ADAPTION {
 public:
  struct RESULT {
    size_t yaw_ba : 1;    // yaw bias adaption
    size_t spd_sa : 1;    // wheel speed scale adaption
    size_t imu_bias : 1;  // imu bias using MSF estimation
    RESULT() { *this = 0ul; }
    RESULT& operator=(size_t r) noexcept {
      *(size_t*)this = r;
      return *this;
    }
    explicit operator bool() const noexcept { return *(bool*)this; }
  };

  RESULT operator()(const DrData&, const MsfData&, double whl_spd, double dt,
                    double rtk_fix_time);

  DR_ADAPTION() {
    yaw_bias = 0.0;
    whl_spd_factor = 1.0;
    gyro_bias << 0.0, 0.0, 0.0;
    acc_bias << 0.0, 0.0, 0.0;
    yaw_ref.fill(0.0);
    yaw_dr.fill(0.0);
    v_ref.fill(0.0);
    v_whl.fill(0.0);
  }

 private:
  byd::geoinfo::EARTH earth;

 public:
  double yaw_bias;
  double whl_spd_factor;
  Eigen::Vector3d gyro_bias, acc_bias;

 private:
  static const size_t BUFFER_SIZE = 20;
  size_t idx = BUFFER_SIZE - 1;
  size_t count = 0;
  std::array<double, BUFFER_SIZE> yaw_ref;
  std::array<double, BUFFER_SIZE> yaw_dr;
  std::array<double, BUFFER_SIZE> v_ref;
  std::array<double, BUFFER_SIZE> v_whl;
};

using AS = class ACC_SMOOTHER {
 public:
  ACC_SMOOTHER(){
    acc_buf.fill(0.0);
  }
 public:
  static const size_t WINDOW = 12;

 private:
  size_t idx = 0;
  std::array<double, WINDOW> acc_buf;

 public:
  double operator()(double acc) {
    idx++;
    if (idx >= WINDOW) {
      idx = 0;
    }
    acc_buf[idx] = acc;
    double acc_sum = 0.0;
    for (auto& acc : acc_buf) {
      acc_sum += acc;
    }
    return acc_sum / WINDOW;
  }
};