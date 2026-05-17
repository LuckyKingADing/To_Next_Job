#pragma once

#include <vector>

#include <Eigen/Dense>

namespace byd {
namespace dr_data {
using SenSta = enum SensorStatus {
  OK = 0,        // Good
  WARNNING = 1,  // Precision loss
  ERROR = 2,     // No guarantee, may diverge shortly
  FATAL = 3      // Bad state, unusable
};

typedef struct INS_DATA {
  double timestamp;
  double pos_x;
  double pos_y;
  double pos_z;
  double vel_x;
  double vel_y;
  double vel_z;
  double ori_x;
  double ori_y;
  double ori_z;
  double ori_w;
  double heading;
  INS_DATA() {
    timestamp = 0.0;
    pos_x = 0.0;
    pos_y = 0.0;
    pos_z = 0.0;
    vel_x = 0.0;
    vel_y = 0.0;
    vel_z = 0.0;
    ori_x = 0.0;
    ori_y = 0.0;
    ori_z = 0.0;
    ori_w = 0.0;
    heading = 0.0;
  }
} InsData;

typedef struct MSF_DATA {
  double timestamp;
  double pos_x;
  double pos_y;
  double pos_z;
  double vel_x;
  double vel_y;
  double vel_z;
  double ori_x;
  double ori_y;
  double ori_z;
  double ori_w;
  double bias_acc_x;
  double bias_acc_y;
  double bias_acc_z;
  double bias_gyro_x;
  double bias_gyro_y;
  double bias_gyro_z;
  double heading;
  int msf_state;
  int zupt_count;
  MSF_DATA() {
    timestamp = 0.0;
    pos_x = 0.0;
    pos_y = 0.0;
    pos_z = 0.0;
    vel_x = 0.0;
    vel_y = 0.0;
    vel_z = 0.0;
    ori_x = 0.0;
    ori_y = 0.0;
    ori_z = 0.0;
    ori_w = 0.0;
    bias_acc_x = 0.0;
    bias_acc_y = 0.0;
    bias_acc_z = 0.0;
    bias_gyro_x = 0.0;
    bias_gyro_y = 0.0;
    bias_gyro_z = 0.0;
    heading = 0.0;
    msf_state = 0;
    zupt_count = 0;
  }
} MsfData;

typedef struct IMU_DATA {
  double timestamp;
  double acc_x;
  double acc_y;
  double acc_z;
  double gyro_x;
  double gyro_y;
  double gyro_z;
  IMU_DATA() {
    timestamp = 0.0;
    acc_x = 0.0;
    acc_y = 0.0;
    acc_z = 1.0;
    gyro_x = 0.0;
    gyro_y = 0.0;
    gyro_z = 0.0;
  }
} ImuData;

typedef struct GPS_DATA {
  double timestamp;
  double pos_x;
  double pos_y;
  double pos_z;
  double vel_x;
  double vel_y;
  double vel_z;
  int status;
  GPS_DATA() {
    timestamp = 0.0;
    pos_x = 0.0;
    pos_y = 0.0;
    pos_z = 0.0;
    vel_x = 0.0;
    vel_y = 0.0;
    vel_z = 0.0;
    status = 0;
  }
} GpsData;

typedef struct VEH_DATA {
  double timestamp;
  double spd_fl;
  double spd_fr;
  double spd_rl;
  double spd_rr;
  double spd;
  double yaw_rate;
  double lat_acc;
  double lon_acc;
  int cnt_fl;
  int cnt_fr;
  int cnt_rl;
  int cnt_rr;
  int dir_fl;
  int dir_fr;
  int dir_rl;
  int dir_rr;
  bool spd_fl_v;
  bool spd_fr_v;
  bool spd_rl_v;
  bool spd_rr_v;
  bool spd_v;
  bool yaw_rate_v;
  bool lat_acc_v;
  bool lon_acc_v;
  VEH_DATA() {
    timestamp = 0.0;
    spd_fl = 0.0;
    spd_fr = 0.0;
    spd_rl = 0.0;
    spd_rr = 0.0;
    spd = 0.0;
    yaw_rate = 0.0;
    lat_acc = 0.0;
    lon_acc = 0.0;
    cnt_fl = 0;
    cnt_fr = 0;
    cnt_rl = 0;
    cnt_rr = 0;
    dir_fl = 0;
    dir_fr = 0;
    dir_rl = 0;
    dir_rr = 0;
    spd_fl_v = false;
    spd_fr_v = false;
    spd_rl_v = false;
    spd_rr_v = false;
    spd_v = false;
    yaw_rate_v = false;
    lat_acc_v = false;
    lon_acc_v = false;
  }
} VehData;

typedef struct DR_DATA {
  double timestamp;
  double pos_x;
  double pos_y;
  double pos_z;
  double ori_x;
  double ori_y;
  double ori_z;
  double ori_w;
  double acc_x;
  double acc_y;
  double acc_z;
  double gyro_x;
  double gyro_y;
  double gyro_z;
  double vel_x;
  double vel_y;
  double vel_z;
  double heading;
  SenSta imu_state;
  SenSta veh_state;
  DR_DATA() {
    timestamp = 0.0;
    pos_x = 0.0;
    pos_y = 0.0;
    pos_z = 0.0;
    ori_x = 0.0;
    ori_y = 0.0;
    ori_z = 0.0;
    ori_w = 0.0;
    acc_x = 0.0;
    acc_y = 0.0;
    acc_z = 0.0;
    gyro_x = 0.0;
    gyro_y = 0.0;
    gyro_z = 0.0;
    vel_x = 0.0;
    vel_y = 0.0;
    vel_z = 0.0;
    heading = 0.0;
    imu_state = SenSta::OK;
    veh_state = SenSta::OK;
  }
} DrData;

}  // namespace dr_data
}  // namespace byd
