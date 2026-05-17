#pragma once

/*
! Deprecated !
*/

#include "modules/localization/src/TCMSF/tcmsf/sensor/base/include/base_type.h"
#include "modules/localization/src/TCMSF/tcmsf/sensor/base/include/tcmsf_config.h"
#include "toml++/toml.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <numeric>
#include <vector>

namespace MSF {
enum MOTION_STATUS {
    kUnknow              = 0, //未知，初始值，和不稳定做区分
    kStationary          = 1, //静止
    kRectilinear         = 2, //直线
    kRectilinearCruising = 3, //直线匀速
    kTurning             = 4, //转弯
    kUnstable            = 5  //不稳定
};

struct motion_status_config {
    // angle unit: degree
    // speed unit: km/h
    size_t buffer_size                  = 300;
    double max_steer_angle_for_turning  = 5.0;
    double min_steer_angle_for_straight = 2.0;
    double max_gyro_z_for_turning       = 1.5;
    double min_gyro_z_for_straight      = 0.5;
    double min_speed_for_static         = 1.0;
    double min_acc_x_uniform            = 0.02;
    double percent_for_valid            = 0.85;
    size_t observation_window           = 100;
};

class MotionStatus {

private:
    byd::tcmsf::config::Parameters &parameters_sgt = byd::tcmsf::config::Parameters::getInstance(byd::tcmsf::config::TCMSF_CONFIG_FILE_DIR_);

public:
    static MotionStatus &GetInstance();
    MOTION_STATUS        GetMotionStatus(double timestamp);
    MOTION_STATUS        GetMotionStatus();
    double               GetMotionStatusTmsp();
    void                 PushImuData(const MSF::ImuData &imu_data);
    void                 PushVehicleData(const MSF::VehicleData &vehicle_data);
    void                 PrintStatus();
    void                 UpdateStatus();

private:
    // static std::unique_ptr<MotionStatus> motion_status_instance_ptr;
    MotionStatus(const MotionStatus &)            = delete;
    MotionStatus &operator=(const MotionStatus &) = delete;
    MotionStatus();
    void LoadConfig();
    void PrintConfig();
    void InsertMotionStatus(double time, MOTION_STATUS status);

    std::vector<std::pair<double, MOTION_STATUS>> motion_status_vec;
    motion_status_config                          config;
    std::vector<MSF::ImuData>                     imu_vec;
    std::vector<MSF::VehicleData>                 vehicle_vec;
};
} // namespace MSF
