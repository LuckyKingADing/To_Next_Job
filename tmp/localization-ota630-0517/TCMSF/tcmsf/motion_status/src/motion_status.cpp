/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "motion_status.h"
#include "cyber/common/log.h"
namespace MSF {

// static MotionStatus& motion_status_instance = MotionStatus::GetInstance();

MotionStatus::MotionStatus() {
    LoadConfig();
    // PrintConfig();
    imu_vec.reserve(config.buffer_size);
    vehicle_vec.reserve(config.buffer_size);
    motion_status_vec.reserve(config.buffer_size);
}

MotionStatus &MotionStatus::GetInstance() {
    static MotionStatus motion_status_instance_ref;
    return motion_status_instance_ref;
}

void MotionStatus::PrintStatus() {
    switch (motion_status_vec.back().second) {
        case 0:
            AINFO << "kUnknow" << std::endl;
            break;
        case 1:
            AINFO << "kStationary" << std::endl;
            break;
        case 2:
            AINFO << "kRectilinear" << std::endl;
            break;
        case 3:
            AINFO << "kRectilinearCruising" << std::endl;
            break;
        case 4:
            AINFO << "kTurning" << std::endl;
            break;
        case 5:
            AINFO << "kUnstable" << std::endl;
            break;
        default:
            AINFO << "kUnknow" << std::endl;
            break;
    }
}

void MotionStatus::PushImuData(const MSF::ImuData &imu_data) {
    if (imu_vec.size() >= config.buffer_size) {
        imu_vec.erase(imu_vec.begin());
    }
    imu_vec.emplace_back(imu_data);
}

void MotionStatus::PushVehicleData(const MSF::VehicleData &vehicle_data) {
    if (vehicle_vec.size() >= config.buffer_size) {
        vehicle_vec.erase(vehicle_vec.begin());
    }
    vehicle_vec.emplace_back(vehicle_data);
}

void MotionStatus::LoadConfig() {
    config.buffer_size                  = parameters_sgt.get_buffer_size();
    config.max_steer_angle_for_turning  = parameters_sgt.get_max_steer_angle_for_turning();
    config.min_steer_angle_for_straight = parameters_sgt.get_min_steer_angle_for_straight();
    config.max_gyro_z_for_turning       = parameters_sgt.get_max_gyro_z_for_turning();
    config.min_gyro_z_for_straight      = parameters_sgt.get_min_gyro_z_for_straight();
    config.min_speed_for_static         = parameters_sgt.get_min_speed_for_static();
    config.min_acc_x_uniform            = parameters_sgt.get_min_acc_x_uniform();
    config.percent_for_valid            = parameters_sgt.get_percent_for_valid();
    config.observation_window           = parameters_sgt.get_observation_window();
}

void MotionStatus::PrintConfig() {
    AINFO << std::left;
    AINFO << std::setw(30) << "buffer_size: " << config.buffer_size << std::endl;
    AINFO << std::setw(30) << "max_steer_angle_for_turning: " << config.max_steer_angle_for_turning << std::endl;
    AINFO << std::setw(30) << "min_steer_angle_for_straight: " << config.min_steer_angle_for_straight << std::endl;
    AINFO << std::setw(30) << "max_gyro_z_for_turning: " << config.max_gyro_z_for_turning << std::endl;
    AINFO << std::setw(30) << "min_gyro_z_for_straight: " << config.min_gyro_z_for_straight << std::endl;
    AINFO << std::setw(30) << "min_speed_for_static: " << config.min_speed_for_static << std::endl;
    AINFO << std::setw(30) << "min_acc_x_uniform: " << config.min_acc_x_uniform << std::endl;
    AINFO << std::setw(30) << "percent_for_valid: " << config.percent_for_valid << std::endl;
    AINFO << std::setw(30) << "observation_window: " << config.observation_window << std::endl;
}

MOTION_STATUS MotionStatus::GetMotionStatus() {
    return motion_status_vec.back().second;
}

double MotionStatus::GetMotionStatusTmsp() {
    return motion_status_vec.back().first;
}

MOTION_STATUS MotionStatus::GetMotionStatus(double timestamp) {
    MOTION_STATUS motion_status = kUnknow;

    if (motion_status_vec.empty()) {
        motion_status = kUnknow;
    }

    if (timestamp < motion_status_vec.front().first || timestamp > motion_status_vec.back().first) {
        motion_status = kUnknow;
    } else if (timestamp == motion_status_vec.front().first) {
        motion_status = motion_status_vec.front().second;
    } else if (timestamp == motion_status_vec.back().first) {
        motion_status = motion_status_vec.back().second;
    }

    // from end to start
    for (int i = motion_status_vec.size() - 1; i > 0; --i) {
        if (timestamp < motion_status_vec[i].first && timestamp > motion_status_vec[i - 1].first) {
            auto &prev_status = motion_status_vec[i - 1];
            auto &next_status = motion_status_vec[i];
            if (prev_status.second == next_status.second) {
                return next_status.second;
            } else {
                double t1 = motion_status_vec[i].first - timestamp;
                double t2 = timestamp - motion_status_vec[i - 1].first;
                if (t1 < t2) {
                    return motion_status_vec[i].second;
                } else {
                    return motion_status_vec[i - 1].second;
                }
            }
        }
    }

    return motion_status;
}

void MotionStatus::InsertMotionStatus(double time, MOTION_STATUS status) {
    if (motion_status_vec.size() >= config.buffer_size) {
        motion_status_vec.erase(motion_status_vec.begin());
    }
    motion_status_vec.emplace_back(time, status);
}

void MotionStatus::UpdateStatus() {
    auto          motion_status_time = imu_vec.back().measurement_timestamp;
    MOTION_STATUS motion_status_current;

    double min_speed          = config.min_speed_for_static / 3.6;
    double max_gyro_rad       = config.max_gyro_z_for_turning * M_PI / 180;
    double min_gyro_rad       = config.min_gyro_z_for_straight * M_PI / 180;
    double max_steer_rad      = config.max_steer_angle_for_turning * M_PI / 180;
    double min_steer_rad      = config.min_steer_angle_for_straight * M_PI / 180;
    int    observation_window = config.observation_window;
    double dv_imu             = 0.0;
    double dyaw_imu           = 0.0;
    double gyro_sum           = 0.0;
    double gyro_avg           = 0.0;

    int imu_size     = imu_vec.size();
    int vehicle_size = vehicle_vec.size();
    if (imu_size < observation_window || vehicle_size < observation_window) {
        motion_status_current = kUnstable;
        InsertMotionStatus(motion_status_time, motion_status_current);
        return;
    }

    for (int i = observation_window - 1; i >= 0; --i) {
        gyro_sum += imu_vec[imu_size - 1 - i].gyro[2];
        auto current_imu_data  = imu_vec[imu_size - 1 - i];
        auto previous_imu_data = imu_vec[imu_size - 1 - i - 1];

        double dt         = current_imu_data.measurement_timestamp - previous_imu_data.measurement_timestamp;
        double acc_x_avg  = (current_imu_data.acc[0] + previous_imu_data.acc[0]) / 2;
        double gyro_z_avg = (current_imu_data.gyro[2] + previous_imu_data.gyro[2]) / 2;
        dv_imu            = dv_imu + acc_x_avg * dt;
        dyaw_imu          = dyaw_imu + gyro_z_avg * dt;
    }
    dyaw_imu = dyaw_imu * 180 / M_PI;
    dv_imu *= 3.6; //不太准
    gyro_avg = gyro_sum / observation_window;

    // AINFO << "dyaw_imu: " << dyaw_imu << std::endl;

    // 1. 先判断是不是转弯，如果是转弯，那就不可能是直行/匀速直行/静止
    if (std::fabs(vehicle_vec.back().steer) >= max_steer_rad) {
        if (std::fabs(gyro_avg) >= max_gyro_rad && vehicle_vec.back().speed_rl > min_speed && vehicle_vec.back().speed_rr > min_speed) {
            motion_status_current = kTurning;
            InsertMotionStatus(motion_status_time, motion_status_current);
            return;
        }
    }

    // 2. 判断是否是直行,方向盘一般情况不会出现大的抖动，所以方向盘先判断
    auto current_odo_data  = vehicle_vec[vehicle_size - 1];
    auto previous_odo_data = vehicle_vec[vehicle_size - 1 - observation_window];
    auto dv_rl             = current_odo_data.speed_rl - previous_odo_data.speed_rl;
    auto dv_rr             = current_odo_data.speed_rr - previous_odo_data.speed_rr;
    auto dv_vehicle        = std::fabs(dv_rl + dv_rr) / 2;
    auto v_vehicle         = std::fabs(current_odo_data.speed_rl + current_odo_data.speed_rr) / 2;

    auto dp_fl = (current_odo_data.fl_pls_cnt - previous_odo_data.fl_pls_cnt) & 0x3FF;
    auto dp_fr = (current_odo_data.fr_pls_cnt - previous_odo_data.fr_pls_cnt) & 0x3FF;
    auto dp_rl = (current_odo_data.rl_pls_cnt - previous_odo_data.rl_pls_cnt) & 0x3FF;
    auto dp_rr = (current_odo_data.rr_pls_cnt - previous_odo_data.rr_pls_cnt) & 0x3FF;
    // 测试发现，前后轮在行驶相同距离的时候，dp就会有差值，感觉是车轮大小不一样大，所以前后分开了
    auto dp_max_front = std::max({dp_fl, dp_fr});
    auto dp_min_front = std::min({dp_fl, dp_fr});
    auto d_dp_front   = dp_max_front - dp_min_front;

    auto dp_max_rear = std::max({dp_rl, dp_rr});
    auto dp_min_rear = std::min({dp_rl, dp_rr});
    auto d_dp_rear   = dp_max_rear - dp_min_rear;

    // AINFO << "d_dp_front: " << d_dp_front << std::endl;
    // AINFO << "d_dp_rear: " << d_dp_rear << std::endl;
    // AINFO << "current_odo_data.speed_rl: " << current_odo_data.speed_rl * 3.6 << std::endl;
    // AINFO << "current_odo_data.steer: " << current_odo_data.steer * 180 / M_PI << std::endl;

    if (std::fabs(vehicle_vec.back().steer) <= min_steer_rad && vehicle_vec.back().speed_rl > min_speed && vehicle_vec.back().speed_rr > min_speed) {
        // dp在直行的时候，最大能达到4,非常不科学，但是传感器性能确实是这样，只能把这个数字调大
        if (dyaw_imu > 0.5 || d_dp_front > 5 || d_dp_rear > 5 || gyro_avg > min_gyro_rad) {
            motion_status_current = kUnstable;
            InsertMotionStatus(motion_status_time, motion_status_current);
            return;
        }

        // AINFO << "dv_vehicle: " << dv_vehicle << std::endl;
        // AINFO << "v_vehicle: " << v_vehicle << std::endl;
        if (dv_vehicle <= v_vehicle * 0.02) {
            motion_status_current = kRectilinearCruising;
            InsertMotionStatus(motion_status_time, motion_status_current);
            return;
        }

        motion_status_current = kRectilinear;
        InsertMotionStatus(motion_status_time, motion_status_current);
        return;
    }

    // 3. 判断静止
    if (v_vehicle < (min_speed / 5)) {
        if (dp_fl == 0 && dp_fr == 0 && dp_rl == 0 && dp_rr == 0) {
            motion_status_current = kStationary;
            InsertMotionStatus(motion_status_time, motion_status_current);
            return;
        }
    }

    motion_status_current = kUnknow;
    InsertMotionStatus(motion_status_time, motion_status_current);
    return;
}

} // namespace MSF
