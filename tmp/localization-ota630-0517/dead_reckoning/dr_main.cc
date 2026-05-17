#include "dr_main.h"
#include "fmt/format.h"
#include <filesystem>
#include <fstream>
#include <iostream>

#define GPS_TPC "/drivers/gnss/raw"
#define IMU_TPC "/drivers/imu/raw"
#define INS_TPC "/drivers/ins/raw"
#define VEH_TPC "/drivers/canbus/vehicle_info"
#define MSF_TPC "/localization/tcmsf"
#define DR_TPC "/localization/dr"
#define ModSta_TPC "/localization/dr/status"

constexpr double rad2deg = 180.0 / M_PI;
constexpr double deg2rad = M_PI / 180.0;

void DR_::refine_imu(std::shared_ptr<Imu> imu_, std::shared_ptr<Pose> msf_) {
    if (std::fabs(imu_->header().measurement_timestamp() - msf_->header().measurement_timestamp()) < 1.0) {
        imu_->mutable_gyro()->set_x(imu_->gyro().x() - msf_->gyro_bias().x() * rad2deg);
        imu_->mutable_gyro()->set_y(imu_->gyro().y() - msf_->gyro_bias().y() * rad2deg);
        imu_->mutable_gyro()->set_z(imu_->gyro().z() - msf_->gyro_bias().z() * rad2deg);
    }
}
void DR_::refine_wheel(std::shared_ptr<VehInfo> veh_, std::shared_ptr<Pose> msf_) {
    if (std::fabs(veh_->header().measurement_timestamp() - msf_->header().measurement_timestamp()) < 1.0) {
        veh_->mutable_ego_motion_status()->set_da_in_rlwhlspd_sg(veh_->ego_motion_status().da_in_rlwhlspd_sg() * (1.0 + msf_->vehicle_bias().y()));
        veh_->mutable_ego_motion_status()->set_da_in_rrwhlspd_sg(veh_->ego_motion_status().da_in_rrwhlspd_sg() * (1.0 + msf_->vehicle_bias().y()));
    }
}

int DR_::run(const std::string &data_path, const std::string &output_path) {
    using namespace byd::dr_data;
    namespace fs = std::filesystem;
    if (!fs::exists(data_path)) {
        return -1;
    }
    fs::directory_entry entry(data_path);
    if (!entry.is_directory()) {
        return -1;
    }

    if (!fs::exists(output_path)) {
        if (!fs::create_directories(output_path)) {
            return -1;
        }
    }
    fs::directory_entry entry_output(output_path);
    if (!entry_output.is_directory()) {
        return -1;
    }

    auto output_dir = fs::path(output_path);

    fs::path dr_filepath(output_dir / "dr_replay.csv");

    std::fstream dr_fs(dr_filepath, std::ios::out);

    if (!dr_fs) {
        std::cout << "Create file failed!" << std::endl;
    }

    std::vector<std::string> records;

    fs::directory_iterator iters(data_path);

    for (auto &iter : iters) {
        records.push_back(iter.path());
    }

    std::sort(records.begin(), records.end());

    std::shared_ptr<Gps>                  gps_msg_ = std::make_shared<Gps>();
    std::shared_ptr<Imu>                  imu_msg_ = std::make_shared<Imu>();
    std::shared_ptr<VehInfo>              veh_msg_ = std::make_shared<VehInfo>();
    std::shared_ptr<LocalizationEstimate> dr_msg_  = std::make_shared<LocalizationEstimate>();
    std::shared_ptr<Pose>                 msf_msg_ = std::make_shared<Pose>();

    std::string dr_header_str_ =
        "pub_time,"     // 1
        "measure_time," // 2
        "pos_f,"        // 3
        "pos_l,"        // 4
        "pos_u,"        // 5
        "q_x,"          // 6
        "q_y,"          // 7
        "q_z,"          // 8
        "q_w,"          // 9
        "heading,"      // 10
        "sequence_num," // 11
        "angle,"        // 12
        "eulr_x,"       // 13
        "eulr_y,"       // 14
        "eulr_z"        // 15
        "\n";

    dr_fs << dr_header_str_;

    auto dr_cb_ = [&dr_fs](const DrData &dr_, double timestamp_) {
        static uint32_t sequence_num = 0;
        sequence_num++;
        Eigen::Quaterniond q_;
        q_.x() = dr_.ori_x;
        q_.y() = dr_.ori_y;
        q_.z() = dr_.ori_z;
        q_.w() = dr_.ori_w;
        Eigen::AngleAxisd aa(q_);

        Eigen::Vector3d eulr_ = INS::quaternion2euler(q_);

        auto line = fmt::format(
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}," //
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}," //
            "{:d},{:.4f},{:.4f},{:.4f},{:.4f}\n", //
            dr_.timestamp,                        // 1
            timestamp_,                           // 2
            dr_.pos_x,                            // 3
            dr_.pos_y,                            // 4
            dr_.pos_z,                            // 5
            dr_.ori_x,                            // 6
            dr_.ori_y,                            // 7
            dr_.ori_z,                            // 8
            dr_.ori_w,                            // 9
            dr_.heading,                          // 10
            sequence_num,                         // 11
            aa.angle(),                           // 12
            eulr_.x() * 180.0 / M_PI,             // 13
            eulr_.y() * 180.0 / M_PI,             // 14
            eulr_.z() * 180.0 / M_PI              // 15
        );
        dr_fs << line;
    };

    auto result_cb_ = [this, &dr_fs, &dr_cb_](double timestamp_) {
        dr_cb_(dr_->get_result(), timestamp_);
    };

    for (auto record_ : records) {
        std::cout << record_ << "\n";
        apollo::cyber::record::RecordReader  reader(record_);
        apollo::cyber::record::RecordMessage msg_;
        while (reader.ReadMessage(&msg_)) {
            if (msg_.channel_name == GPS_TPC) {
                gps_msg_->ParseFromString(msg_.content);
                {
                    if (gps_msg_->has_header() &&
                        gps_msg_->header().has_measurement_timestamp() &&
                        gps_msg_->has_position_status()) {
                        GpsData gps_data;
                        gps_data.timestamp = gps_msg_->header().measurement_timestamp();
                        gps_data.status    = gps_msg_->position_status();
                        dr_->insert_gps(gps_data);
                    }
                }
                // gnss_cb_(gps_);
            }
            if (msg_.channel_name == VEH_TPC) {
                veh_msg_->ParseFromString(msg_.content);
                {
                    if (veh_msg_->has_header() &&
                        veh_msg_->header().has_measurement_timestamp() &&
                        veh_msg_->has_ego_motion_status() &&
                        veh_msg_->ego_motion_status().has_da_in_rlwhlspd_sg() &&
                        veh_msg_->ego_motion_status().has_da_in_rlwhlspd_v_u8() &&
                        veh_msg_->ego_motion_status().has_da_in_rrwhlspd_sg() &&
                        veh_msg_->ego_motion_status().has_da_in_rrwhlspd_v_u8() &&
                        veh_msg_->ego_motion_status().has_da_in_rlwhldrvdir_u8() &&
                        veh_msg_->ego_motion_status().has_da_in_rrwhldrvdir_u8() &&
                        veh_msg_->ego_motion_status().has_da_in_rlwhldrvdir_v_u8() &&
                        veh_msg_->ego_motion_status().has_da_in_rrwhldrvdir_v_u8() &&
                        veh_msg_->has_propulsion_system_status() &&
                        veh_msg_->propulsion_system_status().has_da_in_trmgearlvl_u8() &&
                        veh_msg_->ego_motion_status().has_da_in_yawrate_sg()) {
                        refine_wheel(veh_msg_, msf_msg_);
                        if (veh_msg_->ego_motion_status().da_in_rlwhlspd_v_u8() &&
                            veh_msg_->ego_motion_status().da_in_rrwhlspd_v_u8()) {
                            VehData veh_data;
                            veh_data.timestamp = veh_msg_->header().measurement_timestamp();

                            auto dir_rl = veh_msg_->ego_motion_status().da_in_rlwhldrvdir_u8();
                            auto dir_rr = veh_msg_->ego_motion_status().da_in_rrwhldrvdir_u8();

                            auto dir_rl_v = veh_msg_->ego_motion_status().da_in_rlwhldrvdir_v_u8();
                            auto dir_rr_v = veh_msg_->ego_motion_status().da_in_rrwhldrvdir_v_u8();

                            auto trmgear =
                                veh_msg_->propulsion_system_status().da_in_trmgearlvl_u8();

                            auto vel_rl =
                                std::abs(veh_msg_->ego_motion_status().da_in_rlwhlspd_sg());
                            auto vel_rr =
                                std::abs(veh_msg_->ego_motion_status().da_in_rrwhlspd_sg());

                            if (dir_rl_v == 0 && dir_rr_v == 0) {
                                if ((vel_rl > 1e-10 || vel_rr > 1e-10) &&
                                    (dir_rl == 3 || dir_rr == 3)) {
                                    // 这里考虑轮速方向的延迟
                                    // 如果出现轮速有值，但是方向给静止的话，就使用挡位来判断轮速的方向
                                    veh_data.spd_rl = (trmgear == 4) ? -vel_rl : vel_rl;
                                    veh_data.spd_rr = (trmgear == 4) ? -vel_rr : vel_rr;
                                } else {
                                    // 左右后轮行驶方向状态均有效
                                    veh_data.spd_rl = (dir_rl == 2) ? -vel_rl : vel_rl;
                                    veh_data.spd_rr = (dir_rr == 2) ? -vel_rr : vel_rr;
                                }
                            } else if (dir_rl_v == 0 || dir_rr_v == 0) {
                                // 左右后轮行驶方向状态一个无效，一个有效
                                if (dir_rl_v == 0) {
                                    veh_data.spd_rl = (dir_rl == 2) ? -vel_rl : vel_rl;
                                    veh_data.spd_rr = (dir_rl == 2) ? -vel_rr : vel_rr;
                                }
                                if (dir_rr_v == 0) {
                                    veh_data.spd_rr = (dir_rr == 2) ? -vel_rr : vel_rr;
                                    veh_data.spd_rl = (dir_rr == 2) ? -vel_rl : vel_rl;
                                }
                            } else {
                                // 左右后轮行驶方向标志都无效
                                if (trmgear == 4) {
                                    veh_data.spd_rl = -vel_rl;
                                    veh_data.spd_rr = -vel_rr;
                                } else {
                                    veh_data.spd_rl = vel_rl;
                                    veh_data.spd_rr = vel_rr;
                                }
                            }

                            veh_data.yaw_rate = veh_msg_->ego_motion_status().da_in_yawrate_sg();
                            dr_->insert_veh(veh_data);
                        }
                    }
                    // veh_cb_(veh_);
                }
            }
            if (msg_.channel_name == IMU_TPC) {
                imu_msg_->ParseFromString(msg_.content);
                {
                    if (imu_msg_->has_header() &&
                        imu_msg_->header().has_measurement_timestamp() &&
                        imu_msg_->has_accel() && imu_msg_->accel().has_x() &&
                        imu_msg_->accel().has_y() && imu_msg_->accel().has_z() &&
                        imu_msg_->has_gyro() && imu_msg_->gyro().has_x() &&
                        imu_msg_->gyro().has_y() && imu_msg_->gyro().has_z() &&
                        imu_msg_->has_imu_status()) {
                        refine_imu(imu_msg_, msf_msg_);
                        ImuData imu_data;
                        imu_data.timestamp = imu_msg_->header().measurement_timestamp();
                        imu_data.acc_x     = imu_msg_->accel().x();
                        imu_data.acc_y     = imu_msg_->accel().y();
                        imu_data.acc_z     = imu_msg_->accel().z();
                        imu_data.gyro_x    = imu_msg_->gyro().x() / 180.0 * M_PI;
                        imu_data.gyro_y    = imu_msg_->gyro().y() / 180.0 * M_PI;
                        imu_data.gyro_z    = imu_msg_->gyro().z() / 180.0 * M_PI;
                        if (std::isnan(imu_data.acc_x) || std::isnan(imu_data.gyro_x) ||
                            std::isnan(imu_data.acc_y) || std::isnan(imu_data.gyro_y) ||
                            std::isnan(imu_data.acc_z) || std::isnan(imu_data.gyro_z)) {
                            // 检查是否有NaN，如果有则抛弃
                            AERROR << "IMU msg NaN detected, drop!";
                        } else if (imu_msg_->imu_status() != 1) {
                            // 检查IMU状态是否OK，如果不则抛弃
                            AERROR_EVERY(100) << "imu status is not ok, drop!";
                        } else {
                            dr_->insert_imu(imu_data);
                        }
                        static double pre_timestamp_ = 0.0;
                        double        dt             = imu_data.timestamp - pre_timestamp_;
                        if (dt > 0.0 && std::fabs(dt) < 1.0) {
                            dr_->dr_step(dt);
                            result_cb_(imu_data.timestamp);
                        }
                        pre_timestamp_ = imu_data.timestamp;
                    }
                }
                // imu_cb_(imu_);
            }

            if (msg_.channel_name == MSF_TPC) {
                msf_msg_->ParseFromString(msg_.content);
                {
                    if (msf_msg_->has_header() &&
                        msf_msg_->header().has_measurement_timestamp() &&
                        msf_msg_->has_velocity() && msf_msg_->velocity().has_x() &&
                        msf_msg_->velocity().has_y() && msf_msg_->velocity().has_z() &&
                        msf_msg_->has_acc_bias() && msf_msg_->has_gyro_bias() &&
                        msf_msg_->acc_bias().has_x() && msf_msg_->acc_bias().has_y() &&
                        msf_msg_->acc_bias().has_z() && msf_msg_->gyro_bias().has_x() &&
                        msf_msg_->gyro_bias().has_y() && msf_msg_->gyro_bias().has_z() &&
                        msf_msg_->has_heading() && msf_msg_->has_zupt_count() &&
                        msf_msg_->has_fusion_status()) {
                        MsfData msf_data;
                        msf_data.timestamp   = msf_msg_->header().measurement_timestamp();
                        msf_data.vel_x       = msf_msg_->velocity().x();
                        msf_data.vel_y       = msf_msg_->velocity().y();
                        msf_data.vel_z       = msf_msg_->velocity().z();
                        msf_data.bias_acc_x  = msf_msg_->acc_bias().x();
                        msf_data.bias_acc_y  = msf_msg_->acc_bias().y();
                        msf_data.bias_acc_z  = msf_msg_->acc_bias().z();
                        msf_data.bias_gyro_x = msf_msg_->gyro_bias().x();
                        msf_data.bias_gyro_y = msf_msg_->gyro_bias().y();
                        msf_data.bias_gyro_z = msf_msg_->gyro_bias().z();
                        msf_data.heading     = msf_msg_->heading();
                        msf_data.msf_state   = msf_msg_->fusion_status();
                        msf_data.zupt_count  = msf_msg_->zupt_count();
                        if (msf_msg_->fusion_status() ==
                                Pose::FusionStatus::Pose_FusionStatus_FULLSTATE ||
                            msf_msg_->fusion_status() ==
                                Pose::FusionStatus::Pose_FusionStatus_GPSONLY) {
                            // dr_->insert_msf(msf_data);
                        }
                    }
                }
            }
        }
    }

    dr_fs.close();

    return 0;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cout << "usage: DR record_dir output_dir\n";
        return -1;
    }
    DR_ dr;
    dr.run(argv[1], argv[2]);
    return 0;
}