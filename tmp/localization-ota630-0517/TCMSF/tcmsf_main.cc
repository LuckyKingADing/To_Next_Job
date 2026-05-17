#include "tcmsf_main.h"
#include "Coord.h"
#include "cyber/cyber.h"
#include "fmt/format.h"
#include "modules/msg/environment_model_msgs/drive_boundary.pb.h"
#include "modules/msg/localization_msgs/vf_result.pb.h"
#include "replay_mode.h"

#include "imu_issue_fallback_interface.h"

#include <fstream>
#include <iostream>
#include <string>

#define PVT_TPC "/drivers/gnss/pvt"
#define GPS_TPC "/drivers/gnss/raw"
#define IMU_TPC "/drivers/imu/raw"
#define INS_TPC "/drivers/ins/raw"
#define VEH_TPC "/drivers/canbus/vehicle_info"
#define TCMSF_TPC "/localization/tcmsf"
#define DR_TPC "/localization/dr"
#define ROVER_TPC "/drivers/rover_rtcm/raw"
#define BASE_TPC "/drivers/base_rtcm/raw"
#define VF_TPC "/localization/vf/vf_result"
#define SDMM_TPC "/localization/sd_mapmatch_result"
#define DB_TPC "/perception/detection/drive_boundary"

using byd::modules::loc_vf::VFResult;
using byd::modules::localization::SDMapMatchResult;
using byd::modules::tcmsf::Pose;
using byd::msg::drivers::Gps;
using byd::msg::drivers::Imu;
using byd::msg::drivers::Ins;
using byd::msg::drivers::Rtcm;
using byd::msg::drivers::VehInfo;
using byd::msg::env_model::DriveBoundary;
using byd::msg::localization::LocalizationEstimate;

// 全局变量：pvtlc模式使用的数据源选择
// "pvt" -> 使用 PVT_TPC ("/drivers/gnss/pvt")
// "gnss" -> 使用 GPS_TPC ("/drivers/gnss/raw")
static std::string g_pvtlc_source_topic = "pvt";  // 默认使用pvt

TCMSF_::TCMSF_(const std::string &imu_config_path) {
    tcmsf_ = byd::tcmsf::TCMSF::create(imu_config_path);
}

// 设置pvtlc数据源topic
void TCMSF_::set_pvtlc_source(const std::string &source) {
    g_pvtlc_source_topic = source;
}

int TCMSF_::run(const std::string &data_path, const std::string &output_path) {
    namespace fs = std::filesystem;
    if (!fs::exists(data_path)) {
        return -1;
    }
    fs::directory_entry entry(data_path);
    if (!entry.is_directory() && !entry.is_regular_file()) {
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

    // fs::path gnss_filepath(output_dir / "gnss.csv");
    // fs::path vehicle_filepath(output_dir / "vehicle.csv");
    // fs::path imu_filepath(output_dir / "imu.csv");
    // fs::path msf_filepath(output_dir / "msf_veh_frame.csv");

    // std::fstream gnss_fs(gnss_filepath, std::ios::out);
    // std::fstream veh_fs(vehicle_filepath, std::ios::out);
    // std::fstream imu_fs(imu_filepath, std::ios::out);
    // std::fstream msf_fs(msf_filepath, std::ios::out);

#ifdef _TCMSF_WRITE_RESULT_TO_RECORD_
    fs::create_directory(output_dir / "result");
    apollo::cyber::record::RecordWriter writer;
#endif

    // if (!gnss_fs || !veh_fs || !imu_fs || !msf_fs) {
    //     std::cout << "Create file failed!" << std::endl;
    // }

    std::vector<fs::path> records;

    if (entry.is_directory()) {
        fs::directory_iterator iters(data_path);

        for (auto &iter : iters) {
            records.push_back(iter.path());
        }
        std::sort(records.begin(), records.end());
    } else {
        records.push_back(data_path);
    }

    std::shared_ptr<Gps>                  gps_  = std::make_shared<Gps>();
    std::shared_ptr<Imu>                  imu_  = std::make_shared<Imu>();
    std::shared_ptr<VehInfo>              veh_  = std::make_shared<VehInfo>();
    std::shared_ptr<LocalizationEstimate> dr_   = std::make_shared<LocalizationEstimate>();
    std::shared_ptr<VFResult>             vf_   = std::make_shared<VFResult>();
    std::shared_ptr<SDMapMatchResult>     sdmm_ = std::make_shared<SDMapMatchResult>();
    std::shared_ptr<DriveBoundary>        db_   = std::make_shared<DriveBoundary>();

    // auto gnss_cb_ = [&gnss_fs](const std::shared_ptr<Gps> &gps_) {
    //     gnss_fs << std::setprecision(14)                  //
    //             << gps_->header().measurement_timestamp() //
    //             << ',' << gps_->position().lat()          //
    //             << ',' << gps_->position().lon()          //
    //             << ',' << gps_->position().height()       //
    //             << ',' << gps_->linear_velocity().x()     //
    //             << ',' << gps_->linear_velocity().y()     //
    //             << ',' << gps_->linear_velocity().z()     //
    //             << ',' << gps_->heading()                 //
    //             << ',' << gps_->num_sats()                //
    //             << ',' << gps_->position_status()         //
    //             << std::endl;
    // };

    // auto imu_cb_ = [&imu_fs](const std::shared_ptr<Imu> &imu_) {
    //     imu_fs << std::setprecision(14)                  //
    //            << imu_->header().measurement_timestamp() //
    //            << ',' << imu_->accel().x()               //
    //            << ',' << imu_->accel().y()               //
    //            << ',' << imu_->accel().z()               //
    //            << ',' << imu_->gyro().x()                //
    //            << ',' << imu_->gyro().y()                //
    //            << ',' << imu_->gyro().z() << std::endl;
    // };

    // auto veh_cb_ = [&veh_fs](const std::shared_ptr<VehInfo> &veh_) {
    //     auto   dir_rl   = veh_->ego_motion_status().da_in_rlwhldrvdir_u8();
    //     auto   dir_rr   = veh_->ego_motion_status().da_in_rrwhldrvdir_u8();
    //     auto   vel_rl   = std::abs(veh_->ego_motion_status().da_in_rlwhlspd_sg());
    //     auto   vel_rr   = std::abs(veh_->ego_motion_status().da_in_rrwhlspd_sg());
    //     double speed_rl = 0.0, speed_rr = 0.0;
    //     if (dir_rl == 1) {
    //         speed_rl = vel_rl;
    //     } else {
    //         speed_rl = -vel_rl;
    //     }
    //     if (dir_rr == 1) {
    //         speed_rr = vel_rr;
    //     } else {
    //         speed_rr = -vel_rr;
    //     }
    //     double yaw_rate = veh_->ego_motion_status().da_in_yawrate_sg();
    //     veh_fs << std::setprecision(14)                  //
    //            << veh_->header().measurement_timestamp() //
    //            << ',' << speed_rl                        //
    //            << ',' << speed_rr                        //
    //            << ',' << yaw_rate << std::endl;
    // };

    std::string msf_msg_str_;
    bool        msf_msg_ready_ = false;

    auto result_cb_ = [this, &msf_msg_str_, &msf_msg_ready_]() {
        std::shared_ptr<Pose> tcmsf_result_msg_ = std::make_shared<Pose>();
        tcmsf_->output_msg(tcmsf_result_msg_);
        tcmsf_result_msg_->SerializeToString(&msf_msg_str_);

        static double pre_timestamp = 0.0;
        double        dt_           = tcmsf_result_msg_->header().measurement_timestamp() - pre_timestamp;

        msf_msg_ready_ = dt_ < 10.0;

        if (dt_ < 0.0) {
            AERROR << "frame disorder detected! dt = " << dt_;
        }
        pre_timestamp = tcmsf_result_msg_->header().measurement_timestamp();
        // msf_fs << //
        //     fmt::format("{:14.4f},{:d}\n",
        //                 tcmsf_result_msg_->header().measurement_timestamp(), //
        //                 (int)tcmsf_result_msg_->align_status()               //
        //     );
        // msf_fs << std::setprecision(14) << tcmsf_result_msg_->header().measurement_timestamp() // 量测时间  1
        //        << ',' << tcmsf_result_msg_->position().lat()                                   // 纬度     2
        //        << ',' << tcmsf_result_msg_->position().lon()                                   // 经度     3
        //        << ',' << tcmsf_result_msg_->position().height()                                // 高度     4
        //        << ',' << tcmsf_result_msg_->attitude().qw()                                    // 姿态 w   5
        //        << ',' << tcmsf_result_msg_->attitude().qx()                                    // 姿态 x   6
        //        << ',' << tcmsf_result_msg_->attitude().qy()                                    // 姿态 y   7
        //        << ',' << tcmsf_result_msg_->attitude().qz()                                    // 姿态 z   8
        //        << ',' << 0.0                                                                   // 姿态 r   9
        //        << ',' << 0.0                                                                   // 姿态 p   10
        //        << ',' << 0.0                                                                   // 姿态 y   11
        //        << ',' << tcmsf_result_msg_->velocity().x()                                     // 速度 x   12
        //        << ',' << tcmsf_result_msg_->velocity().y()                                     // 速度 y   13
        //        << ',' << tcmsf_result_msg_->velocity().z()                                     // 速度 z   14
        //        << ',' << tcmsf_result_msg_->heading()                                          // 航向     15
        //        << ',' << std::endl;
    };

    byd::tcmsf::config::Parameters &parameters_sgt = byd::tcmsf::config::Parameters::getInstance(byd::tcmsf::config::TCMSF_CONFIG_FILE_DIR_);
    for (auto record_ : records) {
        std::cout << record_ << "\n";

#ifdef _TCMSF_WRITE_RESULT_TO_RECORD_
        fs::path result_record_ = output_dir / "result" / record_.filename();
        writer.Open(result_record_);
#endif

#ifdef __TCMSF_DEBUG__ENABLE_REPLAY_MODE
        byd::replay_mode::ReplayModeSGT::getInstance(record_);
#endif

        auto iif_ = MSF::IIF::ImuIssueFallbackInterface::create();

        apollo::cyber::record::RecordReader  reader(record_);
        apollo::cyber::record::RecordMessage msg_;
        while (reader.ReadMessage(&msg_)) {
//             if (msg_.channel_name == GPS_TPC) {
//                 gps_->ParseFromString(msg_.content);
//                 double lat_mars = 0.0, lon_mars = 0.0;
//                 wgtochina_lb(0, gps_->position().lon(), gps_->position().lat(), gps_->position().height(), 0, 0, &lon_mars, &lat_mars);
//                 gps_->mutable_position()->set_lon(lon_mars);
//                 gps_->mutable_position()->set_lat(lat_mars);
//                 tcmsf_->insert_msg(gps_);
//                 // gnss_cb_(gps_);
// #ifdef _TCMSF_WRITE_RESULT_TO_RECORD_
//                 writer.WriteMessage(msg_.channel_name, msg_.content, msg_.time);
// #endif
//             }

            if (msg_.channel_name == GPS_TPC && parameters_sgt.get_gnss_fusion_mode() != byd::tcmsf::config::Parameters::GnssFusionMode::GNSS_LOOSE_COUPLE) {
                gps_->ParseFromString(msg_.content);
                if (GPS_TPC == "/drivers/gnss/raw") {
                    // raw topic 未加偏，这里需要做一下加偏
                    double lat_mars = 0.0, lon_mars = 0.0;
                    wgtochina_lb(0, gps_->position().lon(), gps_->position().lat(), gps_->position().height(), 0, 0, &lon_mars, &lat_mars);
                    gps_->mutable_position()->set_lon(lon_mars);
                    gps_->mutable_position()->set_lat(lat_mars);
                }
                tcmsf_->insert_msg(gps_);
                // gnss_cb_(gps_);
#ifdef _TCMSF_WRITE_RESULT_TO_RECORD_
                writer.WriteMessage(msg_.channel_name, msg_.content, msg_.time);
#endif
            }

            // PVTLC模式（GNSS_LOOSE_COUPLE）数据源选择逻辑
            // - "pvt": 使用 PVT_TPC ("/drivers/gnss/pvt")
            // - "gnss": 使用 GPS_TPC ("/drivers/gnss/raw")
            if (parameters_sgt.get_gnss_fusion_mode() == byd::tcmsf::config::Parameters::GnssFusionMode::GNSS_LOOSE_COUPLE) {
                bool use_this_msg = false;

                if (g_pvtlc_source_topic == "pvt") {
                    use_this_msg = (msg_.channel_name == PVT_TPC);
                } else if (g_pvtlc_source_topic == "gnss") {
                    use_this_msg = (msg_.channel_name == GPS_TPC);
                }

                if (use_this_msg) {
                    gps_->ParseFromString(msg_.content);
                    // gnss/raw topic需要做坐标加偏
                    if (msg_.channel_name == GPS_TPC) {
                        double lat_mars = 0.0, lon_mars = 0.0;
                        wgtochina_lb(0, gps_->position().lon(), gps_->position().lat(), gps_->position().height(), 0, 0, &lon_mars, &lat_mars);
                        gps_->mutable_position()->set_lon(lon_mars);
                        gps_->mutable_position()->set_lat(lat_mars);
                    }
                    tcmsf_->insert_msg(gps_);
#ifdef _TCMSF_WRITE_RESULT_TO_RECORD_
                    writer.WriteMessage(msg_.channel_name, msg_.content, msg_.time);
#endif
                }
            }
            if (msg_.channel_name == VEH_TPC) {
                veh_->ParseFromString(msg_.content);
                iif_->insert_veh(veh_);
                tcmsf_->insert_msg(veh_);
                // veh_cb_(veh_);
#ifdef _TCMSF_WRITE_RESULT_TO_RECORD_
                writer.WriteMessage(msg_.channel_name, msg_.content, msg_.time);
#endif
            }
            if (msg_.channel_name == DR_TPC) {
                dr_->ParseFromString(msg_.content);
                tcmsf_->insert_msg(dr_);
#ifdef _TCMSF_WRITE_RESULT_TO_RECORD_
                writer.WriteMessage(msg_.channel_name, msg_.content, msg_.time);
#endif
            }
            if (msg_.channel_name == SDMM_TPC) {
                sdmm_->ParseFromString(msg_.content);
                tcmsf_->insert_msg(sdmm_);
#ifdef _TCMSF_WRITE_RESULT_TO_RECORD_
                writer.WriteMessage(msg_.channel_name, msg_.content, msg_.time);
#endif
            }
            if (msg_.channel_name == VF_TPC) {
                vf_->ParseFromString(msg_.content);
                tcmsf_->insert_msg(vf_);
#ifdef _TCMSF_WRITE_RESULT_TO_RECORD_
                writer.WriteMessage(msg_.channel_name, msg_.content, msg_.time);
#endif
            }
            if (msg_.channel_name == DB_TPC) {
                db_->ParseFromString(msg_.content);
                tcmsf_->insert_msg(db_);
#ifdef _TCMSF_WRITE_RESULT_TO_RECORD_
                writer.WriteMessage(msg_.channel_name, msg_.content, msg_.time);
#endif
            }
            if (msg_.channel_name == IMU_TPC) {
                imu_->ParseFromString(msg_.content);
                auto imu_mod_ = iif_->insert_imu(imu_);
                tcmsf_->insert_msg(imu_mod_);
                tcmsf_->offline_mode_step(result_cb_);
                // imu_cb_(imu_);
#ifdef _TCMSF_WRITE_RESULT_TO_RECORD_
                writer.WriteMessage(msg_.channel_name, msg_.content, msg_.time);
                if (msf_msg_ready_) {
                    writer.WriteMessage(TCMSF_TPC, msf_msg_str_, msg_.time);
                }
#endif
            }
        }
    }
#ifdef _TCMSF_WRITE_RESULT_TO_RECORD_
    writer.Close();
#endif

    // gnss_fs.close();
    // veh_fs.close();
    // imu_fs.close();
    // msf_fs.close();

    return 0;
}

int main(int argc, char **argv) {
    // 更新参数说明，支持新的 --pvtlc-source 参数
    if (argc < 3) {
        std::cout << "usage: TCMSF record_dir output_dir [options]\n";
        std::cout << "options:\n";
        std::cout << "  --pvtlc-source <pvt|gnss>  Specify GNSS data source for pvtlc mode\n";
        std::cout << "                              'pvt' (default): use /drivers/gnss/pvt topic\n";
        std::cout << "                              'gnss': use /drivers/gnss/raw topic\n";
        std::cout << "  --config <path>             Specify imu config file path\n";
        std::cout << "\nexamples:\n";
        std::cout << "  TCMSF record_dir output_dir                              (use default pvt source)\n";
        std::cout << "  TCMSF record_dir output_dir --pvtlc-source gnss         (use gnss source for pvtlc)\n";
        std::cout << "  TCMSF record_dir output_dir --config imu_config.toml    (use custom config)\n";
        std::cout << "  TCMSF record_dir output_dir --pvtlc-source gnss --config config.toml\n";
        return -1;
    }

    // 解析命令行参数
    std::string imu_config_path = "";
    std::string pvtlc_source = "pvt";  // 默认使用pvt
    std::string data_path = argv[1];
    std::string output_path = argv[2];

    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--pvtlc-source" && i + 1 < argc) {
            pvtlc_source = argv[i + 1];
            i++;  // 跳过参数值
            std::cout << "[INFO] pvtlc source set to: " << pvtlc_source << "\n";
            if (pvtlc_source != "pvt" && pvtlc_source != "gnss") {
                std::cout << "[WARN] Invalid pvtlc-source value: " << pvtlc_source << ", using default 'pvt'\n";
                pvtlc_source = "pvt";
            }
        } else if (arg == "--config" && i + 1 < argc) {
            imu_config_path = argv[i + 1];
            i++;  // 跳过参数值
            std::cout << "[INFO] imu config path set to: " << imu_config_path << "\n";
        } else if (arg.find("--") != 0) {
            // 兼容旧格式：第三个参数是config路径（不带--config前缀）
            imu_config_path = arg;
            std::cout << "[INFO] imu config path set to: " << imu_config_path << " (legacy format)\n";
        }
    }

    TCMSF_ tcmsf(imu_config_path);
    tcmsf.set_pvtlc_source(pvtlc_source);
    tcmsf.run(data_path, output_path);

    return 0;
}