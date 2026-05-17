#include <fstream>
#include <iostream>

#include "parse_record.h"

#define GPS_TPC "/drivers/gnss/raw"
#define IMU_TPC "/drivers/imu/raw"
#define INS_TPC "/drivers/ins/raw"
#define VEH_TPC "/drivers/canbus/vehicle_info"
#define DR_TPC "/localization/dr"
#define MSF_TPC "/localization/ld_loc_result"

using byd::modules::localization::LocResult;
using byd::msg::drivers::Gps;
using byd::msg::drivers::Imu;
using byd::msg::drivers::VehInfo;
using byd::msg::localization::LocalizationEstimate;

int PARSER::run(const std::string &data_path, const std::string &output_path) {
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

    fs::path gnss_filepath(output_dir / "gnss.csv");
    fs::path vehicle_filepath(output_dir / "vehicle.csv");
    fs::path imu_filepath(output_dir / "imu.csv");
    fs::path msf_filepath(output_dir / "msf.csv");
    fs::path dr_filepath(output_dir / "dr.csv");

    std::fstream gnss_fs(gnss_filepath, std::ios::out);
    std::fstream veh_fs(vehicle_filepath, std::ios::out);
    std::fstream imu_fs(imu_filepath, std::ios::out);
    std::fstream msf_fs(msf_filepath, std::ios::out);
    std::fstream dr_fs(dr_filepath, std::ios::out);

    if (!gnss_fs || !veh_fs || !imu_fs || !msf_fs || !dr_fs) {
        std::cout << "Create file failed!" << std::endl;
    }

    std::vector<std::string> records;

    fs::directory_iterator iters(data_path);

    for (auto &iter : iters) {
        records.push_back(iter.path());
    }

    std::sort(records.begin(), records.end());

    std::shared_ptr<Gps>                  gps_ = std::make_shared<Gps>();
    std::shared_ptr<Imu>                  imu_ = std::make_shared<Imu>();
    std::shared_ptr<VehInfo>              veh_ = std::make_shared<VehInfo>();
    std::shared_ptr<LocalizationEstimate> dr_  = std::make_shared<LocalizationEstimate>();
    std::shared_ptr<LocResult>            msf_ = std::make_shared<LocResult>();

    auto gnss_cb_ = [&gnss_fs](const std::shared_ptr<Gps> &gps_) {
        double lon_mars, lat_mars;

        // wgtochina_lb(0, gps_->position().lon(), gps_->position().lat(), gps_->position().height(), 0, 0, &lon_mars, &lat_mars);
        lon_mars = gps_->position().lon();
        lat_mars = gps_->position().lat();
        gnss_fs << std::setprecision(14)                         //
                << gps_->header().publish_timestamp()            // 1
                << "," << gps_->header().measurement_timestamp() // 2
                << ',' << gps_->position().lat()                 // 3
                << ',' << gps_->position().lon()                 // 4
                << ',' << gps_->position().height()              // 5
                << ',' << gps_->linear_velocity().x()            // 6
                << ',' << gps_->linear_velocity().y()            // 7
                << ',' << gps_->linear_velocity().z()            // 8
                << ',' << gps_->heading()                        // 9
                << ',' << gps_->num_sats()                       // 10
                << ',' << gps_->position_status()                // 11
                << ',' << gps_->header().sequence_num()          // 12
                << ',' << lat_mars                               // 13
                << ',' << lon_mars                               // 14
                << std::endl;
    };

    auto imu_cb_ = [&imu_fs](const std::shared_ptr<Imu> &imu_) {
        imu_fs << std::setprecision(14)                         //
               << imu_->header().publish_timestamp()            // 1
               << ',' << imu_->header().measurement_timestamp() // 2
               << ',' << imu_->accel().x()                      // 3
               << ',' << imu_->accel().y()                      // 4
               << ',' << imu_->accel().z()                      // 5
               << ',' << imu_->gyro().x()                       // 6
               << ',' << imu_->gyro().y()                       // 7
               << ',' << imu_->gyro().z()                       // 8
               << ',' << imu_->header().sequence_num()          // 9
               << std::endl;                                    //
    };

    auto veh_cb_ = [&veh_fs, this](const std::shared_ptr<VehInfo> &veh_) {
        auto   dir_rl   = veh_->ego_motion_status().da_in_rlwhldrvdir_u8();
        auto   dir_rr   = veh_->ego_motion_status().da_in_rrwhldrvdir_u8();
        auto   vel_rl   = std::abs(veh_->ego_motion_status().da_in_rlwhlspd_sg());
        auto   vel_rr   = std::abs(veh_->ego_motion_status().da_in_rrwhlspd_sg());
        double speed_rl = 0.0, speed_rr = 0.0;
        if (dir_rl == 1) {
            speed_rl = vel_rl;
        } else {
            speed_rl = -vel_rl;
        }
        if (dir_rr == 1) {
            speed_rr = vel_rr;
        } else {
            speed_rr = -vel_rr;
        }
        double yaw_rate = veh_->ego_motion_status().da_in_yawrate_sg();

        auto veh_info_lp_ = veh_lpf({speed_rl, speed_rr, yaw_rate});

        veh_fs << std::setprecision(14)                         //
               << veh_->header().publish_timestamp()            // 1
               << ',' << veh_->header().measurement_timestamp() // 2
               << ',' << speed_rl                               // 3
               << ',' << speed_rr                               // 4
               << ',' << yaw_rate                               // 5
               << ',' << veh_->header().sequence_num()          // 6
               << ',' << veh_info_lp_[0]                        // 7
               << ',' << veh_info_lp_[1]                        // 8
               << ',' << veh_info_lp_[2]                        // 9
               << std::endl;                                    //
    };

    auto msf_cb_ = [&msf_fs](const std::shared_ptr<LocResult> &msf_) {
        msf_fs << std::setprecision(14)                         //
               << msf_->header().publish_timestamp()            // 1
               << "," << msf_->header().measurement_timestamp() // 2
               << "," << msf_->position().lat() * 180.0 / M_PI  // 3
               << "," << msf_->position().lon() * 180.0 / M_PI  // 4
               << "," << msf_->position().height()              // 5
               << "," << msf_->linear_velocity().x()            // 6
               << "," << msf_->linear_velocity().y()            // 7
               << "," << msf_->linear_velocity().z()            // 8
               << "," << msf_->acc_bias().x()                   // 9
               << "," << msf_->acc_bias().y()                   // 10
               << "," << msf_->acc_bias().z()                   // 11
               << "," << msf_->gyro_bias().x()                  // 12
               << "," << msf_->gyro_bias().y()                  // 13
               << "," << msf_->gyro_bias().z()                  // 14
               << "," << msf_->heading()                        // 15
               << "," << msf_->fusion_status()                  // 16
               << "," << msf_->header().sequence_num()          // 17
               << std::endl;
    };

    auto dr_cb_ = [&dr_fs](const std::shared_ptr<LocalizationEstimate> &dr_) {
        dr_fs << std::setprecision(14)                        //
              << dr_->header().publish_timestamp()            // 1
              << "," << dr_->header().measurement_timestamp() // 2
              << ',' << dr_->pose().position().x()            // 3
              << ',' << dr_->pose().position().y()            // 4
              << ',' << dr_->pose().position().z()            // 5
              << ',' << dr_->pose().orientation().qx()        // 6
              << ',' << dr_->pose().orientation().qy()        // 7
              << ',' << dr_->pose().orientation().qz()        // 8
              << ',' << dr_->pose().orientation().qw()        // 9
              << ',' << dr_->pose().heading()                 // 10
              << ',' << dr_->header().sequence_num()          // 11
              << std::endl;
    };

    for (auto record_ : records) {
        std::cout << record_ << "\n";
        apollo::cyber::record::RecordReader  reader(record_);
        apollo::cyber::record::RecordMessage msg_;
        while (reader.ReadMessage(&msg_)) {
            if (msg_.channel_name == GPS_TPC) {
                gps_->ParseFromString(msg_.content);
                gnss_cb_(gps_);
            }
            if (msg_.channel_name == VEH_TPC) {
                veh_->ParseFromString(msg_.content);
                veh_cb_(veh_);
            }
            if (msg_.channel_name == DR_TPC) {
                dr_->ParseFromString(msg_.content);
                dr_cb_(dr_);
            }
            if (msg_.channel_name == IMU_TPC) {
                imu_->ParseFromString(msg_.content);
                imu_cb_(imu_);
            }
            if (msg_.channel_name == MSF_TPC) {
                msf_->ParseFromString(msg_.content);
                msf_cb_(msf_);
            }
        }
    }

    gnss_fs.close();
    veh_fs.close();
    imu_fs.close();
    msf_fs.close();
    dr_fs.close();

    return 0;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cout << "usage: PARSER record_dir output_dir\n";
        return -1;
    }
    PARSER parser;
    parser.run(argv[1], argv[2]);
    return 0;
}