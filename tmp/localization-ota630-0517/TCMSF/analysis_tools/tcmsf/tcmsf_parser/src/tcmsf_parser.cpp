#include "Eigen/Dense"
#include "fmt/format.h"
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "tcmsf_parser.h"

#include "local_trans.h"

#include "rigid_transform.h"
// #include "proj_utils4.hpp"

#include "tcmsf_lever.h"
#include "tcmsf_config.h"
// #include "resolve_post.h"

#define GPS_TPC "/drivers/gnss/raw"
#define PVT_TPC "/drivers/gnss/pvt"
#define GPS_TPC_02 "/drivers/gnss/packet"
#define IMU_TPC "/drivers/imu/raw"
#define INS_TPC "/drivers/ins/raw"
#define VEH_TPC "/drivers/canbus/vehicle_info"
#define TCMSF_TPC "/localization/tcmsf"
#define DR_TPC "/localization/dr"
#define VF_TPC "/localization/vf/vf_result"
#define ROVER_TPC "/drivers/rover_rtcm/raw"
#define BASE_TPC "/drivers/base_rtcm/raw"
#define MSF_TPC "/localization/ld_loc_result"
#define LOCALMAP_TPC "/localization/local_map"
#define ROUTING_MAP_TPC "/noa_map/routing_map"

using byd::modules::loc_vf::VFResult;
using byd::modules::localization::LocResult;
using byd::modules::tcmsf::Pose;
using byd::msg::drivers::Gps;
using byd::msg::drivers::Imu;
using byd::msg::drivers::Rtcm;
using byd::msg::drivers::VehInfo;
using byd::msg::localization::LocalizationEstimate;
using byd::msg::localization::LocalizationMapEngineMessage;
using byd::msg::orin::routing_map::RoutingMap;

int PARSER::run(const std::string &data_path, const std::string &output_path, const std::string &lever_config_path, const std::string &replay) {
    namespace fs = std::filesystem;
    uint64_t mars_flag = 0;
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

    fs::path output_dir(output_path);
    if (replay == "replay") {
        output_dir = output_dir / "replay";
    }
    else if (replay == "mars"){
        mars_flag = 1;
    }

    fs::path gnss_filepath(output_dir / "gnss.csv");
    fs::path pvt_filepath(output_dir / "pvt.csv");
    fs::path gnss_02_filepath(output_dir / "gnss_02.csv");
    fs::path vehicle_filepath(output_dir / "vehicle.csv");
    fs::path imu_filepath(output_dir / "imu.csv");
    fs::path msf_filepath(output_dir / "msf.csv");
    fs::path dr_filepath(output_dir / "dr.csv");
    fs::path tcmsf_filepath(output_dir / "tcmsf.csv");
    fs::path vf_filepath(output_dir / "vf.csv");
    fs::path routingmap_filepath(output_dir / "routingmap.csv");
    fs::path localmap_dir(output_dir / "localmap");
    fs::path rover_filepath(output_dir / "rover.csv");
    if (!fs::exists(localmap_dir)) {
        if (!fs::create_directories(localmap_dir)) {
            return -1;
        }
    }


    std::fstream gnss_fs(gnss_filepath, std::ios::out);
    std::fstream pvt_fs(pvt_filepath, std::ios::out);
    std::fstream gnss_02_fs(gnss_02_filepath, std::ios::out);
    std::fstream veh_fs(vehicle_filepath, std::ios::out);
    std::fstream imu_fs(imu_filepath, std::ios::out);
    std::fstream msf_fs(msf_filepath, std::ios::out);
    std::fstream dr_fs(dr_filepath, std::ios::out);
    std::fstream tcmsf_fs(tcmsf_filepath, std::ios::out);
    std::fstream vf_fs(vf_filepath, std::ios::out);
    std::fstream routingmap_fs(routingmap_filepath, std::ios::out);
    std::fstream rover_fs(rover_filepath, std::ios::out);

    if (!gnss_fs || !pvt_fs || !gnss_02_fs || !veh_fs || !imu_fs || !msf_fs || !dr_fs || !tcmsf_fs || !vf_fs || !routingmap_fs || !rover_fs) {
        std::cout << "Create file failed!" << std::endl;
    }

    constexpr double rad2deg = 180.0 / M_PI;
    constexpr double deg2rad = M_PI / 180.0;

    // 从配置文件解析杆臂
    VehicleInfo::Lever lever = VehicleInfo::Lever::fromJson(lever_config_path);

    const static uint64_t TCMSF_QUEUE_MAX_SIZE = 200;
    std::deque<Pose>      tcmsf_queue;

    std::vector<std::string> records;

    if (entry.is_directory()) {
        fs::directory_iterator iters(data_path);

        for (auto &iter : iters) {
            records.push_back(iter.path());
        }
        std::sort(records.begin(), records.end());
    } else {
        records.push_back(data_path);
    }

    std::shared_ptr<Gps>                          gps_        = std::make_shared<Gps>();
    std::shared_ptr<Gps>                          pvt_        = std::make_shared<Gps>();
    std::shared_ptr<Gps>                          gps_02_     = std::make_shared<Gps>();
    std::shared_ptr<Imu>                          imu_        = std::make_shared<Imu>();
    std::shared_ptr<VehInfo>                      veh_        = std::make_shared<VehInfo>();
    std::shared_ptr<LocalizationEstimate>         dr_         = std::make_shared<LocalizationEstimate>();
    std::shared_ptr<LocResult>                    msf_        = std::make_shared<LocResult>();
    std::shared_ptr<VFResult>                     vf_         = std::make_shared<VFResult>();
    std::shared_ptr<Pose>                         tcmsf_      = std::make_shared<Pose>();
    std::shared_ptr<LocalizationMapEngineMessage> localmap_   = std::make_shared<LocalizationMapEngineMessage>();
    std::shared_ptr<RoutingMap>                   routingmap_ = std::make_shared<RoutingMap>();
    std::shared_ptr<Rtcm>                  rover_rtcm_ = std::make_shared<Rtcm>();
    std::shared_ptr<Rtcm>                  base_rtcm_ = std::make_shared<Rtcm>();

    std::string gps_header_str_ =
        "pub_time,"     // 1
        "measure_time," // 2
        "lat,"          // 3
        "lon,"          // 4
        "height,"       // 5
        "vel_e,"        // 6
        "vel_n,"        // 7
        "vel_u,"        // 8
        "heading,"      // 9
        "num_sats,"     // 10
        "rtk_status,"   // 11
        "sequence_num," // 12
        "lat_mars,"     // 13
        "lon_mars\n";   // 14

    std::string gps_02_header_str_ =
        "pub_time,"       // 1
        "measure_time,"   // 2
        "lat,"            // 3
        "lon,"            // 4
        "height,"         // 5
        "vel_e,"          // 6
        "vel_n,"          // 7
        "vel_u,"          // 8
        "heading,"        // 9
        "num_sats,"       // 10
        "rtk_status,"     // 11
        "sequence_num\n"; // 12

    std::string imu_header_str_ =
        "pub_time,"     // 1
        "measure_time," // 2
        "acc_x,"        // 3
        "acc_y,"        // 4
        "acc_z,"        // 5
        "gyro_x,"       // 6
        "gyro_y,"       // 7
        "gyro_z,"       // 8
        "sequence_num," // 9
        "angle_vel\n";  // 10

    std::string veh_header_str_ =
        "pub_time,"       // 1
        "measure_time,"   // 2
        "spd_rl,"         // 3
        "spd_rr,"         // 4
        "yaw_rate,"       // 5
        "sequence_num\n"; // 6

    std::string msf_header_str_ =
        "pub_time,"       // 1
        "measure_time,"   // 2
        "lat_mars,"       // 3
        "lon_mars,"       // 4
        "height,"         // 5
        "vel_e,"          // 6
        "vel_n,"          // 7
        "vel_u,"          // 8
        "bias_acc_x,"     // 9
        "bias_acc_y,"     // 10
        "bias_acc_z,"     // 11
        "bias_gyro_x,"    // 12
        "bias_gyro_y,"    // 13
        "bias_gyro_z,"    // 14
        "heading,"        // 15
        "fusion_status,"  // 16
        "sequence_num\n"; // 17

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

    std::string vf_header_str_ =
        "pub_time,"           // 1
        "measure_time,"       // 2
        "offset_f,"           // 3
        "offset_l,"           // 4
        "offset_u,"           // 5
        "offset_heading,"     // 6
        "offset_std_f,"       // 7
        "offset_std_l,"       // 8
        "offset_std_u,"       // 9
        "offset_std_heading," // 10
        "offset_lat_mars,"    // 11 映射到融合定位结果上
        "offset_lon_mars,"    // 12
        "sequence_num\n";     // 13

    std::string tcmsf_header_str_ =
        "pub_time,"                      // 1
        "measure_time,"                  // 2
        "lat_mars_imu,"                  // 3
        "lon_mars_imu,"                  // 4
        "lat_mars_antenna,"              // 5
        "lon_mars_antenna,"              // 6
        "lat_mars_vehicle,"              // 7
        "lon_mars_vehicle,"              // 8
        "lat_mars_imu_with_mapbias,"     // 9
        "lon_mars_imu_with_mapbias,"     // 10
        "lat_mars_antenna_with_mapbias," // 11
        "lon_mars_antenna_with_mapbias," // 12
        "lat_mars_vehicle_with_mapbias," // 13
        "lon_mars_vehicle_with_mapbias," // 14
        "rtk_status,"                    // 15
        "fusion_status,"                 // 16
        "align_status,"                  // 17
        "map_bias_yaw,"                  // 18
        "heading,"                       // 19
        "sequence_num,"                  // 20
        "vel_e,"                         // 21
        "vel_n,"                         // 22
        "vel_u,"                         // 23
        "angle,"                         // 24
        "eulr_x,"                        // 25
        "eulr_y,"                        // 26
        "eulr_z,"                        // 27
        "bg_x,"                          // 28
        "bg_y,"                          // 29
        "bg_z,"                          // 30
        "height,"                        // 31
        "vel_ego_R,"                     // 32
        "vel_ego_F,"                     // 33
        "vel_ego_U,"                     // 34
        "ba_x,"                          // 35
        "ba_y,"                          // 36
        "ba_z,"                          // 37
        "imu_bias_pitch,"                // 38
        "wheel_bias,"                    // 39
        "imu_bias_yaw"                   // 40
        "\n";                            //

    std::string routingmap_header_str_ =
        "pub_time,"     // 1
        "measure_time," // 2
        "ld_linkid,"    // 3
        "ld_offset,"    // 4
        "sd_linkid,"    // 5
        "sd_offset\n";  // 6

    std::string rtcm_header_str_ =
        "pub_time,"     // 1
        "measure_time," // 2
        "sequence_num,"    // 3
        "size,"    // 4  // 6
        ;

    gnss_fs << gps_header_str_;
    pvt_fs << gps_header_str_;
    gnss_02_fs << gps_02_header_str_;
    veh_fs << veh_header_str_;
    imu_fs << imu_header_str_;
    msf_fs << msf_header_str_;
    dr_fs << dr_header_str_;
    tcmsf_fs << tcmsf_header_str_;
    vf_fs << vf_header_str_;
    routingmap_fs << routingmap_header_str_;
    rover_fs << routingmap_header_str_;

    byd::geo::LocalTrans trans;

    auto gnss_cb_ = [this](const std::shared_ptr<Gps> &gps_,std::fstream &fs) {
        double lon_mars, lat_mars;
        wgtochina_lb(0, gps_->position().lon(), gps_->position().lat(), gps_->position().height(), 0, 0, &lon_mars, &lat_mars);
        // lon_mars  = gps_->position().lon();
        // lat_mars  = gps_->position().lat();
        auto line = fmt::format(
            "{:.4f},{:.4f},{:.8f},{:.8f},{:.4f},"   //
            "{:.4f},{:.4f},{:.4f},{:.4f},{:d},"     //
            "{:d},{:d},{:.8f},{:.8f},{:.4f},{:.4f},{:.4f}\n",            //
            gps_->header().publish_timestamp(),     // 1
            gps_->sec_in_gps_week(), // 2
            gps_->position().lat(),                 // 3
            gps_->position().lon(),                 // 4
            gps_->position().height(),              // 5
            gps_->linear_velocity().x(),            // 6
            gps_->linear_velocity().y(),            // 7
            gps_->linear_velocity().z(),            // 8
            gps_->heading(),                        // 9
            gps_->num_sats(),                       // 10
            (int)gps_->position_status(),           // 12
            gps_->header().sequence_num(),          // 12
            lat_mars,                               // 13
            lon_mars,                                // 14
            gps_->position_std().lat(),
            gps_->position_std().lon(),
            gps_->position_std().height()
            // gps_->header().measurement_timestamp() // 2,{:.4f}
        );
        this->dt_yukong_sow_ = gps_->header().measurement_timestamp() - gps_->sec_in_gps_week();
        if((int)gps_->position_status() > 0) fs << line;
    };

    std::vector<std::string> debug_infos;
    // ObservationDetails od={0};
    // byd::tcmsf::rtcm::ResolvePost rpost(debug_infos, od);
    // auto rtcm_cb_ = [&rpost,this](const std::shared_ptr<Rtcm> &rtcm_,std::fstream &rtcm_fs) {
    //     std::vector<uint8_t> rtcm;
    //     for (int i = 0; i < rtcm_->value_size(); i++)
    //         rtcm.emplace_back(rtcm_->value(i));
        
    //     size_t prot=(size_t)(rtcm_->header().publish_timestamp());
    //     gtime_t timepoint = {prot,0};
    //     rpost.update_roverinfo(&timepoint,rtcm);
    //     double sow = rpost.get_rover_sow();
    //     if (sow < 1) return;
    //     auto line = fmt::format(
    //         "{:.4f},{:.2f},"      //
    //         "{:d},{:d},{:.4f}\n",                             //
    //         rtcm_->header().publish_timestamp() - this->dt_yukong_sow_,     // 1 目前publish_timestamp 几乎与measurement_timestamp相等，都是publish的时间
    //         sow, // 2 //解析其sow
    //         rtcm_->header().sequence_num(),           // 12
    //         rtcm_->value_size(),rtcm_->header().measurement_timestamp()
    //     );
    //     rtcm_fs << line;
    // };

    auto gnss_02_cb_ = [&gnss_02_fs](const std::shared_ptr<Gps> &gps_02_) {
        auto line = fmt::format(
            "{:.4f},{:.4f},{:.8f},{:.8f},{:.4f},"      //
            "{:.4f},{:.4f},{:.4f},{:.4f},{:d},"        //
            "{:d},{:d}\n",                             //
            gps_02_->header().publish_timestamp(),     // 1
            gps_02_->header().measurement_timestamp(), // 2
            gps_02_->position().lat(),                 // 3
            gps_02_->position().lon(),                 // 4
            gps_02_->position().height(),              // 5
            gps_02_->linear_velocity().x(),            // 6
            gps_02_->linear_velocity().y(),            // 7
            gps_02_->linear_velocity().z(),            // 8
            gps_02_->heading(),                        // 9
            gps_02_->num_sats(),                       // 10
            (int)gps_02_->position_status(),           // 12
            gps_02_->header().sequence_num()           // 12
        );
        gnss_02_fs << line;
    };

    auto imu_cb_ = [&imu_fs](const std::shared_ptr<Imu> &imu_) {
        Eigen::Vector3d angle_vel;
        angle_vel << imu_->gyro().x(), imu_->gyro().y(), imu_->gyro().z();
        double angle_vel_ = angle_vel.norm();
        if (imu_->gyro().z() < 0.0) {
            angle_vel_ = -angle_vel_;
        }
        auto line = fmt::format(
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"   //
            "{:.4f},{:.4f},{:.4f},{:d},{:.4f}\n",   //
            imu_->header().publish_timestamp(),     // 1
            imu_->header().measurement_timestamp(), // 2
            imu_->accel().x(),                      // 3
            imu_->accel().y(),                      // 4
            imu_->accel().z(),                      // 5
            imu_->gyro().x(),                       // 6
            imu_->gyro().y(),                       // 7
            imu_->gyro().z(),                       // 8
            imu_->header().sequence_num(),          // 9
            angle_vel_                              // 10
        );
        imu_fs << line;
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
        auto   line     = fmt::format(
                  "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"   //
                  "{:d}\n",                               //
                  veh_->header().publish_timestamp(),     // 1
                  veh_->header().measurement_timestamp(), // 2
                  speed_rl,                               // 3
                  speed_rr,                               // 4
                  yaw_rate * 180.0 / M_PI,                // 5
                  veh_->header().sequence_num()           // 6
              );
        veh_fs << line;
    };

    auto msf_cb_ = [&msf_fs](const std::shared_ptr<LocResult> &msf_) {
        auto line = fmt::format(
            "{:.4f},{:.4f},{:.8f},{:.8f},{:.4f},"   //
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"   //
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"   //
            "{:d},{:d}\n",                          //
            msf_->header().publish_timestamp(),     // 1
            msf_->header().measurement_timestamp(), // 2
            msf_->position().lat() * 180.0 / M_PI,  // 3
            msf_->position().lon() * 180.0 / M_PI,  // 4
            msf_->position().height(),              // 5
            msf_->linear_velocity().x(),            // 6
            msf_->linear_velocity().y(),            // 7
            msf_->linear_velocity().z(),            // 8
            msf_->acc_bias().x(),                   // 9
            msf_->acc_bias().y(),                   // 10
            msf_->acc_bias().z(),                   // 11
            msf_->gyro_bias().x(),                  // 12
            msf_->gyro_bias().y(),                  // 13
            msf_->gyro_bias().z(),                  // 14
            msf_->heading(),                        // 15
            (int)msf_->fusion_status(),             // 16
            msf_->header().sequence_num()           // 17
        );
        msf_fs << line;
    };

    auto dr_cb_ = [&dr_fs](const std::shared_ptr<LocalizationEstimate> &dr_) {
        Eigen::Quaterniond q_;
        q_.x() = dr_->pose().orientation().qx();
        q_.y() = dr_->pose().orientation().qy();
        q_.z() = dr_->pose().orientation().qz();
        q_.w() = dr_->pose().orientation().qw();
        Eigen::AngleAxisd aa(q_);

        Eigen::Vector3d eulr_ = INS::quaternion2euler(q_);

        auto line = fmt::format(
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"  //
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"  //
            "{:d},{:.4f},{:.4f},{:.4f},{:.4f}\n",  //
            dr_->header().publish_timestamp(),     // 1
            dr_->header().measurement_timestamp(), // 2
            dr_->pose().position().x(),            // 3
            dr_->pose().position().y(),            // 4
            dr_->pose().position().z(),            // 5
            dr_->pose().orientation().qx(),        // 6
            dr_->pose().orientation().qy(),        // 7
            dr_->pose().orientation().qz(),        // 8
            dr_->pose().orientation().qw(),        // 9
            dr_->pose().heading(),                 // 10
            dr_->header().sequence_num(),          // 11
            aa.angle(),                            // 12
            eulr_.x() * 180.0 / M_PI,              // 13
            eulr_.y() * 180.0 / M_PI,              // 14
            eulr_.z() * 180.0 / M_PI               // 15
        );
        dr_fs << line;
    };

    auto vf_cb_ = [&vf_fs, &tcmsf_queue, &trans](const std::shared_ptr<VFResult> &vf_) {
        double offset_lat_mars, offset_lon_mars;
        auto   t_cmp_func = [](const Pose &data, double t) -> bool { return data.header().measurement_timestamp() < t; };
        auto   lb         = std::lower_bound(tcmsf_queue.begin(), tcmsf_queue.end(), vf_->header().measurement_timestamp(), t_cmp_func);
        if (lb != tcmsf_queue.end()) {
            double lon_mars, lat_mars;
            // wgtochina_lb(0, lb->position().lon(), lb->position().lat(), lb->position().height(), 0, 0, &lon_mars, &lat_mars);
            lon_mars = lb->position().lon();
            lat_mars = lb->position().lat();
            Eigen::Quaterniond att;
            att.w() = lb->attitude().qw();
            att.x() = lb->attitude().qx();
            att.y() = lb->attitude().qy();
            att.z() = lb->attitude().qz();

            Eigen::Vector3d lla_ref;
            lla_ref << lat_mars * deg2rad, lon_mars * deg2rad, lb->position().height();
            Eigen::Vector3d offset_RFU;
            offset_RFU << -vf_->offset().offset().y(), vf_->offset().offset().x(), vf_->offset().offset().z();
            Eigen::Vector3d lla_vf_mars = trans.DPos_Ego2LLA(lla_ref, -offset_RFU, att);
            offset_lat_mars             = lla_vf_mars.x() * rad2deg;
            offset_lon_mars             = lla_vf_mars.y() * rad2deg;
        } else {
            AINFO << "no near tcmsf msg found";
            return;
        }
        auto line = fmt::format(
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"  //
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"  //
            "{:.8f},{:.8f},{:d}\n",                //
            vf_->header().publish_timestamp(),     // 1
            vf_->header().measurement_timestamp(), // 2
            vf_->offset().offset().x(),            // 3
            vf_->offset().offset().y(),            // 4
            vf_->offset().offset().z(),            // 5
            vf_->offset().heading(),               // 6
            vf_->offset_std().offset().x(),        // 7
            vf_->offset_std().offset().y(),        // 8
            vf_->offset_std().offset().z(),        // 9
            vf_->offset_std().heading(),           // 10
            offset_lat_mars,                       // 11
            offset_lon_mars,                       // 12
            vf_->header().sequence_num()           // 13
        );
        vf_fs << line;
    };

auto tcmsf_cb_ = [&tcmsf_fs, &tcmsf_queue, &trans, &lever, &mars_flag](const std::shared_ptr<Pose> &tcmsf_) {
        tcmsf_queue.push_back(*tcmsf_);
        while (tcmsf_queue.size() > TCMSF_QUEUE_MAX_SIZE) {
            tcmsf_queue.pop_front();
        }
// mars坐标转换处理
        if (mars_flag)
        {
            double lat_02 = 0.0, lon_02 = 0.0;
            wgtochina_lb(0, tcmsf_->position().lon(), tcmsf_->position().lat(), 0.0, 0, 0, &lon_02, &lat_02);
            tcmsf_->mutable_position()->set_lat(lat_02);
            tcmsf_->mutable_position()->set_lon(lon_02);
        }

        // ========== 基本数据提取 ==========

        // 车辆后轴中心位置（角度制，mars系，已包含map_bias）
        // 参考：ENU坐标系，position = 真实位置 + map_bias
        Eigen::Vector3d pos_vehicle_map_llarad;
        pos_vehicle_map_llarad << tcmsf_->position().lat() * deg2rad,
            tcmsf_->position().lon() * deg2rad,
            tcmsf_->position().height();

        // map_bias: ENU系下的制图误差，xyz分别对应东、北、天方向
        // z分量是yaw偏差（弧度），不是位置偏差，位置计算只用x和y
        Eigen::Vector3d map_bias_enu;
        map_bias_enu << tcmsf_->map_bias().x(), tcmsf_->map_bias().y(), 0.0;

        // map_bias_yaw 默认为零（map_bias.z 不是航向偏差）
        double map_bias_yaw = 0.0;

        // 姿态四元数：IMU body系 → ENU参考系的旋转
        Eigen::Quaterniond att;
        att.w() = tcmsf_->attitude().qw();
        att.x() = tcmsf_->attitude().qx();
        att.y() = tcmsf_->attitude().qy();
        att.z() = tcmsf_->attitude().qz();

        // ========== 坐标转换推导 ==========

        // 步骤1：将map_bias从ENU系转换到RFU系（自车坐标系）
        // 推导：R = att.toRotationMatrix() 将 RFU → ENU
        //       R^T 将 ENU → RFU
        //       map_bias_rfu = R^T * map_bias_enu
        Eigen::Vector3d map_bias_rfu = att.toRotationMatrix().transpose() * map_bias_enu;

        // 步骤2：计算车辆后轴中心的真实位置（去除map_bias）
        // 推导：真实位置 = 地图位置 - map_bias
        //       DPos_Ego2LLA 将 RFU位移转换到LLA变化量
        //       传入 -map_bias_rfu（RFU系），函数内部转为 -map_bias_enu
        //       结果：pos_vehicle_map - map_bias = pos_vehicle_true
        Eigen::Vector3d pos_vehicle_true_llarad = trans.DPos_Ego2LLA(pos_vehicle_map_llarad, -map_bias_rfu, att);

        // 步骤3：转换杆臂从FLU系到RFU系
        // 推导：杆臂定义在FLU系（前-左-上）
        //       RFU系（右-前-上）：FLU的前→RFU左负方向，FLU的左→RFU右正方向
        //       FLU2RFU = [[0,-1,0], [1,0,0], [0,0,1]]
        //       lever_rfu = FLU2RFU * lever_flu
        Eigen::Vector3d lever_antenna_rfu = INS::frame_trans.FLU2RFU * lever.antenna;
        Eigen::Vector3d lever_imu_rfu     = INS::frame_trans.FLU2RFU * lever.imu;

        // 步骤4：计算各传感器真实位置（从车辆真实位置加杆臂）
        // 推导：传感器位置 = 车辆位置 + 杆臂
        //       杆臂是RFU系下从车辆中心到传感器的位移
        Eigen::Vector3d pos_antenna_true_llarad = trans.DPos_Ego2LLA(pos_vehicle_true_llarad, lever_antenna_rfu, att);
        Eigen::Vector3d pos_imu_true_llarad     = trans.DPos_Ego2LLA(pos_vehicle_true_llarad, lever_imu_rfu, att);

        // 步骤5：计算各传感器地图位置（从车辆地图位置加杆臂）
        // 推导：地图位置 = 真实位置 + map_bias（系统误差，所有点同样偏差）
        //       验证：(车辆真实+map_bias) + 杆臂 = 车辆地图 + 杆臂 = 传感器地图
        Eigen::Vector3d pos_antenna_map_llarad = trans.DPos_Ego2LLA(pos_vehicle_map_llarad, lever_antenna_rfu, att);
        Eigen::Vector3d pos_imu_map_llarad     = trans.DPos_Ego2LLA(pos_vehicle_map_llarad, lever_imu_rfu, att);

        // 步骤6：转换结果到角度制
        double lat_mars_imu                  = pos_imu_true_llarad.x() * rad2deg;
        double lon_mars_imu                  = pos_imu_true_llarad.y() * rad2deg;
        double lat_mars_antenna              = pos_antenna_true_llarad.x() * rad2deg;
        double lon_mars_antenna              = pos_antenna_true_llarad.y() * rad2deg;
        double lat_mars_vehicle              = pos_vehicle_true_llarad.x() * rad2deg;
        double lon_mars_vehicle              = pos_vehicle_true_llarad.y() * rad2deg;
        double lat_mars_imu_with_mapbias     = pos_imu_map_llarad.x() * rad2deg;
        double lon_mars_imu_with_mapbias     = pos_imu_map_llarad.y() * rad2deg;
        double lat_mars_antenna_with_mapbias = pos_antenna_map_llarad.x() * rad2deg;
        double lon_mars_antenna_with_mapbias = pos_antenna_map_llarad.y() * rad2deg;
        double lat_mars_vehicle_with_mapbias = tcmsf_->position().lat();
        double lon_mars_vehicle_with_mapbias = tcmsf_->position().lon();

        // ========== 其他数据提取 ==========

        double heading = tcmsf_->heading();

        Eigen::Vector3d vel_enu(tcmsf_->velocity().x(), tcmsf_->velocity().y(), tcmsf_->velocity().z());
        Eigen::Vector3d vel_ego_rfu = att.toRotationMatrix().transpose() * vel_enu;

        Eigen::AngleAxisd aa(att);
        Eigen::Vector3d   eulr_ = INS::quaternion2euler(att);

        auto line = fmt::format(
            "{:.4f},{:.4f},{:.8f},{:.8f},{:.8f},"      //
            "{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},"      //
            "{:.8f},{:.8f},{:.8f},{:.8f},{:d},"        //
            "{:d},{:d},{:.8f},{:.8f},{:d},"            //
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"      //
            "{:.4f},{:.4f},{:.5f},{:.5f},{:.5f},"      //
            "{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},"      //
            "{:.5f},{:.5f},{:.5f},{:.5f},{:.5f}\n",    //
            tcmsf_->header().publish_timestamp(),      // 1
            tcmsf_->header().measurement_timestamp(),  // 2
            lat_mars_imu,                              // 3
            lon_mars_imu,                              // 4
            lat_mars_antenna,                          // 5
            lon_mars_antenna,                          // 6
            lat_mars_vehicle,                          // 7
            lon_mars_vehicle,                          // 8
            lat_mars_imu_with_mapbias,                 // 9
            lon_mars_imu_with_mapbias,                 // 10
            lat_mars_antenna_with_mapbias,             // 11
            lon_mars_antenna_with_mapbias,             // 12
            lat_mars_vehicle_with_mapbias,             // 13
            lon_mars_vehicle_with_mapbias,             // 14
            (int)tcmsf_->gnss_status(),                // 15
            (int)tcmsf_->fusion_status(),              // 16
            (int)tcmsf_->align_status(),               // 17
            map_bias_yaw,                              // 18
            heading,                                   // 19
            tcmsf_->header().sequence_num(),           // 20
            tcmsf_->velocity().x(),                    // 21
            tcmsf_->velocity().y(),                    // 22
            tcmsf_->velocity().z(),                    // 23
            aa.angle(),                                // 24
            eulr_.x() * 180.0 / M_PI,                  // 25
            eulr_.y() * 180.0 / M_PI,                  // 26
            eulr_.z() * 180.0 / M_PI,                  // 27
            // eulr_2.x() * 180.0 / M_PI,                  // 25
            // eulr_2.y() * 180.0 / M_PI,                  // 26
            // eulr_2.z() * 180.0 / M_PI,                  // 27
            tcmsf_->gyro_bias().x() * 180.0 / M_PI,    // 28
            tcmsf_->gyro_bias().y() * 180.0 / M_PI,    // 29
            tcmsf_->gyro_bias().z() * 180.0 / M_PI,    // 30
tcmsf_->position().height(),               // 31
            vel_ego_rfu.x(),                           // 32
            vel_ego_rfu.y(),                           // 33
            vel_ego_rfu.z(),                           // 34
            tcmsf_->acc_bias().x(),                    // 35
            tcmsf_->acc_bias().y(),                    // 36
            tcmsf_->acc_bias().z(),                    // 37
            tcmsf_->vehicle_bias().x() * 180.0 / M_PI, // 38
            tcmsf_->vehicle_bias().y(),                // 39
            tcmsf_->vehicle_bias().z() * 180.0 / M_PI  // 40
        );
        tcmsf_fs << line;
    };

    auto localmap_cb_ = [&localmap_dir](const std::shared_ptr<LocalizationMapEngineMessage> &localmap_) {
        if (localmap_->api_name() == "local_map_update") {
            std::string  filename = "ldmap_" + std::to_string(localmap_->header().measurement_timestamp()) + ".txt";
            fs::path     localmap_filepath(localmap_dir / filename);
            std::fstream lm_fs(localmap_filepath, std::ios::out);
            if (!lm_fs) {
                std::cout << "Create localmap file failed!" << std::endl;
                return;
            }

            lm_fs << "localmap update, timestamp:" << std::setprecision(16) << localmap_->header().measurement_timestamp() << std::endl;
            lm_fs << "link num: " << localmap_->local_map().links_size() << std::endl;
            lm_fs << "lane num: " << localmap_->local_map().lanes_size() << std::endl;
            lm_fs << "boundary num: " << localmap_->local_map().lane_boundarys_size() << std::endl;

            for (int i = 0; i < localmap_->local_map().lanes_size(); i++) {
                auto &lane         = localmap_->local_map().lanes()[i];
                auto &boundary_ids = lane.boundary_ids();
                for (int j = 0; j < boundary_ids.size(); j++) {
                    lm_fs << "linkid:" << lane.link_id() << ", laneid:" << lane.id()
                          << ", boundaryid:" << boundary_ids[j] << std::endl;
                }
            }

            for (int i = 0; i < localmap_->local_map().lane_boundarys_size(); i++) {
                auto &boundary = localmap_->local_map().lane_boundarys()[i];
                auto &geos     = boundary.geometry();
                for (int j = 0; j < geos.size(); j++) {
                    lm_fs << "linkid:" << boundary.link_id() << ", boundaryid:" << boundary.id()
                          << ", lon:" << geos[j].lon() << ", lat:" << geos[j].lat() << std::endl;
                }
            }
        }
    };

    auto routingmap_cb_ = [&routingmap_fs](const std::shared_ptr<RoutingMap> &routingmap_) {
        auto line = fmt::format(
            "{:.6f},{:.6f},{:d},{:.4f},{:d},{:.4f}\n",
            routingmap_->header().publish_timestamp(),         // 1
            routingmap_->header().measurement_timestamp(),     // 2
            routingmap_->route().navi_start().section_id(),    // 3
            routingmap_->route().navi_start().s_offset(),      // 4
            routingmap_->sd_route().navi_start().section_id(), // 5
            routingmap_->sd_route().navi_start().s_offset()    // 6
        );
        routingmap_fs << line;
    };

    for (auto record_ : records) {
        std::cout << record_ << "\n";
        apollo::cyber::record::RecordReader  reader(record_);
        apollo::cyber::record::RecordMessage msg_;
        while (reader.ReadMessage(&msg_)) {
            if (msg_.channel_name == GPS_TPC) {
                gps_->ParseFromString(msg_.content);
                gnss_cb_(gps_,gnss_fs);
            }
            if (msg_.channel_name == PVT_TPC) {
                pvt_->ParseFromString(msg_.content);
                gnss_cb_(pvt_,pvt_fs);
            }
            // if (msg_.channel_name == ROVER_TPC && dt_yukong_sow_ > 1.0) {
            //     rover_rtcm_->ParseFromString(msg_.content);
            //     rtcm_cb_(rover_rtcm_,rover_fs);
            // }
            if (msg_.channel_name == GPS_TPC_02) {
                gps_02_->ParseFromString(msg_.content);
                gnss_02_cb_(gps_02_);
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
            if (msg_.channel_name == VF_TPC) {
                vf_->ParseFromString(msg_.content);
                vf_cb_(vf_);
            }
            if (msg_.channel_name == TCMSF_TPC) {
                tcmsf_->ParseFromString(msg_.content);
                tcmsf_cb_(tcmsf_);
            }
            if (msg_.channel_name == LOCALMAP_TPC) {
                localmap_->ParseFromString(msg_.content);
                localmap_cb_(localmap_);
            }
            if (msg_.channel_name == ROUTING_MAP_TPC) {
                routingmap_->ParseFromString(msg_.content);
                routingmap_cb_(routingmap_);
            }
        }
    }

    gnss_fs.close();
    veh_fs.close();
    imu_fs.close();
    msf_fs.close();
    dr_fs.close();
    vf_fs.close();
    tcmsf_fs.close();

    return 0;
}

int main(int argc, char **argv) {
    if (argc < 4 || argc > 5) {
        std::cout << "usage: PARSER record_dir output_dir lever_config_json [replay]\n";
        return -1;
    }
    PARSER parser;
    std::string replay = (argc == 5) ? argv[4] : "";
    parser.run(argv[1], argv[2], argv[3], replay);
    return 0;
}