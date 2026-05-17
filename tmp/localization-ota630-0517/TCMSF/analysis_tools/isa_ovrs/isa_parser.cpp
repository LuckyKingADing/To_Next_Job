#include "Eigen/Dense"
#include "fmt/format.h"
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "isa_parser.h"

#include "local_trans.h"

#include "rigid_transform.h"

#define GPS_TPC "/drivers/gnss/raw"
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
using byd::msg::drivers::VehInfo;
using byd::msg::localization::LocalizationEstimate;
using byd::msg::localization::LocalizationMapEngineMessage;
using byd::msg::orin::routing_map::RoutingMap;

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
    fs::path tcmsf_filepath(output_dir / "tcmsf.csv");
    fs::path vf_filepath(output_dir / "vf.csv");
    fs::path routingmap_filepath(output_dir / "routingmap.csv");
    fs::path localmap_dir(output_dir / "localmap");
    if (!fs::exists(localmap_dir)) {
        if (!fs::create_directories(localmap_dir)) {
            return -1;
        }
    }

    std::fstream gnss_fs(gnss_filepath, std::ios::out);
    std::fstream veh_fs(vehicle_filepath, std::ios::out);
    std::fstream imu_fs(imu_filepath, std::ios::out);
    std::fstream msf_fs(msf_filepath, std::ios::out);
    std::fstream dr_fs(dr_filepath, std::ios::out);
    std::fstream tcmsf_fs(tcmsf_filepath, std::ios::out);
    std::fstream vf_fs(vf_filepath, std::ios::out);
    std::fstream routingmap_fs(routingmap_filepath, std::ios::out);

    if (!gnss_fs || !veh_fs || !imu_fs || !msf_fs || !dr_fs || !tcmsf_fs || !vf_fs || !routingmap_fs) {
        std::cout << "Create file failed!" << std::endl;
    }

    constexpr double rad2deg = 180.0 / M_PI;
    constexpr double deg2rad = M_PI / 180.0;

    const static uint64_t TCMSF_QUEUE_MAX_SIZE = 200;
    std::deque<Pose>      tcmsf_queue;

    std::vector<std::string> records;

    fs::directory_iterator iters(data_path);

    for (auto &iter : iters) {
        records.push_back(iter.path());
    }

    std::sort(records.begin(), records.end());

    std::shared_ptr<Gps>                          gps_        = std::make_shared<Gps>();
    std::shared_ptr<Imu>                          imu_        = std::make_shared<Imu>();
    std::shared_ptr<VehInfo>                      veh_        = std::make_shared<VehInfo>();
    std::shared_ptr<LocalizationEstimate>         dr_         = std::make_shared<LocalizationEstimate>();
    std::shared_ptr<LocResult>                    msf_        = std::make_shared<LocResult>();
    std::shared_ptr<VFResult>                     vf_         = std::make_shared<VFResult>();
    std::shared_ptr<Pose>                         tcmsf_      = std::make_shared<Pose>();
    std::shared_ptr<LocalizationMapEngineMessage> localmap_   = std::make_shared<LocalizationMapEngineMessage>();
    std::shared_ptr<RoutingMap>                   routingmap_ = std::make_shared<RoutingMap>();

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
        "lat_std,"      // 10
        "lon_std,"      // 11
        "height_std,"   // 12
        "vel_e_std,"    // 13
        "vel_n_std,"    // 14
        "vel_u_std,"    // 15
        "heading_std,"  // 16
        "num_sats,"     // 17
        "rtk_status,"   // 18
        "sequence_num," // 19
        "lat_mars,"     // 20
        "lon_mars\n";   // 21

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
        "angle\n";      // 12

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
        "sequence_num\n";                // 20

    std::string routingmap_header_str_ =
        "pub_time,"     // 1
        "measure_time," // 2
        "ld_linkid,"    // 3
        "ld_offset,"    // 4
        "sd_linkid,"    // 5
        "sd_offset\n";  // 6

    gnss_fs << gps_header_str_;
    veh_fs << veh_header_str_;
    imu_fs << imu_header_str_;
    msf_fs << msf_header_str_;
    dr_fs << dr_header_str_;
    tcmsf_fs << tcmsf_header_str_;
    vf_fs << vf_header_str_;
    routingmap_fs << routingmap_header_str_;

    byd::geo::LocalTrans trans;

    auto gnss_cb_ = [&gnss_fs](const std::shared_ptr<Gps> &gps_) {
        double lon_mars, lat_mars;
        wgtochina_lb(0, gps_->position().lon(), gps_->position().lat(), gps_->position().height(), 0, 0, &lon_mars, &lat_mars);
        auto line = fmt::format(
            "{:.4f},{:.4f},{:.8f},{:.8f},{:.4f},"   //
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"   //
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"   //
            "{:.4f},{:d},{:d},{:d},{:.8f},"         //
            "{:.8f}\n",                             //
            gps_->header().publish_timestamp(),     // 1
            gps_->header().measurement_timestamp(), // 2
            gps_->position().lat(),                 // 3
            gps_->position().lon(),                 // 4
            gps_->position().height(),              // 5
            gps_->linear_velocity().x(),            // 6
            gps_->linear_velocity().y(),            // 7
            gps_->linear_velocity().z(),            // 8
            gps_->heading(),                        // 9
            gps_->position_std().lat(),             // 10
            gps_->position_std().lon(),             // 11
            gps_->position_std().height(),          // 12
            gps_->linear_velocity_std().x(),        // 13
            gps_->linear_velocity_std().y(),        // 14
            gps_->linear_velocity_std().z(),        // 15
            gps_->heading_std(),                    // 16
            gps_->num_sats(),                       // 17
            (int)gps_->position_status(),           // 18
            gps_->header().sequence_num(),          // 19
            lat_mars,                               // 20
            lon_mars                                // 21
        );
        gnss_fs << line;
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
                  yaw_rate,                               // 5
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

        auto line = fmt::format(
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"  //
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"  //
            "{:d},{:.4f}\n",                       //
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
            aa.angle()                             // 12
        );
        dr_fs << line;
    };

    auto vf_cb_ = [&vf_fs, &tcmsf_queue, &trans](const std::shared_ptr<VFResult> &vf_) {
        double offset_lat_mars, offset_lon_mars;
        auto   t_cmp_func = [](const Pose &data, double t) -> bool { return data.header().measurement_timestamp() < t; };
        auto   lb         = std::lower_bound(tcmsf_queue.begin(), tcmsf_queue.end(), vf_->header().measurement_timestamp(), t_cmp_func);
        if (lb != tcmsf_queue.end()) {
            double lon_mars, lat_mars;
            wgtochina_lb(0, lb->position().lon(), lb->position().lat(), lb->position().height(), 0, 0, &lon_mars, &lat_mars);
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

    auto tcmsf_cb_ = [&tcmsf_fs, &tcmsf_queue, &trans](const std::shared_ptr<Pose> &tcmsf_) {
        tcmsf_queue.push_back(*tcmsf_);
        while (tcmsf_queue.size() > TCMSF_QUEUE_MAX_SIZE) {
            tcmsf_queue.pop_front();
        }
        double lat_mars_imu                  = 0.0;
        double lon_mars_imu                  = 0.0;
        double lat_mars_antenna              = 0.0;
        double lon_mars_antenna              = 0.0;
        double lat_mars_vehicle              = 0.0;
        double lon_mars_vehicle              = 0.0;
        double lat_mars_imu_with_mapbias     = 0.0;
        double lon_mars_imu_with_mapbias     = 0.0;
        double lat_mars_antenna_with_mapbias = 0.0;
        double lon_mars_antenna_with_mapbias = 0.0;
        double lat_mars_vehicle_with_mapbias = 0.0;
        double lon_mars_vehicle_with_mapbias = 0.0;

        double lat_vehicle_with_mapbias = tcmsf_->position().lat();
        double lon_vehicle_with_mapbias = tcmsf_->position().lon();

        Eigen::Vector3d pos_with_map_bias;
        pos_with_map_bias << tcmsf_->position().lat() * deg2rad, tcmsf_->position().lon() * deg2rad, tcmsf_->position().height();
        Eigen::Vector3d map_bias;
        map_bias.x() = tcmsf_->map_bias().x();
        map_bias.y() = tcmsf_->map_bias().y();
        map_bias.z() = 0.0;

        double map_bias_yaw = tcmsf_->map_bias().z() * rad2deg;

        double heading = tcmsf_->heading();

        Eigen::Quaterniond att;
        att.w()                     = tcmsf_->attitude().qw();
        att.x()                     = tcmsf_->attitude().qx();
        att.y()                     = tcmsf_->attitude().qy();
        att.z()                     = tcmsf_->attitude().qz();
        Eigen::Vector3d pos         = trans.DPos_Ego2LLA(pos_with_map_bias, -map_bias, att);
        double          lat_vehicle = pos.x() * rad2deg;
        double          lon_vehicle = pos.y() * rad2deg;

        Eigen::Vector3d lever_antenna_(0.1779969, 0.0001245, 1.456017);
        Eigen::Vector3d pos_antenna_ = trans.DPos_Ego2LLA(pos, lever_antenna_, att);

        double lat_antenna = pos_antenna_.x() * rad2deg;
        double lon_antenna = pos_antenna_.y() * rad2deg;

        wgtochina_lb(0, lon_vehicle, lat_vehicle, pos.z(), 0, 0, &lon_mars_vehicle, &lat_mars_vehicle);
        wgtochina_lb(0, lon_antenna, lat_antenna, pos.z(), 0, 0, &lon_mars_antenna, &lat_mars_antenna);
        wgtochina_lb(0, lon_vehicle_with_mapbias, lat_vehicle_with_mapbias, pos.z(), 0, 0, &lon_mars_vehicle_with_mapbias, &lat_mars_vehicle_with_mapbias);
        auto line = fmt::format(
            "{:.4f},{:.4f},{:.8f},{:.8f},{:.8f},"     //
            "{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},"     //
            "{:.8f},{:.8f},{:.8f},{:.8f},{:d},"       //
            "{:d},{:d},{:.8f},{:.8f},{:d}\n",         //
            tcmsf_->header().publish_timestamp(),     // 1
            tcmsf_->header().measurement_timestamp(), // 2
            lat_mars_imu,                             // 3
            lon_mars_imu,                             // 4
            lat_mars_antenna,                         // 5
            lon_mars_antenna,                         // 6
            lat_mars_vehicle,                         // 7
            lon_mars_vehicle,                         // 8
            lat_mars_imu_with_mapbias,                // 9
            lon_mars_imu_with_mapbias,                // 10
            lat_mars_antenna_with_mapbias,            // 11
            lon_mars_antenna_with_mapbias,            // 12
            lat_mars_vehicle_with_mapbias,            // 13
            lon_mars_vehicle_with_mapbias,            // 14
            (int)tcmsf_->gnss_status(),               // 15
            (int)tcmsf_->fusion_status(),             // 16
            (int)tcmsf_->align_status(),              // 17
            map_bias_yaw,                             // 18
            heading,                                  // 19
            tcmsf_->header().sequence_num()           // 20
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
    if (argc != 3) {
        std::cout << "usage: PARSER record_dir output_dir\n";
        return -1;
    }
    PARSER parser;
    parser.run(argv[1], argv[2]);
    return 0;
}