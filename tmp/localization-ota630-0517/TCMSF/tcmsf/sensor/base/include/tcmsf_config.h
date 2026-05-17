#pragma once

#include "Eigen/Dense"
#include "cyber/common/log.h"
#include "nlohmann/json.hpp"
#include "toml++/toml.h"
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace byd {
namespace tcmsf {
namespace config {

/* parameters for TCMSF
 * Unless otherwise specified,
 * the International System of Units are adopted.
 * Such as: meter, second, radian ...
 */
class Parameters {

public:
    static Parameters &getInstance(const std::string para_file_) {
        static Parameters inst(para_file_);
        return inst;
    }

    Parameters(const Parameters &)            = delete;
    Parameters &operator=(const Parameters &) = delete;

private:
    std::mutex lever_mutex;

public:
    inline bool update_vehicle_lever(const std::string vehicle_para_file_) {

        enum CFG_TYPE { INVALID = 0,
                        JSON    = 1,
                        TOML    = 2,
                        OTHER   = 3
        };
        CFG_TYPE cfg_type{INVALID};
        {
            // 检查后缀名，看看使用哪一种配置文件解析
            size_t pos = vehicle_para_file_.find_last_of('.'); // 查找最后一个点号的位置

            if (pos != std::string::npos) {                                 // 如果找到点号
                std::string extension = vehicle_para_file_.substr(pos + 1); // 提取后缀名
                if (extension == "json") {
                    cfg_type = JSON;
                } else if (extension == "toml") {
                    cfg_type = TOML;
                } else {
                    cfg_type = OTHER;
                }
            } else {
                cfg_type = INVALID;
            }
        }

        std::scoped_lock lever_lock(lever_mutex);
        AINFO << "use additional vehicle info from cfg file: " << vehicle_para_file_;

        switch (cfg_type) {
            case INVALID: {
                AERROR << "file's extension is not 'json' or 'toml'";
                return false;
            } break;

            case JSON:
                try {
                    using json = nlohmann::json;
                    std::ifstream veh_cfg_f(vehicle_para_file_);
                    if (!veh_cfg_f.is_open()) {
                        return false;
                    }
                    json cfg = json::parse(veh_cfg_f);

                    // 这里有个注意点，vehicle仓库，杆臂配置文件里面的这两个杆臂向量的方向与键名不一致，正好相反
                    // 杆臂轴向FLU定义
                    std::vector<double> veh2ante;
                    std::vector<double> veh2imu;
                    cfg.at("ante2RearAxle").get_to(veh2ante);
                    cfg.at("imu2RearAxle").get_to(veh2imu);
                    if (veh2ante.size() != 3 || veh2imu.size() != 3) {
                        return false;
                    }
                    // 这个json格式的配置文件不带车型信息，这里就把车型信息清空吧
                    vehicle_info      = "";
                    lever_imu2vehicle = Eigen::Vector3d{
                        veh2imu.at(1),  //
                        -veh2imu.at(0), //
                        -veh2imu.at(2)  //
                    };
                    lever_vehicle2gnss = Eigen::Vector3d{
                        -veh2ante.at(1), //
                        veh2ante.at(0),  //
                        veh2ante.at(2)   //
                    };
                    lever_imu2gnss = lever_imu2vehicle + lever_vehicle2gnss;
                    AINFO << "vehicle_info: " << vehicle_info;
                    AINFO << "lever_imu2vehicle: " << lever_imu2vehicle.transpose();
                    AINFO << "lever_imu2gnss: " << lever_imu2gnss.transpose();
                    AINFO << "lever_vehicle2gnss: " << lever_vehicle2gnss.transpose();
                    return true;
                } catch (...) {
                    AERROR << "parse additional vehicle info config file failed: " << vehicle_para_file_;
                    return false;
                }
                break;

            case TOML:
                try {
                    toml::v3::ex::parse_result veh_info_para = toml::parse_file(vehicle_para_file_);

                    vehicle_info      = veh_info_para["basic_info"]["vehicle_info"].value_or("");
                    lever_imu2vehicle = Eigen::Vector3d{
                        veh_info_para["lever_FLU"]["lever_flu_vehicle_2_imu"][1].value_or(0.0) / 1000.0,  //
                        -veh_info_para["lever_FLU"]["lever_flu_vehicle_2_imu"][0].value_or(0.0) / 1000.0, //
                        -veh_info_para["lever_FLU"]["lever_flu_vehicle_2_imu"][2].value_or(0.0) / 1000.0  //
                    };
                    lever_vehicle2gnss = Eigen::Vector3d{
                        -veh_info_para["lever_FLU"]["lever_flu_vehicle_2_gnss_antenna"][1].value_or(0.0) / 1000.0, //
                        veh_info_para["lever_FLU"]["lever_flu_vehicle_2_gnss_antenna"][0].value_or(0.0) / 1000.0,  //
                        veh_info_para["lever_FLU"]["lever_flu_vehicle_2_gnss_antenna"][2].value_or(0.0) / 1000.0   //
                    };
                    lever_imu2gnss = lever_imu2vehicle + lever_vehicle2gnss;
                    AINFO << "vehicle_info: " << vehicle_info;
                    AINFO << "lever_imu2vehicle: " << lever_imu2vehicle.transpose();
                    AINFO << "lever_imu2gnss: " << lever_imu2gnss.transpose();
                    AINFO << "lever_vehicle2gnss: " << lever_vehicle2gnss.transpose();
                    return true;
                } catch (...) {
                    AERROR << "parse additional vehicle info config file failed: " << vehicle_para_file_;
                    return false;
                }
                break;

            default:
                break;
        }
        return false;
    }

public:
    enum GNSS_STATUS {
        INVALID = 0,
        NONE    = 1,
        SINGLE  = 2,
        DGPS    = 3,
        PPP     = 4,
        FLOAT   = 5,
        FIX     = 6
    };
    enum OUTPUT_REFERENCE_FRAME {
        ENU_NAVI_FRAME = 0,
        MAP_FRAME      = 1,
    };
    enum OUTPUT_POSITION_CENTER {
        GNSS_ANTENNA             = 0,
        VEHICLE_REAR_AXLE_CENTER = 1,
    };

    enum GnssFusionMode {
        GNSS_LOOSE_COUPLE = 0, // without RTK
        GNSS_TIGHT_COUPLE = 1, // without RTK
        RTK_LOOSE_COUPLE  = 2,
        RTK_TIGHT_COUPLE  = 3,
    };

private:
    ~Parameters() {
        AINFO << "exit Parameters";
    }

private:
    bool is_parse_success = false;

    std::string                para_file{};
    toml::v3::ex::parse_result para;

private:
    // [basic_info]
    std::string vehicle_info;

    // [initialization]
    Eigen::Matrix<double, 21, 1> init_P;

    // [imu]
    double gyro_ND;
    double gyro_RW;
    double acc_ND;
    double acc_RW;
    double imu_data_refresh_dt;

    // [lever] # R-F-U
    Eigen::Vector3d lever_imu2vehicle;
    Eigen::Vector3d lever_imu2gnss;
    Eigen::Vector3d lever_vehicle2gnss;

    // [misalignment] # pitch - roll - yaw | 内旋 | frame transform
    Eigen::Vector3d    misalignment_imu2vehicle_euler_angle; // rad
    Eigen::Quaterniond misalignment_imu2vehicle_quaternion;
    Eigen::Matrix3d    misalignment_imu2vehicle_matrix;

    // [wheel]
    uint64_t wheel_msg_skip;
    double   wheel_speed_bias;     // ratio
    double   wheel_speed_bias_std; // ratio
    double   wheel_data_refresh_dt;

    double wheel_yaw_rw;
    double wheel_pitch_rw;
    double wheel_spd_scale_rw;

    double wheel_scale_adapter_00mps;
    double wheel_scale_adapter_10mps;
    double wheel_scale_adapter_20mps;
    double wheel_scale_adapter_30mps;
    double wheel_scale_adapter_40mps;

    double slip_index_rejection_bound;
    double slow_speed_bound;

    // [gnss]
    bool                          gnss_override_raw_std;
    std::map<GNSS_STATUS, double> gnss_position_bias_std; // m
    std::map<GNSS_STATUS, double> gnss_velocity_bias_std; // m/s
    double                        gnss_data_refresh_dt;

    bool   enable_ego_position_compensation;
    double ego_pos_inno_lat_diff_bound;
    double ego_pos_inno_lat_std_bound;
    double ego_pos_inno_lon_bound;
    double ego_pos_inno_lon_mean_bound;
    double ego_pos_inno_lon_std_bound;

    double ego_pos_inno_hgt_bound;
    double ego_pos_inno_hgt_mean_bound;
    double ego_pos_inno_hgt_std_bound;

    // [vision_fusion]
    double vision_pos_lat_bias_std;
    double vision_pos_lon_bias_std;
    double vision_heading_bias_std;
    double vision_data_refresh_dt;
    double map_pos_east_bias_rw;
    double map_pos_north_bias_rw;
    double map_heading_bias_rw;

    // [smooth]
    bool   enable_position_smooth_lat;
    double position_smooth_lat_update_bound;
    double position_max_delta_lat_feedback;

    // [persistence]
    bool        is_persistence_enable = false;
    std::string vehicle_info_file_path;

    // [time_compesation]
    bool enable_gnss_time_compesation;
    bool enable_wheel_time_compesation;
    bool enable_vision_time_compesation;

    // [constrain]
    Eigen::Vector3d              constrain_euler_angle_imu2navi;
    Eigen::Vector3d              constrain_velocity_navi2earth;
    Eigen::Vector3d              constrain_gyro_bias;
    Eigen::Vector3d              constrain_acc_bias;
    double                       constrain_wheel_speed_bias;
    Eigen::Vector3d              constrain_euler_angle_imu2vehicle;
    Eigen::Matrix<double, 21, 1> constrain_P_std_min;
    Eigen::Matrix<double, 21, 1> constrain_P_std_max;
    Eigen::Vector3d              constrain_map_bias;
    double                       ego_lat_velocity_constrain; // m/s

    // [measurement]
    double                        wheel_velocity_additional_std_scale;
    std::map<GNSS_STATUS, double> gnss_position_additional_std_scale;
    std::map<GNSS_STATUS, double> gnss_velocity_additional_std_scale;
    std::map<GNSS_STATUS, double> gnss_heading_additional_std_scale;
    double                        vision_pos_lat_additional_std_scale;
    double                        vision_pos_lon_additional_std_scale;
    double                        vision_heading_additional_std_scale;

    double maneuver_status_steady_acc_bound         = 0.2;
    double maneuver_status_low_dynamic_acc_bound    = 0.5;
    double maneuver_status_steady_rotate_bound      = 1.0;
    double maneuver_status_low_dynamic_rotate_bound = 2.0;

    // [IGG3] IGG3 抗差 Typical values k0=1.0~1.5，k1=2.0~3.0
    double gnss_pos_igg3_k0  = 1.0;
    double gnss_pos_igg3_k1  = 3.0;
    double gnss_vel_igg3_k0  = 1.0;
    double gnss_vel_igg3_k1  = 2.0;
    double gnss_hdg_igg3_k0  = 0.5;
    double gnss_hdg_igg3_k1  = 1.0;
    double wheel_spd_igg3_k0 = 1.0;
    double wheel_spd_igg3_k1 = 2.0;
    double zupt_igg3_k0      = 1.0;
    double zupt_igg3_k1      = 2.0;
    double vision_pos_lat_igg3_k0;
    double vision_pos_lat_igg3_k1;
    double vision_pos_lon_igg3_k0;
    double vision_pos_lon_igg3_k1;
    double vision_hdg_igg3_k0;
    double vision_hdg_igg3_k1;

    // [stablization]
    bool attitude_stablization_by_gravity;

    // [dead_reckoning]
    bool use_internal_dr_info;

    // [fusion]
    bool enable_gnss_fusion;
    bool enable_gnss_pos_fusion;
    bool enable_gnss_vel_fusion;
    bool enable_gnss_heading_fusion;
    bool enable_wheel_vel_fusion;
    bool enable_vision_fusion;
    bool enable_vision_lat_fusion;
    bool enable_vision_lon_fusion;
    bool enable_vision_hdg_fusion;
    bool enable_map_fusion;
    bool enable_db_fusion;

    // [align]
    double gnss_minimum_speed_required_for_initialization;

    double gnss_maximum_position_innovation_for_fine_align;
    double gnss_maximum_speed_innovation_for_fine_align;
    double gnss_maximum_heading_innovation_for_fine_align;
    double gnss_maximum_position_innovation_for_fine_align_std;
    double gnss_maximum_speed_innovation_for_fine_align_std;
    double gnss_maximum_heading_innovation_for_fine_align_std;

    double gnss_maximum_position_innovation_for_coarse_align;
    double gnss_maximum_speed_innovation_for_coarse_align;
    double gnss_maximum_heading_innovation_for_coarse_align;
    double gnss_maximum_position_innovation_for_coarse_align_std;
    double gnss_maximum_speed_innovation_for_coarse_align_std;
    double gnss_maximum_heading_innovation_for_coarse_align_std;

    double gnss_minimum_position_innovation_for_exit_aligned;
    double gnss_minimum_speed_innovation_for_exit_aligned;
    double gnss_minimum_heading_innovation_for_exit_aligned;
    double gnss_minimum_position_innovation_for_exit_aligned_std;
    double gnss_minimum_speed_innovation_for_exit_aligned_std;
    double gnss_minimum_heading_innovation_for_exit_aligned_std;

    double gnss_minimum_position_innovation_for_exit_fine_align;
    double gnss_minimum_speed_innovation_for_exit_fine_align;
    double gnss_minimum_heading_innovation_for_exit_fine_align;
    double gnss_minimum_position_innovation_for_exit_fine_align_std;
    double gnss_minimum_speed_innovation_for_exit_fine_align_std;
    double gnss_minimum_heading_innovation_for_exit_fine_align_std;

    double gnss_minimum_position_innovation_for_initialization;
    double gnss_minimum_speed_innovation_for_initialization;
    double gnss_minimum_heading_innovation_for_initialization;
    double gnss_minimum_position_innovation_for_initialization_std;
    double gnss_minimum_speed_innovation_for_initialization_std;
    double gnss_minimum_heading_innovation_for_initialization_std;

    // [diagnosis]
    bool   sensor_delay_diagnosis_enable;
    double sensor_delay_valid_bound;
    double sensor_delay_filter_bound_gnss;
    double sensor_delay_filter_bound_vehicle;
    double sensor_delay_filter_bound_vision;

    // [output]
    OUTPUT_POSITION_CENTER output_position_center;
    OUTPUT_REFERENCE_FRAME output_reference_frame;

    // [vehicle_info_override]
    bool        vehicle_info_override_enable;
    std::string vehicle_info_override_file_path;

    // [motion_status]
    int    buffer_size;
    double max_steer_angle_for_turning;
    double min_steer_angle_for_straight;
    double max_gyro_z_for_turning;
    double min_gyro_z_for_straight;
    double min_speed_for_static;
    double min_acc_x_uniform;
    double percent_for_valid;
    int    observation_window;

    // [fusion_mode]
    GnssFusionMode gnss_fusion_mode{RTK_LOOSE_COUPLE};
    bool           gnss_fusion_mode_adaption;

    // [sd_map]
    bool   enable_sd_map_bias_comp;
    double sd_map_max_bias_propor_dist;
    double sd_map_bias_bound;
    double sd_map_min_thres_trig_corr;
    double sd_map_bias_fading_propor_dist;

public:
    inline std::string                  get_vehicle_info() { return vehicle_info; }
    inline Eigen::Matrix<double, 21, 1> get_init_P() {
        return init_P;
    }
    inline double          get_gyro_RW() { return gyro_RW; };
    inline double          get_gyro_ND() { return gyro_ND; };
    inline double          get_acc_RW() { return acc_RW; };
    inline double          get_acc_ND() { return acc_ND; };
    inline double          get_imu_data_refresh_dt() { return imu_data_refresh_dt; };
    inline Eigen::Vector3d get_lever_imu2vehicle() {
        std::scoped_lock lever_lock(lever_mutex);
        return lever_imu2vehicle;
    };
    inline Eigen::Vector3d get_lever_imu2gnss() {
        std::scoped_lock lever_lock(lever_mutex);
        return lever_imu2gnss;
    };
    inline Eigen::Vector3d get_lever_vehicle2gnss() {
        std::scoped_lock lever_lock(lever_mutex);
        return lever_vehicle2gnss;
    };
    inline Eigen::Vector3d    get_misalignment_imu2vehicle_euler_angle() { return misalignment_imu2vehicle_euler_angle; };
    inline Eigen::Quaterniond get_misalignment_imu2vehicle_quaternion() { return misalignment_imu2vehicle_quaternion; };
    inline Eigen::Matrix3d    get_misalignment_imu2vehicle_matrix() { return misalignment_imu2vehicle_matrix; };
    inline uint64_t           get_wheel_msg_skip() { return wheel_msg_skip; }
    inline double             get_wheel_speed_bias() { return wheel_speed_bias; };
    inline double             get_wheel_speed_bias_std() { return wheel_speed_bias_std; };
    inline double             get_wheel_data_refresh_dt() { return wheel_data_refresh_dt; };

    inline double get_wheel_yaw_rw() { return wheel_yaw_rw; };
    inline double get_wheel_pitch_rw() { return wheel_pitch_rw; };
    inline double get_wheel_spd_scale_rw() { return wheel_spd_scale_rw; };

    inline double get_wheel_scale_adapter_00mps() {
        return wheel_scale_adapter_00mps;
    }
    inline double get_wheel_scale_adapter_10mps() {
        return wheel_scale_adapter_10mps;
    }
    inline double get_wheel_scale_adapter_20mps() {
        return wheel_scale_adapter_20mps;
    }
    inline double get_wheel_scale_adapter_30mps() {
        return wheel_scale_adapter_30mps;
    }
    inline double get_wheel_scale_adapter_40mps() {
        return wheel_scale_adapter_40mps;
    }

    inline double get_slip_index_rejection_bound() {
        return slip_index_rejection_bound;
    }

    inline double get_slow_speed_bound() {
        return slow_speed_bound;
    }

    inline double get_gnss_override_raw_std() { return this->gnss_override_raw_std; };
    inline double get_gnss_position_bias_std(GNSS_STATUS gnss_status) {
        try {
            return this->gnss_position_bias_std[gnss_status];
        } catch (...) {
            AINFO << "GNSS Status " << gnss_status << " has no corresponding STD.";
            return 1.0e10;
        }
    };
    inline double get_gnss_velocity_bias_std(GNSS_STATUS gnss_status) {
        try {
            return this->gnss_velocity_bias_std[gnss_status];
        } catch (...) {
            AINFO << "GNSS Status " << gnss_status << " has no corresponding STD.";
            return 1.0e10;
        }
    };
    inline double get_gnss_data_refresh_dt() { return gnss_data_refresh_dt; };

    inline bool get_enable_ego_position_compensation() {
        return enable_ego_position_compensation;
    }
    inline double get_ego_pos_inno_lat_diff_bound() {
        return ego_pos_inno_lat_diff_bound;
    }
    inline double get_ego_pos_inno_lat_std_bound() {
        return ego_pos_inno_lat_std_bound;
    }
    inline double get_ego_pos_inno_lon_bound() {
        return ego_pos_inno_lon_bound;
    }
    inline double get_ego_pos_inno_lon_mean_bound() {
        return ego_pos_inno_lon_mean_bound;
    }
    inline double get_ego_pos_inno_lon_std_bound() {
        return ego_pos_inno_lon_std_bound;
    }
    inline double get_ego_pos_inno_hgt_bound() {
        return ego_pos_inno_hgt_bound;
    }
    inline double get_ego_pos_inno_hgt_mean_bound() {
        return ego_pos_inno_hgt_mean_bound;
    }
    inline double get_ego_pos_inno_hgt_std_bound() {
        return ego_pos_inno_hgt_std_bound;
    }

    inline double get_vision_pos_lat_bias_std() {
        return vision_pos_lat_bias_std;
    };
    inline double get_vision_pos_lon_bias_std() {
        return vision_pos_lon_bias_std;
    };
    inline double get_vision_heading_bias_std() {
        return vision_heading_bias_std;
    };
    inline double get_vision_data_refresh_dt() {
        return vision_data_refresh_dt;
    };
    inline double get_map_pos_east_bias_rw() {
        return map_pos_east_bias_rw;
    };
    inline double get_map_pos_north_bias_rw() {
        return map_pos_north_bias_rw;
    };
    inline double get_map_heading_bias_rw() {
        return map_heading_bias_rw;
    };

    inline double get_position_smooth_lat_update_bound() {
        return position_smooth_lat_update_bound;
    }
    inline double get_position_max_delta_lat_feedback() {
        return position_max_delta_lat_feedback;
    }
    inline bool get_enable_position_smooth_lat() {
        return enable_position_smooth_lat;
    }

    inline bool get_is_persistence_enable() {
        return is_persistence_enable;
    }

    inline std::string get_vehicle_info_file_path() {
        return vehicle_info_file_path;
    }

    inline bool get_enable_gnss_time_compesation() {
        return enable_gnss_time_compesation;
    }
    inline bool get_enable_wheel_time_compesation() {
        return enable_wheel_time_compesation;
    }
    inline bool get_enable_vision_time_compesation() {
        return enable_vision_time_compesation;
    }

    inline Eigen::Vector3d              get_constrain_euler_angle_imu2navi() { return constrain_euler_angle_imu2navi; };
    inline Eigen::Vector3d              get_constrain_velocity_navi2earth() { return constrain_velocity_navi2earth; };
    inline Eigen::Vector3d              get_constrain_gyro_bias() { return constrain_gyro_bias; };
    inline Eigen::Vector3d              get_constrain_acc_bias() { return constrain_acc_bias; };
    inline double                       get_constrain_wheel_speed_bias() { return constrain_wheel_speed_bias; };
    inline Eigen::Vector3d              get_constrain_euler_angle_imu2vehicle() { return constrain_euler_angle_imu2vehicle; };
    inline Eigen::Matrix<double, 21, 1> get_constrain_P_std_min() { return constrain_P_std_min; };
    inline Eigen::Matrix<double, 21, 1> get_constrain_P_std_max() { return constrain_P_std_max; };

    inline Eigen::Vector3d get_constrain_map_bias() {
        return constrain_map_bias;
    }

    inline double get_ego_lat_velocity_constrain() {
        return ego_lat_velocity_constrain;
    }

    inline double get_wheel_velocity_additional_std_scale() { return wheel_velocity_additional_std_scale; };
    inline double get_gnss_position_additional_std_scale(GNSS_STATUS gnss_status) {
        try {
            return this->gnss_position_additional_std_scale[gnss_status];
        } catch (...) {
            AINFO << "GNSS Status " << gnss_status << " has no corresponding position STD scale.";
            return 1.0e10;
        }
    };
    inline double get_gnss_velocity_additional_std_scale(GNSS_STATUS gnss_status) {
        try {
            return this->gnss_velocity_additional_std_scale[gnss_status];
        } catch (...) {
            AINFO << "GNSS Status " << gnss_status << " has no corresponding velocity STD scale.";
            return 1.0e10;
        }
    };
    inline double get_gnss_heading_additional_std_scale(GNSS_STATUS gnss_status) {
        try {
            return this->gnss_heading_additional_std_scale[gnss_status];
        } catch (...) {
            AINFO << "GNSS Status " << gnss_status << " has no corresponding heading STD scale.";
            return 1.0e10;
        }
    };

    inline double get_vision_pos_lat_additional_std_scale() { return vision_pos_lat_additional_std_scale; }
    inline double get_vision_pos_lon_additional_std_scale() { return vision_pos_lon_additional_std_scale; }
    inline double get_vision_heading_additional_std_scale() { return vision_heading_additional_std_scale; }

    inline double get_maneuver_status_steady_acc_bound() {
        return maneuver_status_steady_acc_bound;
    }
    inline double get_maneuver_status_low_dynamic_acc_bound() {
        return maneuver_status_low_dynamic_acc_bound;
    }
    inline double get_maneuver_status_steady_rotate_bound() {
        return maneuver_status_steady_rotate_bound;
    }
    inline double get_maneuver_status_low_dynamic_rotate_bound() {
        return maneuver_status_low_dynamic_rotate_bound;
    }

    inline double get_gnss_pos_igg3_k0() { return gnss_pos_igg3_k0; };
    inline double get_gnss_pos_igg3_k1() { return gnss_pos_igg3_k1; };
    inline double get_gnss_vel_igg3_k0() { return gnss_vel_igg3_k0; };
    inline double get_gnss_vel_igg3_k1() { return gnss_vel_igg3_k1; };
    inline double get_gnss_hdg_igg3_k0() { return gnss_hdg_igg3_k0; };
    inline double get_gnss_hdg_igg3_k1() { return gnss_hdg_igg3_k1; };
    inline double get_wheel_spd_igg3_k0() { return wheel_spd_igg3_k0; };
    inline double get_wheel_spd_igg3_k1() { return wheel_spd_igg3_k1; };
    inline double get_vision_pos_lat_igg3_k0() { return vision_pos_lat_igg3_k0; }
    inline double get_vision_pos_lat_igg3_k1() { return vision_pos_lat_igg3_k1; }
    inline double get_vision_pos_lon_igg3_k0() { return vision_pos_lon_igg3_k0; }
    inline double get_vision_pos_lon_igg3_k1() { return vision_pos_lon_igg3_k1; }
    inline double get_vision_hdg_igg3_k0() { return vision_hdg_igg3_k0; }
    inline double get_vision_hdg_igg3_k1() { return vision_hdg_igg3_k1; }
    inline double get_zupt_igg3_k0() { return zupt_igg3_k0; };
    inline double get_zupt_igg3_k1() { return zupt_igg3_k1; };
    inline bool   get_attitude_stablization_by_gravity() { return attitude_stablization_by_gravity; };
    inline bool   get_use_internal_dr_info() { return use_internal_dr_info; };
    inline bool   get_enable_gnss_fusion() { return enable_gnss_fusion; };
    inline bool   get_enable_gnss_pos_fusion() { return enable_gnss_pos_fusion; };
    inline bool   get_enable_gnss_vel_fusion() { return enable_gnss_vel_fusion; };
    inline bool   get_enable_wheel_vel_fusion() { return enable_wheel_vel_fusion; };
    inline bool   get_enable_vision_fusion() { return enable_vision_fusion; };
    inline bool   get_enable_vision_lat_fusion() { return enable_vision_lat_fusion; };
    inline bool   get_enable_vision_lon_fusion() { return enable_vision_lon_fusion; };
    inline bool   get_enable_vision_hdg_fusion() { return enable_vision_hdg_fusion; };
    inline bool   get_enable_gnss_heading_fusion() { return enable_gnss_heading_fusion; };
    inline bool   get_enable_map_fusion() { return enable_map_fusion; };
    inline bool   get_enable_db_fusion() { return enable_db_fusion; };

    inline double get_gnss_minimum_speed_required_for_initialization() {
        return gnss_minimum_speed_required_for_initialization;
    }

    inline double get_gnss_maximum_position_innovation_for_fine_align() {
        return gnss_maximum_position_innovation_for_fine_align;
    }
    inline double get_gnss_maximum_speed_innovation_for_fine_align() {
        return gnss_maximum_speed_innovation_for_fine_align;
    }
    inline double get_gnss_maximum_heading_innovation_for_fine_align() {
        return gnss_maximum_heading_innovation_for_fine_align;
    }
    inline double get_gnss_maximum_position_innovation_for_fine_align_std() {
        return gnss_maximum_position_innovation_for_fine_align_std;
    }
    inline double get_gnss_maximum_speed_innovation_for_fine_align_std() {
        return gnss_maximum_speed_innovation_for_fine_align_std;
    }
    inline double get_gnss_maximum_heading_innovation_for_fine_align_std() {
        return gnss_maximum_heading_innovation_for_fine_align_std;
    }

    inline double get_gnss_maximum_position_innovation_for_coarse_align() {
        return gnss_maximum_position_innovation_for_coarse_align;
    }
    inline double get_gnss_maximum_speed_innovation_for_coarse_align() {
        return gnss_maximum_speed_innovation_for_coarse_align;
    }
    inline double get_gnss_maximum_heading_innovation_for_coarse_align() {
        return gnss_maximum_heading_innovation_for_coarse_align;
    }
    inline double get_gnss_maximum_position_innovation_for_coarse_align_std() {
        return gnss_maximum_position_innovation_for_coarse_align_std;
    }
    inline double get_gnss_maximum_speed_innovation_for_coarse_align_std() {
        return gnss_maximum_speed_innovation_for_coarse_align_std;
    }
    inline double get_gnss_maximum_heading_innovation_for_coarse_align_std() {
        return gnss_maximum_heading_innovation_for_coarse_align_std;
    }

    inline double get_gnss_minimum_position_innovation_for_exit_aligned() {
        return gnss_minimum_position_innovation_for_exit_aligned;
    }
    inline double get_gnss_minimum_speed_innovation_for_exit_aligned() {
        return gnss_minimum_speed_innovation_for_exit_aligned;
    }
    inline double get_gnss_minimum_heading_innovation_for_exit_aligned() {
        return gnss_minimum_heading_innovation_for_exit_aligned;
    }
    inline double get_gnss_minimum_position_innovation_for_exit_aligned_std() {
        return gnss_minimum_position_innovation_for_exit_aligned_std;
    }
    inline double get_gnss_minimum_speed_innovation_for_exit_aligned_std() {
        return gnss_minimum_speed_innovation_for_exit_aligned_std;
    }
    inline double get_gnss_minimum_heading_innovation_for_exit_aligned_std() {
        return gnss_minimum_heading_innovation_for_exit_aligned_std;
    }

    inline double get_gnss_minimum_position_innovation_for_exit_fine_align() {
        return gnss_minimum_position_innovation_for_exit_fine_align;
    }
    inline double get_gnss_minimum_speed_innovation_for_exit_fine_align() {
        return gnss_minimum_speed_innovation_for_exit_fine_align;
    }
    inline double get_gnss_minimum_heading_innovation_for_exit_fine_align() {
        return gnss_minimum_heading_innovation_for_exit_fine_align;
    }
    inline double get_gnss_minimum_position_innovation_for_exit_fine_align_std() {
        return gnss_minimum_position_innovation_for_exit_fine_align_std;
    }
    inline double get_gnss_minimum_speed_innovation_for_exit_fine_align_std() {
        return gnss_minimum_speed_innovation_for_exit_fine_align_std;
    }
    inline double get_gnss_minimum_heading_innovation_for_exit_fine_align_std() {
        return gnss_minimum_heading_innovation_for_exit_fine_align_std;
    }

    inline double get_gnss_minimum_position_innovation_for_initialization() {
        return gnss_minimum_position_innovation_for_initialization;
    }
    inline double get_gnss_minimum_speed_innovation_for_initialization() {
        return gnss_minimum_speed_innovation_for_initialization;
    }
    inline double get_gnss_minimum_heading_innovation_for_initialization() {
        return gnss_minimum_heading_innovation_for_initialization;
    }
    inline double get_gnss_minimum_position_innovation_for_initialization_std() {
        return gnss_minimum_position_innovation_for_initialization_std;
    }
    inline double get_gnss_minimum_speed_innovation_for_initialization_std() {
        return gnss_minimum_speed_innovation_for_initialization_std;
    }
    inline double get_gnss_minimum_heading_innovation_for_initialization_std() {
        return gnss_minimum_heading_innovation_for_initialization_std;
    }

    inline OUTPUT_POSITION_CENTER get_output_position_center() {
        return output_position_center;
    };
    inline OUTPUT_REFERENCE_FRAME get_output_reference_frame() {
        return output_reference_frame;
    };

    inline bool get_sensor_delay_diagnosis_enable() {
        return sensor_delay_diagnosis_enable;
    }
    inline double get_sensor_delay_valid_bound() {
        return sensor_delay_valid_bound;
    }
    inline double get_sensor_delay_filter_bound_gnss() {
        return sensor_delay_filter_bound_gnss;
    }
    inline double get_sensor_delay_filter_bound_vehicle() {
        return sensor_delay_filter_bound_vehicle;
    }
    inline double get_sensor_delay_filter_bound_vision() {
        return sensor_delay_filter_bound_vision;
    }

    inline double get_buffer_size() {
        return buffer_size;
    };
    inline double get_max_steer_angle_for_turning() {
        return max_steer_angle_for_turning;
    };
    inline double get_min_steer_angle_for_straight() {
        return min_steer_angle_for_straight;
    };
    inline double get_max_gyro_z_for_turning() {
        return max_gyro_z_for_turning;
    };
    inline double get_min_gyro_z_for_straight() {
        return min_gyro_z_for_straight;
    };
    inline double get_min_speed_for_static() {
        return min_speed_for_static;
    };
    inline double get_min_acc_x_uniform() {
        return min_acc_x_uniform;
    };
    inline double get_percent_for_valid() {
        return percent_for_valid;
    };
    inline double get_observation_window() {
        return observation_window;
    };

    inline GnssFusionMode get_gnss_fusion_mode() {
        return gnss_fusion_mode;
    }

    inline void set_gnss_fusion_mode(GnssFusionMode new_gnss_fusion_mode) {
        gnss_fusion_mode = new_gnss_fusion_mode;
    }

    inline bool get_gnss_fusion_mode_adaption() {
        return gnss_fusion_mode_adaption;
    }

    // [sd_map]
    inline bool get_enable_sd_map_bias_comp() {
        return enable_sd_map_bias_comp;
    }
    inline double get_sd_map_max_bias_propor_dist() {
        return sd_map_max_bias_propor_dist;
    }
    inline double get_sd_map_bias_bound() {
        return sd_map_bias_bound;
    }
    inline double get_sd_map_min_thres_trig_corr() {
        return sd_map_min_thres_trig_corr;
    }
    inline double get_sd_map_bias_fading_propor_dist() {
        return sd_map_bias_fading_propor_dist;
    }

private:
    bool parse() {
        para = toml::parse_file(para_file);
        AINFO << "parse initial cfg: " << para_file;
        try {
            // get key-value pairs
            rtcm.dump_to_console = para["RTCM"]["dump_to_console"].value_or<bool>(false);

            vehicle_info = para["basic_info"]["vehicle_info"].value_or("");

            auto init_P_ = para["initialization"]["init_P"];
            init_P(0)    = init_P_[0].value_or(3.0) / 180.0 * M_PI;  // arc   | pitch
            init_P(1)    = init_P_[1].value_or(3.0) / 180.0 * M_PI;  // arc   | roll
            init_P(2)    = init_P_[2].value_or(3.0) / 180.0 * M_PI;  // arc   | yaw
            init_P(3)    = init_P_[3].value_or(1.0);                 // m/s   | east velocity
            init_P(4)    = init_P_[4].value_or(1.0);                 // m/s   | north velocity
            init_P(5)    = init_P_[5].value_or(1.0);                 // m/s   | up velocity
            init_P(6)    = init_P_[6].value_or(1.0);                 // m     | latitude
            init_P(7)    = init_P_[7].value_or(1.0);                 // m     | longitude
            init_P(8)    = init_P_[8].value_or(1.0);                 // m     | height
            init_P(9)    = init_P_[9].value_or(0.1) / 180.0 * M_PI;  // arc/s | x gyro bias
            init_P(10)   = init_P_[10].value_or(0.1) / 180.0 * M_PI; // arc/s | y gyro bias
            init_P(11)   = init_P_[11].value_or(0.1) / 180.0 * M_PI; // arc/s | z gyro bias
            init_P(12)   = init_P_[12].value_or(0.1);                // m/s^2 | x acc bias
            init_P(13)   = init_P_[13].value_or(0.1);                // m/s^2 | y acc bias
            init_P(14)   = init_P_[14].value_or(0.1);                // m/s^2 | z acc bias
            init_P(15)   = init_P_[15].value_or(0.3) / 180.0 * M_PI; // arc   | pitch misalignment error
            init_P(16)   = init_P_[16].value_or(0.01);               // scale | wheel speed error
            init_P(17)   = init_P_[17].value_or(0.3) / 180.0 * M_PI; // arc   | yaw misalignment error
            init_P(18)   = init_P_[18].value_or(0.5);                // m     | position bias such as mapping errors, latitude
            init_P(19)   = init_P_[19].value_or(0.5);                // m     | position bias such as mapping errors, longitude
            init_P(20)   = init_P_[20].value_or(0.5) / 180.0 * M_PI; // arc   | map yaw error

            gyro_ND             = para["imu"]["gyro_ND"].value_or(9.7e-4);
            gyro_RW             = para["imu"]["gyro_RW"].value_or(9.7e-5);
            acc_ND              = para["imu"]["acc_ND"].value_or(6.6e-3);
            acc_RW              = para["imu"]["acc_RW"].value_or(6.6e-4);
            imu_data_refresh_dt = para["imu"]["imu_data_refresh_dt"].value_or(0.01);
            {
                std::scoped_lock lever_lock(lever_mutex);
                lever_imu2vehicle = Eigen::Vector3d{para["lever"]["imu2vehicle"][0].value_or(0.0), para["lever"]["imu2vehicle"][1].value_or(0.0), para["lever"]["imu2vehicle"][2].value_or(0.0)};
                // lever_imu2gnss                       = Eigen::Vector3d{para["lever"]["imu2gnss"][0].value_or(0.0), para["lever"]["imu2gnss"][1].value_or(0.0), para["lever"]["imu2gnss"][2].value_or(0.0)};
                lever_vehicle2gnss = Eigen::Vector3d{para["lever"]["vehicle2gnss"][0].value_or(0.0), para["lever"]["vehicle2gnss"][1].value_or(0.0), para["lever"]["vehicle2gnss"][2].value_or(0.0)};
                lever_imu2gnss     = lever_imu2vehicle + lever_vehicle2gnss;
            }
            Eigen::Vector3d phi_                 = Eigen::Vector3d{para["misalignment"]["euler_angle_imu2vehicle"][0].value_or(0.0), para["misalignment"]["euler_angle_imu2vehicle"][1].value_or(0.0), para["misalignment"]["euler_angle_imu2vehicle"][2].value_or(0.0)};
            misalignment_imu2vehicle_euler_angle = phi_ / 180.0 * M_PI;
            misalignment_imu2vehicle_matrix      = Eigen::AngleAxisd(phi_.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(phi_.y(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(phi_.x(), Eigen::Vector3d::UnitX());
            misalignment_imu2vehicle_quaternion  = misalignment_imu2vehicle_matrix;

            wheel_msg_skip        = para["wheel"]["wheel_msg_skip"].value_or(10.0);
            wheel_speed_bias      = para["wheel"]["wheel_speed_bias"].value_or(0.0);
            wheel_speed_bias_std  = para["wheel"]["wheel_speed_bias_std"].value_or(0.01);
            wheel_data_refresh_dt = para["wheel"]["wheel_data_refresh_dt"].value_or(0.01);

            wheel_yaw_rw       = para["wheel"]["wheel_yaw_rw"].value_or(1e-5) / 180.0 * M_PI;
            wheel_pitch_rw     = para["wheel"]["wheel_pitch_rw"].value_or(1e-5) / 180.0 * M_PI;
            wheel_spd_scale_rw = para["wheel"]["wheel_spd_scale_rw"].value_or(1e-4);

            wheel_scale_adapter_00mps = para["wheel"]["wheel_scale_adapter_00mps"].value_or(0.0e-3);
            wheel_scale_adapter_10mps = para["wheel"]["wheel_scale_adapter_10mps"].value_or(0.5e-3);
            wheel_scale_adapter_20mps = para["wheel"]["wheel_scale_adapter_20mps"].value_or(1.0e-3);
            wheel_scale_adapter_30mps = para["wheel"]["wheel_scale_adapter_30mps"].value_or(2.0e-3);
            wheel_scale_adapter_40mps = para["wheel"]["wheel_scale_adapter_40mps"].value_or(4.0e-3);

            slip_index_rejection_bound = para["wheel"]["slip_index_rejection_bound"].value_or(1.0);

            slow_speed_bound = para["wheel"]["slow_speed_bound"].value_or(5.0);

            gnss_override_raw_std = para["gnss"]["gnss_override_raw_std"].value_or(true);

            auto pos_std_ = para["gnss"]["gnss_position_bias_std"];
            gnss_position_bias_std.clear();
            gnss_position_bias_std.insert(std::make_pair(GNSS_STATUS::FIX, pos_std_[0].value_or(0.3)));
            gnss_position_bias_std.insert(std::make_pair(GNSS_STATUS::FLOAT, pos_std_[1].value_or(3.0)));
            gnss_position_bias_std.insert(std::make_pair(GNSS_STATUS::PPP, pos_std_[2].value_or(3.0)));
            gnss_position_bias_std.insert(std::make_pair(GNSS_STATUS::DGPS, pos_std_[3].value_or(6.0)));
            gnss_position_bias_std.insert(std::make_pair(GNSS_STATUS::SINGLE, pos_std_[4].value_or(15.0)));
            gnss_position_bias_std.insert(std::make_pair(GNSS_STATUS::NONE, pos_std_[5].value_or(1.0e10)));
            gnss_position_bias_std.insert(std::make_pair(GNSS_STATUS::INVALID, pos_std_[6].value_or(1.0e10)));

            auto vel_std_ = para["gnss"]["gnss_velocity_bias_std"];
            gnss_velocity_bias_std.clear();
            gnss_velocity_bias_std.insert(std::make_pair(GNSS_STATUS::FIX, vel_std_[0].value_or(0.1)));
            gnss_velocity_bias_std.insert(std::make_pair(GNSS_STATUS::FLOAT, vel_std_[1].value_or(2.0)));
            gnss_velocity_bias_std.insert(std::make_pair(GNSS_STATUS::PPP, vel_std_[2].value_or(2.0)));
            gnss_velocity_bias_std.insert(std::make_pair(GNSS_STATUS::DGPS, vel_std_[3].value_or(3.0)));
            gnss_velocity_bias_std.insert(std::make_pair(GNSS_STATUS::SINGLE, vel_std_[4].value_or(5.0)));
            gnss_velocity_bias_std.insert(std::make_pair(GNSS_STATUS::NONE, vel_std_[5].value_or(1.0e10)));
            gnss_velocity_bias_std.insert(std::make_pair(GNSS_STATUS::INVALID, vel_std_[6].value_or(1.0e10)));

            gnss_data_refresh_dt = para["gnss"]["gnss_data_refresh_dt"].value_or(0.1);

            enable_ego_position_compensation = para["gnss"]["enable_ego_position_compensation"].value_or(true);
            ego_pos_inno_lat_diff_bound      = para["gnss"]["ego_pos_inno_lat_diff_bound"].value_or(0.2);
            ego_pos_inno_lat_std_bound       = para["gnss"]["ego_pos_inno_lat_std_bound"].value_or(0.2);
            ego_pos_inno_lon_bound           = para["gnss"]["ego_pos_inno_lon_bound"].value_or(0.6);
            ego_pos_inno_lon_mean_bound      = para["gnss"]["ego_pos_inno_lon_mean_bound"].value_or(0.6);
            ego_pos_inno_lon_std_bound       = para["gnss"]["ego_pos_inno_lon_std_bound"].value_or(0.3);
            ego_pos_inno_hgt_bound           = para["gnss"]["ego_pos_inno_hgt_bound"].value_or(0.6);
            ego_pos_inno_hgt_mean_bound      = para["gnss"]["ego_pos_inno_hgt_mean_bound"].value_or(0.6);
            ego_pos_inno_hgt_std_bound       = para["gnss"]["ego_pos_inno_hgt_std_bound"].value_or(0.3);

            vision_pos_lat_bias_std = para["vision_fusion"]["vision_pos_lat_bias_std"].value_or(0.02);
            vision_pos_lon_bias_std = para["vision_fusion"]["vision_pos_lon_bias_std"].value_or(0.06);
            vision_heading_bias_std = para["vision_fusion"]["vision_heading_bias_std"].value_or(0.8) / 180.0 * M_PI;
            vision_data_refresh_dt  = para["vision_fusion"]["vision_data_refresh_dt"].value_or(0.1);
            map_pos_east_bias_rw    = para["vision_fusion"]["map_pos_east_bias_rw"].value_or(0.001);
            map_pos_north_bias_rw   = para["vision_fusion"]["map_pos_north_bias_rw"].value_or(0.002);
            map_heading_bias_rw     = para["vision_fusion"]["map_heading_bias_rw"].value_or(0.03) / 180.0 * M_PI;

            enable_position_smooth_lat       = para["smooth"]["enable_position_smooth_lat"].value_or(true);
            position_smooth_lat_update_bound = para["smooth"]["position_smooth_lat_update_bound"].value_or(2.0);
            position_max_delta_lat_feedback  = para["smooth"]["position_max_delta_lat_feedback"].value_or(0.1);

            is_persistence_enable  = para["persistence"]["is_persistence_enable"].value_or(false);
            vehicle_info_file_path = para["persistence"]["vehicle_info_file_path"].value_or("");

            enable_gnss_time_compesation   = para["time_compesation"]["enable_gnss_time_compesation"].value_or(true);
            enable_wheel_time_compesation  = para["time_compesation"]["enable_wheel_time_compesation"].value_or(true);
            enable_vision_time_compesation = para["time_compesation"]["enable_vision_time_compesation"].value_or(true);

            constrain_euler_angle_imu2navi = Eigen::Vector3d{para["constrain"]["euler_angle_imu2navi"][0].value_or(45.0), para["constrain"]["euler_angle_imu2navi"][1].value_or(45.0), para["constrain"]["euler_angle_imu2navi"][2].value_or(180.0)};
            constrain_euler_angle_imu2navi = constrain_euler_angle_imu2navi / 180.0 * M_PI;

            constrain_velocity_navi2earth = Eigen::Vector3d{para["constrain"]["velocity_navi2earth"][0].value_or(100.0), para["constrain"]["velocity_navi2earth"][1].value_or(100.0), para["constrain"]["velocity_navi2earth"][2].value_or(30.0)};

            constrain_gyro_bias = Eigen::Vector3d{para["constrain"]["gyro_bias"][0].value_or(0.01), para["constrain"]["gyro_bias"][1].value_or(0.01), para["constrain"]["gyro_bias"][2].value_or(0.05)};
            constrain_gyro_bias = constrain_gyro_bias / 180.0 * M_PI;

            constrain_acc_bias = Eigen::Vector3d{para["constrain"]["acc_bias"][0].value_or(0.01), para["constrain"]["acc_bias"][1].value_or(0.01), para["constrain"]["acc_bias"][2].value_or(0.01)};

            constrain_wheel_speed_bias = para["constrain"]["wheel_speed_bias"].value_or(0.01);
            constrain_wheel_speed_bias = constrain_wheel_speed_bias + 0.03; // 这里额外加一点轮速误差系数的上限

            constrain_euler_angle_imu2vehicle = Eigen::Vector3d{para["constrain"]["euler_angle_imu2vehicle"][0].value_or(1.0), para["constrain"]["euler_angle_imu2vehicle"][1].value_or(0.0), para["constrain"]["euler_angle_imu2vehicle"][2].value_or(1.0)};
            constrain_euler_angle_imu2vehicle = constrain_euler_angle_imu2vehicle / 180.0 * M_PI;

            constrain_map_bias     = Eigen::Vector3d{para["constrain"]["map_bias_lat"].value_or(2.0), para["constrain"]["map_bias_lon"].value_or(2.0), para["constrain"]["map_bias_heading"].value_or(1.0)};
            constrain_map_bias.z() = constrain_map_bias.z() / 180.0 * M_PI;

            auto P_std_min_ = para["constrain"]["P_std_min"];
            auto P_std_max_ = para["constrain"]["P_std_max"];

            constrain_P_std_min(0)  = P_std_min_[0].value_or(0.01) / 180.0 * M_PI;
            constrain_P_std_min(1)  = P_std_min_[1].value_or(0.01) / 180.0 * M_PI;
            constrain_P_std_min(2)  = P_std_min_[2].value_or(0.01) / 180.0 * M_PI;
            constrain_P_std_min(3)  = P_std_min_[3].value_or(0.0001);
            constrain_P_std_min(4)  = P_std_min_[4].value_or(0.0001);
            constrain_P_std_min(5)  = P_std_min_[5].value_or(0.0001);
            constrain_P_std_min(6)  = P_std_min_[6].value_or(0.001);
            constrain_P_std_min(7)  = P_std_min_[7].value_or(0.001);
            constrain_P_std_min(8)  = P_std_min_[8].value_or(0.001);
            constrain_P_std_min(9)  = P_std_min_[9].value_or(0.001) / 180.0 * M_PI;
            constrain_P_std_min(10) = P_std_min_[10].value_or(0.001) / 180.0 * M_PI;
            constrain_P_std_min(11) = P_std_min_[11].value_or(0.001) / 180.0 * M_PI;
            constrain_P_std_min(12) = P_std_min_[12].value_or(0.001);
            constrain_P_std_min(13) = P_std_min_[13].value_or(0.001);
            constrain_P_std_min(14) = P_std_min_[14].value_or(0.001);
            constrain_P_std_min(15) = P_std_min_[15].value_or(0.01) / 180.0 * M_PI;
            constrain_P_std_min(16) = P_std_min_[16].value_or(0.0001);
            constrain_P_std_min(17) = P_std_min_[17].value_or(0.01) / 180.0 * M_PI;
            constrain_P_std_min(18) = P_std_min_[18].value_or(0.001);
            constrain_P_std_min(19) = P_std_min_[19].value_or(0.001);
            constrain_P_std_min(20) = P_std_min_[20].value_or(0.001) / 180.0 * M_PI;

            constrain_P_std_max(0)  = P_std_max_[0].value_or(90.0) / 180.0 * M_PI;
            constrain_P_std_max(1)  = P_std_max_[1].value_or(90.0) / 180.0 * M_PI;
            constrain_P_std_max(2)  = P_std_max_[2].value_or(90.0) / 180.0 * M_PI;
            constrain_P_std_max(3)  = P_std_max_[3].value_or(50.0);
            constrain_P_std_max(4)  = P_std_max_[4].value_or(50.0);
            constrain_P_std_max(5)  = P_std_max_[5].value_or(50.0);
            constrain_P_std_max(6)  = P_std_max_[6].value_or(100.0);
            constrain_P_std_max(7)  = P_std_max_[7].value_or(100.0);
            constrain_P_std_max(8)  = P_std_max_[8].value_or(100.0);
            constrain_P_std_max(9)  = P_std_max_[9].value_or(0.1) / 180.0 * M_PI;
            constrain_P_std_max(10) = P_std_max_[10].value_or(0.1) / 180.0 * M_PI;
            constrain_P_std_max(11) = P_std_max_[11].value_or(0.1) / 180.0 * M_PI;
            constrain_P_std_max(12) = P_std_max_[12].value_or(0.1);
            constrain_P_std_max(13) = P_std_max_[13].value_or(0.1);
            constrain_P_std_max(14) = P_std_max_[14].value_or(0.1);
            constrain_P_std_max(15) = P_std_max_[15].value_or(10.0) / 180.0 * M_PI;
            constrain_P_std_max(16) = P_std_max_[16].value_or(0.1);
            constrain_P_std_max(17) = P_std_max_[17].value_or(10.0) / 180.0 * M_PI;
            constrain_P_std_max(18) = P_std_max_[18].value_or(100.0);
            constrain_P_std_max(19) = P_std_max_[19].value_or(100.0);
            constrain_P_std_max(20) = P_std_max_[20].value_or(100.0) / 180.0 * M_PI;

            ego_lat_velocity_constrain = para["constrain"]["ego_lat_velocity_constrain"].value_or(0.3);

            wheel_velocity_additional_std_scale = para["measurement"]["wheel_velocity_additional_std_scale"].value_or(30.0);

            auto position_additional_std_scale_ = para["measurement"]["gnss_position_additional_std_scale"];
            gnss_position_additional_std_scale.clear();
            gnss_position_additional_std_scale.insert(std::make_pair(GNSS_STATUS::FIX, position_additional_std_scale_[0].value_or(0.3)));
            gnss_position_additional_std_scale.insert(std::make_pair(GNSS_STATUS::FLOAT, position_additional_std_scale_[1].value_or(3.0)));
            gnss_position_additional_std_scale.insert(std::make_pair(GNSS_STATUS::PPP, position_additional_std_scale_[2].value_or(3.0)));
            gnss_position_additional_std_scale.insert(std::make_pair(GNSS_STATUS::DGPS, position_additional_std_scale_[3].value_or(6.0)));
            gnss_position_additional_std_scale.insert(std::make_pair(GNSS_STATUS::SINGLE, position_additional_std_scale_[4].value_or(15.0)));
            gnss_position_additional_std_scale.insert(std::make_pair(GNSS_STATUS::NONE, position_additional_std_scale_[5].value_or(1.0e10)));
            gnss_position_additional_std_scale.insert(std::make_pair(GNSS_STATUS::INVALID, position_additional_std_scale_[6].value_or(1.0e10)));

            auto velocity_additional_std_scale_ = para["measurement"]["gnss_velocity_additional_std_scale"];
            gnss_velocity_additional_std_scale.clear();
            gnss_velocity_additional_std_scale.insert(std::make_pair(GNSS_STATUS::FIX, velocity_additional_std_scale_[0].value_or(0.3)));
            gnss_velocity_additional_std_scale.insert(std::make_pair(GNSS_STATUS::FLOAT, velocity_additional_std_scale_[1].value_or(3.0)));
            gnss_velocity_additional_std_scale.insert(std::make_pair(GNSS_STATUS::PPP, velocity_additional_std_scale_[2].value_or(3.0)));
            gnss_velocity_additional_std_scale.insert(std::make_pair(GNSS_STATUS::DGPS, velocity_additional_std_scale_[3].value_or(6.0)));
            gnss_velocity_additional_std_scale.insert(std::make_pair(GNSS_STATUS::SINGLE, velocity_additional_std_scale_[4].value_or(15.0)));
            gnss_velocity_additional_std_scale.insert(std::make_pair(GNSS_STATUS::NONE, velocity_additional_std_scale_[5].value_or(1.0e10)));
            gnss_velocity_additional_std_scale.insert(std::make_pair(GNSS_STATUS::INVALID, velocity_additional_std_scale_[6].value_or(1.0e10)));

            auto heading_additional_std_scale_ = para["measurement"]["gnss_heading_additional_std_scale"];
            gnss_heading_additional_std_scale.clear();
            gnss_heading_additional_std_scale.insert(std::make_pair(GNSS_STATUS::FIX, heading_additional_std_scale_[0].value_or(0.3)));
            gnss_heading_additional_std_scale.insert(std::make_pair(GNSS_STATUS::FLOAT, heading_additional_std_scale_[1].value_or(3.0)));
            gnss_heading_additional_std_scale.insert(std::make_pair(GNSS_STATUS::PPP, heading_additional_std_scale_[2].value_or(3.0)));
            gnss_heading_additional_std_scale.insert(std::make_pair(GNSS_STATUS::DGPS, heading_additional_std_scale_[3].value_or(6.0)));
            gnss_heading_additional_std_scale.insert(std::make_pair(GNSS_STATUS::SINGLE, heading_additional_std_scale_[4].value_or(15.0)));
            gnss_heading_additional_std_scale.insert(std::make_pair(GNSS_STATUS::NONE, heading_additional_std_scale_[5].value_or(1.0e10)));
            gnss_heading_additional_std_scale.insert(std::make_pair(GNSS_STATUS::INVALID, heading_additional_std_scale_[6].value_or(1.0e10)));

            vision_pos_lat_additional_std_scale = para["measurement"]["vision_pos_lat_additional_std_scale"].value_or(1.0);
            vision_pos_lon_additional_std_scale = para["measurement"]["vision_pos_lon_additional_std_scale"].value_or(1.0);
            vision_heading_additional_std_scale = para["measurement"]["vision_heading_additional_std_scale"].value_or(1.0);

            maneuver_status_steady_acc_bound         = para["measurement"]["maneuver_status_steady_acc_bound"].value_or(0.2);
            maneuver_status_low_dynamic_acc_bound    = para["measurement"]["maneuver_status_low_dynamic_acc_bound"].value_or(0.5);
            maneuver_status_steady_rotate_bound      = para["measurement"]["maneuver_status_steady_rotate_bound"].value_or(1.0) / 180.0 * M_PI;
            maneuver_status_low_dynamic_rotate_bound = para["measurement"]["maneuver_status_low_dynamic_rotate_bound"].value_or(2.0) / 180.0 * M_PI;

            gnss_pos_igg3_k0  = para["IGG3"]["gnss_pos_igg3_k0"].value_or(1.0);
            gnss_pos_igg3_k1  = para["IGG3"]["gnss_pos_igg3_k1"].value_or(3.0);
            gnss_vel_igg3_k0  = para["IGG3"]["gnss_vel_igg3_k0"].value_or(1.0);
            gnss_vel_igg3_k1  = para["IGG3"]["gnss_vel_igg3_k1"].value_or(2.0);
            gnss_hdg_igg3_k0  = para["IGG3"]["gnss_hdg_igg3_k0"].value_or(0.5);
            gnss_hdg_igg3_k1  = para["IGG3"]["gnss_hdg_igg3_k1"].value_or(1.0);
            wheel_spd_igg3_k0 = para["IGG3"]["wheel_spd_igg3_k0"].value_or(1.0);
            wheel_spd_igg3_k1 = para["IGG3"]["wheel_spd_igg3_k1"].value_or(2.0);
            zupt_igg3_k0      = para["IGG3"]["zupt_igg3_k0"].value_or(1.0);
            zupt_igg3_k1      = para["IGG3"]["zupt_igg3_k1"].value_or(2.0);

            vision_pos_lat_igg3_k0 = para["IGG3"]["vision_pos_lat_igg3_k0"].value_or(1.0);
            vision_pos_lat_igg3_k1 = para["IGG3"]["vision_pos_lat_igg3_k1"].value_or(2.0);
            vision_pos_lon_igg3_k0 = para["IGG3"]["vision_pos_lon_igg3_k0"].value_or(1.0);
            vision_pos_lon_igg3_k1 = para["IGG3"]["vision_pos_lon_igg3_k1"].value_or(2.0);
            vision_hdg_igg3_k0     = para["IGG3"]["vision_hdg_igg3_k0"].value_or(1.0);
            vision_hdg_igg3_k1     = para["IGG3"]["vision_hdg_igg3_k1"].value_or(2.0);

            attitude_stablization_by_gravity = para["stablization"]["attitude_stablization_by_gravity"].value_or(false);

            use_internal_dr_info = para["dead_reckoning"]["use_internal_dr_info"].value_or(true);

            enable_gnss_fusion         = para["fusion"]["enable_gnss_fusion"].value_or(true);
            enable_gnss_pos_fusion     = para["fusion"]["enable_gnss_pos_fusion"].value_or(true);
            enable_gnss_vel_fusion     = para["fusion"]["enable_gnss_vel_fusion"].value_or(true);
            enable_gnss_heading_fusion = para["fusion"]["enable_gnss_heading_fusion"].value_or(true);
            enable_wheel_vel_fusion    = para["fusion"]["enable_wheel_vel_fusion"].value_or(true);
            enable_vision_fusion       = para["fusion"]["enable_vision_fusion"].value_or(true);
            enable_vision_lat_fusion   = para["fusion"]["enable_vision_lat_fusion"].value_or(true);
            enable_vision_lon_fusion   = para["fusion"]["enable_vision_lon_fusion"].value_or(true);
            enable_vision_hdg_fusion   = para["fusion"]["enable_vision_hdg_fusion"].value_or(true);
            enable_map_fusion          = para["fusion"]["enable_map_fusion"].value_or(true);
            enable_db_fusion           = para["fusion"]["enable_db_fusion"].value_or(true);

            gnss_minimum_speed_required_for_initialization = para["align"]["gnss_minimum_speed_required_for_initialization"].value_or(10.0);

            gnss_maximum_position_innovation_for_fine_align     = para["align"]["gnss_maximum_position_innovation_for_fine_align"].value_or(0.1);
            gnss_maximum_speed_innovation_for_fine_align        = para["align"]["gnss_maximum_speed_innovation_for_fine_align"].value_or(0.1);
            gnss_maximum_heading_innovation_for_fine_align      = para["align"]["gnss_maximum_heading_innovation_for_fine_align"].value_or(0.5) / 180.0 * M_PI;
            gnss_maximum_position_innovation_for_fine_align_std = para["align"]["gnss_maximum_position_innovation_for_fine_align_std"].value_or(0.1);
            gnss_maximum_speed_innovation_for_fine_align_std    = para["align"]["gnss_maximum_speed_innovation_for_fine_align_std"].value_or(0.2);
            gnss_maximum_heading_innovation_for_fine_align_std  = para["align"]["gnss_maximum_heading_innovation_for_fine_align_std"].value_or(0.6) / 180.0 * M_PI;

            gnss_maximum_position_innovation_for_coarse_align     = para["align"]["gnss_maximum_position_innovation_for_coarse_align"].value_or(0.2);
            gnss_maximum_speed_innovation_for_coarse_align        = para["align"]["gnss_maximum_speed_innovation_for_coarse_align"].value_or(0.2);
            gnss_maximum_heading_innovation_for_coarse_align      = para["align"]["gnss_maximum_heading_innovation_for_coarse_align"].value_or(1.0) / 180.0 * M_PI;
            gnss_maximum_position_innovation_for_coarse_align_std = para["align"]["gnss_maximum_position_innovation_for_coarse_align_std"].value_or(0.2);
            gnss_maximum_speed_innovation_for_coarse_align_std    = para["align"]["gnss_maximum_speed_innovation_for_coarse_align_std"].value_or(0.2);
            gnss_maximum_heading_innovation_for_coarse_align_std  = para["align"]["gnss_maximum_heading_innovation_for_coarse_align_std"].value_or(1.0) / 180.0 * M_PI;

            gnss_minimum_position_innovation_for_exit_aligned     = para["align"]["gnss_minimum_position_innovation_for_exit_aligned"].value_or(0.2);
            gnss_minimum_speed_innovation_for_exit_aligned        = para["align"]["gnss_minimum_speed_innovation_for_exit_aligned"].value_or(0.4);
            gnss_minimum_heading_innovation_for_exit_aligned      = para["align"]["gnss_minimum_heading_innovation_for_exit_aligned"].value_or(1.4) / 180.0 * M_PI;
            gnss_minimum_position_innovation_for_exit_aligned_std = para["align"]["gnss_minimum_position_innovation_for_exit_aligned_std"].value_or(0.2);
            gnss_minimum_speed_innovation_for_exit_aligned_std    = para["align"]["gnss_minimum_speed_innovation_for_exit_aligned_std"].value_or(0.4);
            gnss_minimum_heading_innovation_for_exit_aligned_std  = para["align"]["gnss_minimum_heading_innovation_for_exit_aligned_std"].value_or(1.4) / 180.0 * M_PI;

            gnss_minimum_position_innovation_for_exit_fine_align     = para["align"]["gnss_minimum_position_innovation_for_exit_fine_align"].value_or(0.4);
            gnss_minimum_speed_innovation_for_exit_fine_align        = para["align"]["gnss_minimum_speed_innovation_for_exit_fine_align"].value_or(1.0);
            gnss_minimum_heading_innovation_for_exit_fine_align      = para["align"]["gnss_minimum_heading_innovation_for_exit_fine_align"].value_or(2.0) / 180.0 * M_PI;
            gnss_minimum_position_innovation_for_exit_fine_align_std = para["align"]["gnss_minimum_position_innovation_for_exit_fine_align_std"].value_or(0.4);
            gnss_minimum_speed_innovation_for_exit_fine_align_std    = para["align"]["gnss_minimum_speed_innovation_for_exit_fine_align_std"].value_or(1.0);
            gnss_minimum_heading_innovation_for_exit_fine_align_std  = para["align"]["gnss_minimum_heading_innovation_for_exit_fine_align_std"].value_or(2.0) / 180.0 * M_PI;

            gnss_minimum_position_innovation_for_initialization     = para["align"]["gnss_minimum_position_innovation_for_initialization"].value_or(2.0);
            gnss_minimum_speed_innovation_for_initialization        = para["align"]["gnss_minimum_speed_innovation_for_initialization"].value_or(2.0);
            gnss_minimum_heading_innovation_for_initialization      = para["align"]["gnss_minimum_heading_innovation_for_initialization"].value_or(3.0) / 180.0 * M_PI;
            gnss_minimum_position_innovation_for_initialization_std = para["align"]["gnss_minimum_position_innovation_for_initialization_std"].value_or(2.0);
            gnss_minimum_speed_innovation_for_initialization_std    = para["align"]["gnss_minimum_speed_innovation_for_initialization_std"].value_or(2.0);
            gnss_minimum_heading_innovation_for_initialization_std  = para["align"]["gnss_minimum_heading_innovation_for_initialization_std"].value_or(3.0) / 180.0 * M_PI;

            sensor_delay_diagnosis_enable     = para["diagnosis"]["sensor_delay_diagnosis_enable"].value_or(false);
            sensor_delay_valid_bound          = para["diagnosis"]["sensor_delay_valid_bound"].value_or(1.0);
            sensor_delay_filter_bound_gnss    = para["diagnosis"]["sensor_delay_filter_bound_gnss"].value_or(0.2);
            sensor_delay_filter_bound_vehicle = para["diagnosis"]["sensor_delay_filter_bound_vehicle"].value_or(0.2);
            sensor_delay_filter_bound_vision  = para["diagnosis"]["sensor_delay_filter_bound_vision"].value_or(0.2);

            uint64_t pos_center_ = para["output"]["output_position_center"].value_or(0);
            uint64_t ref_frame_  = para["output"]["output_reference_frame"].value_or(0);

            switch (pos_center_) {
                case 0:
                    output_position_center = OUTPUT_POSITION_CENTER::GNSS_ANTENNA;
                    break;
                case 1:
                    output_position_center = OUTPUT_POSITION_CENTER::VEHICLE_REAR_AXLE_CENTER;
                    break;

                default:
                    output_position_center = OUTPUT_POSITION_CENTER::VEHICLE_REAR_AXLE_CENTER;
                    break;
            }
            switch (ref_frame_) {
                case 0:
                    output_reference_frame = OUTPUT_REFERENCE_FRAME::ENU_NAVI_FRAME;
                    break;
                case 1:
                    output_reference_frame = OUTPUT_REFERENCE_FRAME::MAP_FRAME;
                    break;

                default:
                    output_reference_frame = OUTPUT_REFERENCE_FRAME::MAP_FRAME;
                    break;
            }

            vehicle_info_override_enable    = para["vehicle_info_override"]["vehicle_info_override_enable"].value_or(false);
            vehicle_info_override_file_path = para["vehicle_info_override"]["vehicle_info_override_file_path"].value_or("");

            if (vehicle_info_override_enable && vehicle_info_override_file_path != "") {
                update_vehicle_lever(vehicle_info_override_file_path);
            }

            buffer_size                  = para["motion_status"]["buffer_size"].value_or(300);
            max_steer_angle_for_turning  = para["motion_status"]["max_steer_angle_for_turning"].value_or(5.0);
            min_steer_angle_for_straight = para["motion_status"]["min_steer_angle_for_straight"].value_or(2.0);
            max_gyro_z_for_turning       = para["motion_status"]["max_gyro_z_for_turning"].value_or(1.5);
            min_gyro_z_for_straight      = para["motion_status"]["min_gyro_z_for_straight"].value_or(0.5);
            min_speed_for_static         = para["motion_status"]["min_speed_for_static"].value_or(1.0);
            min_acc_x_uniform            = para["motion_status"]["min_acc_x_uniform"].value_or(0.02);
            percent_for_valid            = para["motion_status"]["percent_for_valid"].value_or(0.85);
            observation_window           = para["motion_status"]["observation_window"].value_or(100);

            uint64_t gnss_fusion_mode_ = para["fusion_mode"]["gnss_fusion_mode"].value_or(2);
            switch (gnss_fusion_mode_) {
                case 0: {
                    gnss_fusion_mode = GNSS_LOOSE_COUPLE;
                    AINFO << "----- GNSS LOOSE COUPLE -----";
                } break;
                case 1: {
                    gnss_fusion_mode = GNSS_TIGHT_COUPLE;
                    AINFO << "----- GNSS TIGHT COUPLE -----";
                } break;
                case 2: {
                    gnss_fusion_mode = RTK_LOOSE_COUPLE;
                    AINFO << "----- RTK LOOSE COUPLE -----";
                } break;
                case 3: {
                    gnss_fusion_mode = RTK_TIGHT_COUPLE;
                    AINFO << "----- RTK TIGHT COUPLE -----";
                } break;

                default: {
                    gnss_fusion_mode = RTK_LOOSE_COUPLE;
                    AINFO << "----- RTK LOOSE COUPLE -----";
                } break;
            }

            gnss_fusion_mode_adaption = para["fusion_mode"]["gnss_fusion_mode_adaption"].value_or(false);

            // [sd_map]
            enable_sd_map_bias_comp        = para["sd_map"]["enable_sd_map_bias_comp"].value_or(true);
            sd_map_max_bias_propor_dist    = para["sd_map"]["max_bias_propor_dist"].value_or(0.04);
            sd_map_bias_bound              = para["sd_map"]["map_bias_bound"].value_or(5.0);
            sd_map_min_thres_trig_corr     = para["sd_map"]["min_thres_trig_corr"].value_or(5.0);
            sd_map_bias_fading_propor_dist = para["sd_map"]["bias_fading_propor_dist"].value_or(0.01);

            // AINFO << "TCMSF CONFIG:\n"
            //       << toml::json_formatter{para};
            AINFO << "TCMSF config parsed";

            return true;
        } catch (...) {
            AERROR << "parse config file failed: " << para_file;
        }
        return false;
    }

private:
    Parameters(const std::string para_file_) {
        if (!is_parse_success) {
            para_file        = para_file_;
            is_parse_success = parse();
        }
    }

private:
    struct RTCM {
        bool dump_to_console = false;
    } rtcm;

public:
    bool RTCM_dump_to_console() { return rtcm.dump_to_console; }
};

static const std::string TCMSF_CONFIG_FILE_DIR_ = "modules/localization/conf/tcmsf_init_platform_A.toml";

} // namespace config
} // namespace tcmsf
} // namespace byd
