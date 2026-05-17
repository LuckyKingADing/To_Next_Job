#include "persistence_vehicle_info.h"
#include <filesystem>
#include <fstream>

namespace byd {
namespace persistence {

static constexpr double DEGREE_2_RADIAN = 3.141592653589793 / 180.0;
static constexpr double RADIAN_2_DEGREE = 180.0 / 3.141592653589793;

bool VehicleInfo::create_cfg_file_if_not_exist() {
    namespace FS = std::filesystem;
    FS::path cfg_path_{filepath};
    if (FS::exists(cfg_path_) && FS::status(cfg_path_).type() == FS::file_type::regular) {
        return true;
    } else if (!FS::exists(cfg_path_.parent_path())) {
        try {
            if (FS::create_directories(cfg_path_.parent_path())) {
                AINFO << "file parent path not exist, create: " << cfg_path_.parent_path();
                std::fstream cfg_fs;
                cfg_fs.open(filepath, std::ios::out);
                if (cfg_fs.is_open()) {
                    AINFO << "create file: " << filepath;
                    cfg_fs.close();
                    return true;
                }
            }
        } catch (...) {
            return false;
        }
    } else if (FS::exists(cfg_path_.parent_path())) {
        try {
            std::fstream cfg_fs;
            cfg_fs.open(filepath, std::ios::out);
            if (cfg_fs.is_open()) {
                AINFO << "create file: " << filepath;
                cfg_fs.close();
                return true;
            }
        } catch (...) {
            return false;
        }
    }
    return false;
}

bool VehicleInfo::parse_states_from_file() {
    std::unique_lock<std::recursive_mutex> lock_(write_mutex_);
    AINFO << "parse vehicle info from [" << filepath << "]";
    try {
        para                                   = toml::parse_file(filepath);
        vehicle_states.pitch_misalignment      = para["vehicle_info"]["pitch_misalignment"].value_or(0.0) * DEGREE_2_RADIAN; // 转换为标准的弧度
        vehicle_states.wheel_speed_scale_error = para["vehicle_info"]["wheel_speed_scale_error"].value_or(0.006);            // 如果没读到值，按经验，给一个略大于0.0的值
        vehicle_states.yaw_misalignment        = para["vehicle_info"]["yaw_misalignment"].value_or(0.0) * DEGREE_2_RADIAN;
        AINFO << "pitch misalignment [deg]: " << vehicle_states.pitch_misalignment * RADIAN_2_DEGREE;
        AINFO << "wheel speed scale error: " << vehicle_states.wheel_speed_scale_error;
        AINFO << "yaw misalignment [deg]: " << vehicle_states.yaw_misalignment * RADIAN_2_DEGREE;
    } catch (...) {
        return false;
    }
    return true;
}

VehicleStates VehicleInfo::get_state() {
    std::unique_lock<std::recursive_mutex> lock_(write_mutex_);
    return vehicle_states;
}

void VehicleInfo::update_state(const VehicleStates &vehicle_states_) {
    if (write_mutex_.try_lock()) {
        vehicle_states.pitch_misalignment      = vehicle_states_.pitch_misalignment;
        vehicle_states.wheel_speed_scale_error = vehicle_states_.wheel_speed_scale_error;
        vehicle_states.yaw_misalignment        = vehicle_states_.yaw_misalignment;
        write_mutex_.unlock();
    }
}

bool VehicleInfo::save_states_to_file() {
    std::unique_lock<std::recursive_mutex> lock_(write_mutex_);
    AINFO << "pitch misalignment [deg]: " << vehicle_states.pitch_misalignment * RADIAN_2_DEGREE;
    AINFO << "wheel speed scale error: " << vehicle_states.wheel_speed_scale_error;
    AINFO << "yaw misalignment [deg]: " << vehicle_states.yaw_misalignment * RADIAN_2_DEGREE;
    auto tbl = toml::table{
        {
            "vehicle_info",
            toml::table{
                {"pitch_misalignment", vehicle_states.pitch_misalignment * RADIAN_2_DEGREE}, // 以角度为单位存储，便于人类阅读
                {"wheel_speed_scale_error", vehicle_states.wheel_speed_scale_error},         //
                {"yaw_misalignment", vehicle_states.yaw_misalignment * RADIAN_2_DEGREE}      //
            }                                                                                //
        }                                                                                    //
    };

    std::fstream cfg_fs;
    cfg_fs.open(filepath, std::ios::out);
    if (cfg_fs.is_open()) {
        cfg_fs << "# vehicle dynamic calibration info\n"
               << "# misalignment angle in degrees\n\n";
        cfg_fs << toml::toml_formatter(tbl);
        try {
            cfg_fs.close();
            AINFO << "save vehicle info to [" << filepath << "]";
        } catch (...) {
            AWARN << "save vehicle info to [" << filepath << "] failed";
        }
        return true;
    } else {
        AINFO << "save vehicle info to [" << filepath << "] failed";
    }

    return false;
}

void VehicleInfo::async_persistence_once() {
    if (persistence_async_running.exchange(true)) {
        AINFO << "async_persistence_once is not finished.";
        return;
    }
    persistence_result_ = std::async(std::launch::async, [this]() {
        AINFO << "trigger vehicle info persistence.";
        if (!is_persistence_enable.load()) {
            return false;
        }
        return save_states_to_file();
    });
    persistence_async_running.exchange(false);
}

} // namespace persistence
} // namespace byd