#pragma once

#include "cyber/common/log.h"
#include "fmt/format.h"
#include "toml++/toml.h"
#include <future>
#include <mutex>
// #include <signal.h>
#include <string>

namespace byd {
namespace persistence {

class VehicleStates {
public:
    VehicleStates() :
        pitch_misalignment(0.0),
        wheel_speed_scale_error(0.0),
        yaw_misalignment(0.0) {}
    VehicleStates(double pitch_misalignment_, double wheel_speed_scale_error_, double yaw_misalignment_) {
        pitch_misalignment      = pitch_misalignment_;
        wheel_speed_scale_error = wheel_speed_scale_error_;
        yaw_misalignment        = yaw_misalignment_;
    }
    bool isNaN() {
        return std::isnan(pitch_misalignment) || std::isnan(wheel_speed_scale_error) || std::isnan(yaw_misalignment);
    }
    void setZero() {
        pitch_misalignment      = 0.0;
        wheel_speed_scale_error = 0.0;
        yaw_misalignment        = 0.0;
    }

public:
    double pitch_misalignment;
    double wheel_speed_scale_error;
    double yaw_misalignment;
};

class VehicleInfo {
public:
    static VehicleInfo &getInstance(const std::string &filepath_, bool enable_) {
        static VehicleInfo inst(filepath_, enable_);
        return inst;
    }

    VehicleInfo(const VehicleInfo &)            = delete;
    VehicleInfo &operator=(const VehicleInfo &) = delete;

    ~VehicleInfo() {
        AINFO << "exit vehicle info";
        if (!is_persistence_enable.load()) {
            return;
        }
        if (persistence_result_.valid()) {
            AINFO << "wait until async_persistence_once finished.";
            persistence_result_.get();
        }
        // save_states_to_file();
    }

    // private:
    //     static void term_handle(int sig) {
    //         exit(0);
    //     }

private:
    VehicleInfo(const std::string &filepath_, bool enable_) {
        is_persistence_enable.store(enable_);
        if (!is_persistence_enable.load()) {
            return;
        }
        if (!is_parsed) {
            filepath = filepath_;
            create_cfg_file_if_not_exist();
            parse_states_from_file();
            is_parsed = true;

            // // 注册一个SIGTERM处理函数，保证在接收到KILL信号的时候，持久化类能够正常析构
            // signal(SIGTERM, term_handle);
        }
    }

private:
    std::atomic_bool is_persistence_enable{false};

private:
    VehicleStates vehicle_states;
    std::string   filepath;
    bool          is_parsed = false;

private:
    toml::v3::ex::parse_result para;

public:
    VehicleStates get_state();
    void          update_state(const VehicleStates &vehicle_states_);

private:
    bool create_cfg_file_if_not_exist();
    bool parse_states_from_file();
    bool save_states_to_file();

private:
    std::recursive_mutex write_mutex_;

public:
    void async_persistence_once();

private:
    std::future<bool> persistence_result_;
    std::atomic_bool  persistence_async_running{false};
};

} // namespace persistence
} // namespace byd