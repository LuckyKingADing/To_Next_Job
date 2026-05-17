#pragma once

#include "dr_interface.h"
#include "time_trigger.h"
#include "yaml-cpp/yaml.h"

#include "modules/msg/basic_msgs/module_status.pb.h"
#include "modules/msg/drivers_msgs/gps.pb.h"
#include "modules/msg/drivers_msgs/imu.pb.h"
#include "modules/msg/drivers_msgs/ins.pb.h"
#include "modules/msg/drivers_msgs/veh_info.pb.h"
#include "modules/msg/localization_msgs/localization_info.pb.h"
#include "modules/msg/localization_msgs/tcmsf.pb.h"

#include "modules/common/topic/topic_gflags.h"

#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"
#include "rigid_transform.h"

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

using byd::modules::tcmsf::Pose;
using byd::msg::basic::ModuleStatus;
using byd::msg::drivers::Gps;
using byd::msg::drivers::Imu;
using byd::msg::drivers::Ins;
using byd::msg::drivers::VehInfo;
using byd::msg::localization::LocalizationEstimate;

class DR_ {
private:
    std::unique_ptr<byd::dr::DR_interface> dr_ = byd::dr::DR_interface::create();

private:
    void refine_imu(std::shared_ptr<Imu> imu_, std::shared_ptr<Pose> msf_);
    void refine_wheel(std::shared_ptr<VehInfo> veh_, std::shared_ptr<Pose> msf_);

public:
    DR_() {
        Eigen::Matrix<double, 9, 1> static_bound;
        static_bound << 1e-3, 1e-3, 1e-3, 4e-3, 4e-3, 6e-3, 4e-3, 4e-3, 4e-3;
        dr_->set_static_bound(static_bound);
    }
    int run(const std::string &, const std::string &);
};
