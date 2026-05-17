#pragma once

#include "modules/msg/drivers_msgs/gps.pb.h"
#include "modules/msg/drivers_msgs/imu.pb.h"
#include "modules/msg/drivers_msgs/ins.pb.h"
#include "modules/msg/drivers_msgs/rtcm.pb.h"
#include "modules/msg/drivers_msgs/veh_info.pb.h"
#include "modules/msg/environment_model_msgs/drive_boundary.pb.h"
#include "modules/msg/localization_msgs/localization_info.pb.h"
#include "modules/msg/localization_msgs/sd_map_match.pb.h"
#include "modules/msg/localization_msgs/tcmsf.pb.h"
#include "modules/msg/localization_msgs/vf_result.pb.h"

#include "cyber/common/log.h"
#include <functional>
#include <memory>

namespace byd {
namespace tcmsf {

using byd::modules::loc_vf::VFResult;
using byd::modules::localization::SDMapMatchResult;
using byd::msg::drivers::Gps;
using byd::msg::drivers::Imu;
using byd::msg::drivers::Ins;
using byd::msg::drivers::Rtcm;
using byd::msg::drivers::VehInfo;
using byd::msg::env_model::DriveBoundary;
using byd::msg::localization::LocalizationEstimate;

using byd::modules::tcmsf::Pose;
class TCMSF {
public:
    virtual ~TCMSF() {
        AINFO << "exit TCMSF interface";
    };

public:
    static std::unique_ptr<TCMSF> create(const std::string &lever_file_ = "");

public:
    virtual int insert_msg(std::shared_ptr<Imu>)                  = 0;
    virtual int insert_msg(std::shared_ptr<Gps>)                  = 0;
    virtual int insert_msg(std::shared_ptr<VehInfo>)              = 0;
    virtual int insert_msg(std::shared_ptr<LocalizationEstimate>) = 0;
    virtual int insert_msg(std::shared_ptr<VFResult>)             = 0;
    virtual int insert_msg(std::shared_ptr<SDMapMatchResult>)     = 0;
    virtual int insert_msg(std::shared_ptr<DriveBoundary>)        = 0;
    virtual int output_msg(std::shared_ptr<Pose>)                 = 0;
    virtual int start_fusion_daemon(std::function<void()>)        = 0;
    virtual int offline_mode_step(std::function<void()>)          = 0;
};

} // namespace tcmsf
} // namespace byd
