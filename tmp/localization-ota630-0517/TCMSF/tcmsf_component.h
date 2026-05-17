#pragma once

#include "cyber/component/component.h"
// #include "modules/msg/basic_msgs/module_status.pb.h"
#include "modules/msg/drivers_msgs/gps.pb.h"
#include "modules/msg/drivers_msgs/imu.pb.h"
#include "modules/msg/drivers_msgs/ins.pb.h"
#include "modules/msg/drivers_msgs/rtcm.pb.h"
#include "modules/msg/drivers_msgs/veh_info.pb.h"
#include "modules/msg/environment_model_msgs/drive_boundary.pb.h"
#include "modules/msg/localization_msgs/localization_info.pb.h"
#include "modules/msg/localization_msgs/result_info.pb.h"
#include "modules/msg/localization_msgs/sd_map_match.pb.h"
#include "modules/msg/localization_msgs/tcmsf.pb.h"
#include "modules/msg/localization_msgs/vf_result.pb.h"

#include "cyber/event/trace.h"
// #include "tcmsf_config.h"
#include "tcmsf_interface.h"
// #include "tcmsf_interface_impl.h"
#include <atomic>

#include "rtcm_interface.h"
#include "tcmsf_timer.h"

#include "imu_issue_fallback_interface.h"

namespace byd {
namespace tcmsf {

using apollo::cyber::Reader;
using apollo::cyber::Writer;

using byd::modules::localization::LocResult;
using byd::modules::localization::SDMapMatchResult;
using byd::modules::tcmsf::Pose;
using byd::msg::drivers::Gps;
using byd::msg::drivers::Imu;
using byd::msg::drivers::Ins;
using byd::msg::drivers::Rtcm;
using byd::msg::drivers::VehInfo;
using byd::msg::env_model::DriveBoundary;
using byd::msg::localization::LocalizationEstimate;

class TCMSFComponent final : public apollo::cyber::Component<> {
public:
    TCMSFComponent();
    bool Init() override;
    void Shutdown() override { Component::Shutdown(); }
    ~TCMSFComponent();

private:
    std::unique_ptr<MSF::IIF::ImuIssueFallbackInterface> iif_ptr_ = nullptr;

private:
    std::shared_ptr<Reader<Gps>>                  gps_reader_    = nullptr;
    std::shared_ptr<Reader<Imu>>                  imu_reader_    = nullptr;
    std::shared_ptr<Reader<Ins>>                  ins_reader_    = nullptr;
    std::shared_ptr<Reader<VehInfo>>              veh_reader_    = nullptr;
    std::shared_ptr<Reader<Rtcm>>                 rover_reader_  = nullptr;
    std::shared_ptr<Reader<Rtcm>>                 base_reader_   = nullptr;
    std::shared_ptr<Reader<LocalizationEstimate>> dr_reader_     = nullptr;
    std::shared_ptr<Reader<VFResult>>             vision_reader_ = nullptr;
    std::shared_ptr<Reader<SDMapMatchResult>>     sdmm_reader_   = nullptr;
    std::shared_ptr<Reader<DriveBoundary>>        db_reader_     = nullptr;

    apollo::cyber::CallbackFunc<Gps>                  gps_reader_cb_    = nullptr;
    apollo::cyber::CallbackFunc<Imu>                  imu_reader_cb_    = nullptr;
    apollo::cyber::CallbackFunc<Ins>                  ins_reader_cb_    = nullptr;
    apollo::cyber::CallbackFunc<VehInfo>              veh_reader_cb_    = nullptr;
    apollo::cyber::CallbackFunc<Rtcm>                 rover_reader_cb_  = nullptr;
    apollo::cyber::CallbackFunc<Rtcm>                 base_reader_cb_   = nullptr;
    apollo::cyber::CallbackFunc<LocalizationEstimate> dr_reader_cb_     = nullptr;
    apollo::cyber::CallbackFunc<VFResult>             vision_reader_cb_ = nullptr;
    apollo::cyber::CallbackFunc<SDMapMatchResult>     sdmm_reader_cb_   = nullptr;
    apollo::cyber::CallbackFunc<DriveBoundary>        db_reader_cb_     = nullptr;

    std::function<void()> result_cb_ = nullptr;

    std::atomic<bool> gps_dispose_;
    std::atomic<bool> imu_dispose_;
    std::atomic<bool> ins_dispose_;
    std::atomic<bool> veh_dispose_;
    std::atomic<bool> rover_dispose_;
    std::atomic<bool> base_dispose_;
    std::atomic<bool> tcmsf_dispose_;
    std::atomic<bool> dr_dispose_;
    std::atomic<bool> vision_dispose_;
    std::atomic<bool> sdmm_dispose_;
    std::atomic<bool> db_dispose_;

private:
    std::shared_ptr<Pose>              tcmsf_result_msg_    = nullptr;
    std::shared_ptr<Writer<Pose>>      tcmsf_result_writer_ = nullptr;
    std::shared_ptr<LocResult>         msf_result_msg_      = nullptr;
    std::shared_ptr<Writer<LocResult>> msf_result_writer_   = nullptr;

private:
    time::Timer gps_timer    = time::Timer("gps");
    time::Timer imu_timer    = time::Timer("imu");
    time::Timer ins_timer    = time::Timer("ins");
    time::Timer veh_timer    = time::Timer("vehicle");
    time::Timer rover_timer  = time::Timer("rover");
    time::Timer base_timer   = time::Timer("base");
    time::Timer tcmsf_timer  = time::Timer("tcmsf");
    time::Timer dr_timer     = time::Timer("dr");
    time::Timer vision_timer = time::Timer("vision");
    time::Timer sdmm_timer   = time::Timer("sdmm");
    time::Timer db_timer     = time::Timer("db");

private:
    std::unique_ptr<rtcm::Resolve> resolve_ = nullptr;

private:
    std::unique_ptr<TCMSF> tcmsf_ = nullptr;

private:
    void RegisterCallbackFunctions();

private:
    void update_msf_from_tcmsf();
};

CYBER_REGISTER_COMPONENT(TCMSFComponent)

} // namespace tcmsf
} // namespace byd
