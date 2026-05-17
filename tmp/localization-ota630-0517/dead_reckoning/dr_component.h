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

#include "cyber/component/component.h"

#include "imu_issue_fallback_interface.h"


namespace byd {
namespace dr {

using apollo::cyber::Reader;
using apollo::cyber::Writer;

using byd::modules::tcmsf::Pose;
using byd::msg::basic::ModuleStatus;
using byd::msg::drivers::Gps;
using byd::msg::drivers::Imu;
using byd::msg::drivers::Ins;
using byd::msg::drivers::VehInfo;
using byd::msg::localization::LocalizationEstimate;
class DRComponent final : public apollo::cyber::Component<> {
 public:
  DRComponent();
  bool Init() override;
  void Shutdown() override { Component::Shutdown(); }
  ~DRComponent();

 private:
  std::unique_ptr<MSF::IIF::ImuIssueFallbackInterface> iif_ptr_ = nullptr;

 private:
  std::shared_ptr<Reader<Gps>> gps_reader_ = nullptr;
  std::shared_ptr<Reader<Imu>> imu_reader_ = nullptr;
  std::shared_ptr<Reader<Ins>> ins_reader_ = nullptr;
  std::shared_ptr<Reader<VehInfo>> veh_reader_ = nullptr;
  std::shared_ptr<Reader<Pose>> msf_reader_ = nullptr;

  apollo::cyber::CallbackFunc<Gps> gps_reader_cb_ = nullptr;
  apollo::cyber::CallbackFunc<Imu> imu_reader_cb_ = nullptr;
  apollo::cyber::CallbackFunc<Ins> ins_reader_cb_ = nullptr;
  apollo::cyber::CallbackFunc<VehInfo> veh_reader_cb_ = nullptr;
  apollo::cyber::CallbackFunc<Pose> msf_reader_cb_ = nullptr;

  std::atomic<bool> gps_dispose_;
  std::atomic<bool> imu_dispose_;
  std::atomic<bool> ins_dispose_;
  std::atomic<bool> veh_dispose_;
  std::atomic<bool> msf_dispose_;

 private:
  std::shared_ptr<LocalizationEstimate> dr_result_msg_ = nullptr;
  std::shared_ptr<Writer<LocalizationEstimate>> dr_result_writer_ = nullptr;
  std::shared_ptr<ModuleStatus> module_status_msg_ = nullptr;
  std::shared_ptr<Writer<ModuleStatus>> module_status_writer_ = nullptr;
  uint32_t sequence_num;
  std::function<void(const DrData&)> dr_result_cb_ = nullptr;

 private:
  STATE_TIMER dr_step_timer;
  std::unique_ptr<DR_interface> dr_ = nullptr;
  bool enable_dr_daemon;
  bool enable_msf_info;
  int rt_priority_;
  int rt_priority_delay_s_;

 private:
  void RegisterCallbackFunctions();
  void parse_config(const std::string& config_file_path);
};

CYBER_REGISTER_COMPONENT(DRComponent)

}  // namespace dr
}  // namespace byd
