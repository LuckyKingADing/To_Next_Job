#include "dr_component.h"

#include "cyber/event/trace.h"

namespace byd {
namespace dr {

// 坐标脱敏需求，使用加偏后的坐标
// #define GPS_TPC "/drivers/gnss/raw"
#define GPS_TPC "/drivers/gnss/packet"

#define IMU_TPC "/drivers/imu/raw"
#define INS_TPC "/drivers/ins/raw"
#define VEH_TPC "/drivers/canbus/vehicle_info"
#define MSF_TPC "/localization/tcmsf"
#define DR_TPC "/localization/dr"
#define ModSta_TPC "/localization/dr/status"

using apollo::cyber::ReaderConfig;

DRComponent::DRComponent()
    : gps_dispose_(false),
      imu_dispose_(false),
      ins_dispose_(false),
      veh_dispose_(false),
      msf_dispose_(false) {
  enable_dr_daemon = false;
  enable_msf_info = false;
  rt_priority_ = 0;
  rt_priority_delay_s_ = 0;
  sequence_num = 1;
  dr_ = DR_interface::create();
  dr_result_msg_ = std::make_shared<LocalizationEstimate>();
  module_status_msg_ = std::make_shared<ModuleStatus>();
}

DRComponent::~DRComponent() {
  if (dr_) {
    dr_->stop_drdaemon();
  }
}

bool DRComponent::Init() {
  parse_config("modules/localization/conf/dr_init.yml");
  {
    Eigen::Matrix<double, 9, 1> static_bound;
    static_bound << 1e-3, 1e-3, 1e-3, 4e-3, 4e-3, 6e-3, 4e-3, 4e-3, 4e-3;
    dr_->set_static_bound(static_bound);
  }
  iif_ptr_ = MSF::IIF::ImuIssueFallbackInterface::create();
  ACHECK(iif_ptr_ != nullptr);
  RegisterCallbackFunctions();
  dr_step_timer.reset();
  ReaderConfig veh_cfg, gps_cfg, imu_cfg, ins_cfg, msf_cfg, dr_cfg;
  veh_cfg.channel_name = VEH_TPC;
  veh_cfg.pending_queue_size = 5;
  gps_cfg.channel_name = GPS_TPC;
  gps_cfg.pending_queue_size = 2;
  imu_cfg.channel_name = IMU_TPC;
  imu_cfg.pending_queue_size = 5;
  ins_cfg.channel_name = INS_TPC;
  ins_cfg.pending_queue_size = 5;
  msf_cfg.channel_name = MSF_TPC;
  msf_cfg.pending_queue_size = 5;
  gps_reader_ = node_->CreateReader<Gps>(gps_cfg, gps_reader_cb_);
  ACHECK(gps_reader_ != nullptr);
  imu_reader_ = node_->CreateReader<Imu>(imu_cfg, imu_reader_cb_);
  ACHECK(imu_reader_ != nullptr);
  veh_reader_ = node_->CreateReader<VehInfo>(veh_cfg, veh_reader_cb_);
  ACHECK(veh_reader_ != nullptr);
  if (enable_msf_info) {
    msf_reader_ = node_->CreateReader<Pose>(msf_cfg, msf_reader_cb_);
    ACHECK(msf_reader_ != nullptr);
  } else {
    ins_reader_ = node_->CreateReader<Ins>(ins_cfg, ins_reader_cb_);
    ACHECK(ins_reader_ != nullptr);
  }
  dr_result_writer_ = node_->CreateWriter<LocalizationEstimate>(DR_TPC);
  ACHECK(dr_result_writer_ != nullptr);
  module_status_writer_ = node_->CreateWriter<ModuleStatus>(ModSta_TPC);
  ACHECK(module_status_writer_ != nullptr);
  if (enable_dr_daemon) {  // using a daemon thread to trigger dr update
    dr_->start_dr_daemon(dr_result_cb_, 0.01, rt_priority_,
                         rt_priority_delay_s_);
  }
  return true;
}

void DRComponent::RegisterCallbackFunctions() {
  gps_reader_cb_ = [this](const std::shared_ptr<Gps>& gps_msg_) {
    RETURN_IF_MSG_INVALID(gps_msg_);
    PERFORMANCE_TRACE_START(DR, gps);
    if (gps_dispose_.exchange(true)) {
      AINFO << "new message arrived while gps_reader_cb_ not finished yet!";
      return;
    }
    if (gps_msg_->has_header() &&
        gps_msg_->header().has_measurement_timestamp() &&
        gps_msg_->has_position_status()) {
      GpsData gps_data;
      gps_data.timestamp = gps_msg_->header().measurement_timestamp();
      gps_data.status = gps_msg_->position_status();
      dr_->insert_gps(gps_data);
    }
    gps_dispose_.exchange(false);
  };
  imu_reader_cb_ = [this](const std::shared_ptr<Imu>& imu_msg_) {
    RETURN_IF_MSG_INVALID(imu_msg_);
    PERFORMANCE_TRACE_START(DR, imu);
    if (imu_dispose_.exchange(true)) {
      AINFO << "new message arrived while imu_reader_cb_ not finished yet!";
      return;
    }
    auto imu_msg_mod_ = iif_ptr_->insert_imu(imu_msg_);
    if (imu_msg_mod_->has_header() &&
        imu_msg_mod_->header().has_measurement_timestamp() &&
        imu_msg_mod_->has_accel() && imu_msg_mod_->accel().has_x() &&
        imu_msg_mod_->accel().has_y() && imu_msg_mod_->accel().has_z() &&
        imu_msg_mod_->has_gyro() && imu_msg_mod_->gyro().has_x() &&
        imu_msg_mod_->gyro().has_y() && imu_msg_mod_->gyro().has_z() &&
        imu_msg_mod_->has_imu_status()) {
      ImuData imu_data;
      imu_data.timestamp = imu_msg_mod_->header().measurement_timestamp();
      imu_data.acc_x = imu_msg_mod_->accel().x();
      imu_data.acc_y = imu_msg_mod_->accel().y();
      imu_data.acc_z = imu_msg_mod_->accel().z();
      imu_data.gyro_x = imu_msg_mod_->gyro().x() / 180.0 * M_PI;
      imu_data.gyro_y = imu_msg_mod_->gyro().y() / 180.0 * M_PI;
      imu_data.gyro_z = imu_msg_mod_->gyro().z() / 180.0 * M_PI;
      if (std::isnan(imu_data.acc_x) || std::isnan(imu_data.gyro_x) ||
          std::isnan(imu_data.acc_y) || std::isnan(imu_data.gyro_y) ||
          std::isnan(imu_data.acc_z) || std::isnan(imu_data.gyro_z)) {
        // 检查是否有NaN，如果有则抛弃
        AERROR << "IMU msg NaN detected, drop!";
      } else if (imu_msg_mod_->imu_status() != 1) {
        // 检查IMU状态是否OK，如果不则抛弃
        AERROR_EVERY(100) << "imu status is not ok, drop!";
      } else {
        dr_->insert_imu(imu_data);
      }
    }
    if (!enable_dr_daemon) {  // imu message trigger dr update
      auto dt = dr_step_timer.dt();
      dr_step_timer.reset();
      if (dt < 0.1) {
        dr_->dr_step(dt);
        auto dr_data = dr_->get_result();
        dr_result_cb_(dr_data);
      }
    }
    imu_dispose_.exchange(false);
  };
  veh_reader_cb_ = [this](const std::shared_ptr<VehInfo>& veh_msg_) {
    RETURN_IF_MSG_INVALID(veh_msg_);
    PERFORMANCE_TRACE_START(DR, veh);
    if (veh_dispose_.exchange(true)) {
      AINFO << "new message arrived while veh_reader_cb_ not finished yet!";
      return;
    }
    iif_ptr_->insert_veh(veh_msg_);
    if (veh_msg_->has_header() &&
        veh_msg_->header().has_measurement_timestamp() &&
        veh_msg_->has_ego_motion_status() &&
        veh_msg_->ego_motion_status().has_da_in_rlwhlspd_sg() &&
        veh_msg_->ego_motion_status().has_da_in_rlwhlspd_v_u8() &&
        veh_msg_->ego_motion_status().has_da_in_rrwhlspd_sg() &&
        veh_msg_->ego_motion_status().has_da_in_rrwhlspd_v_u8() &&
        veh_msg_->ego_motion_status().has_da_in_rlwhldrvdir_u8() &&
        veh_msg_->ego_motion_status().has_da_in_rrwhldrvdir_u8() &&
        veh_msg_->ego_motion_status().has_da_in_rlwhldrvdir_v_u8() &&
        veh_msg_->ego_motion_status().has_da_in_rrwhldrvdir_v_u8() &&
        veh_msg_->has_propulsion_system_status() &&
        veh_msg_->propulsion_system_status().has_da_in_trmgearlvl_u8() &&
        veh_msg_->ego_motion_status().has_da_in_yawrate_sg()) {
      if (veh_msg_->ego_motion_status().da_in_rlwhlspd_v_u8() &&
          veh_msg_->ego_motion_status().da_in_rrwhlspd_v_u8()) {
        VehData veh_data;
        veh_data.timestamp = veh_msg_->header().measurement_timestamp();

        auto dir_rl = veh_msg_->ego_motion_status().da_in_rlwhldrvdir_u8();
        auto dir_rr = veh_msg_->ego_motion_status().da_in_rrwhldrvdir_u8();

        auto dir_rl_v = veh_msg_->ego_motion_status().da_in_rlwhldrvdir_v_u8();
        auto dir_rr_v = veh_msg_->ego_motion_status().da_in_rrwhldrvdir_v_u8();

        auto trmgear =
            veh_msg_->propulsion_system_status().da_in_trmgearlvl_u8();

        auto vel_rl =
            std::abs(veh_msg_->ego_motion_status().da_in_rlwhlspd_sg());
        auto vel_rr =
            std::abs(veh_msg_->ego_motion_status().da_in_rrwhlspd_sg());

        if (dir_rl_v == 0 && dir_rr_v == 0) {
          if ((vel_rl > 1e-10 || vel_rr > 1e-10) &&
              (dir_rl == 3 || dir_rr == 3)) {
            // 这里考虑轮速方向的延迟
            // 如果出现轮速有值，但是方向给静止的话，就使用挡位来判断轮速的方向
            veh_data.spd_rl = (trmgear == 4) ? -vel_rl : vel_rl;
            veh_data.spd_rr = (trmgear == 4) ? -vel_rr : vel_rr;
          } else {
            // 左右后轮行驶方向状态均有效
            veh_data.spd_rl = (dir_rl == 2) ? -vel_rl : vel_rl;
            veh_data.spd_rr = (dir_rr == 2) ? -vel_rr : vel_rr;
          }
        } else if (dir_rl_v == 0 || dir_rr_v == 0) {
          // 左右后轮行驶方向状态一个无效，一个有效
          if (dir_rl_v == 0) {
            veh_data.spd_rl = (dir_rl == 2) ? -vel_rl : vel_rl;
            veh_data.spd_rr = (dir_rl == 2) ? -vel_rr : vel_rr;
          }
          if (dir_rr_v == 0) {
            veh_data.spd_rr = (dir_rr == 2) ? -vel_rr : vel_rr;
            veh_data.spd_rl = (dir_rr == 2) ? -vel_rl : vel_rl;
          }
        } else {
          // 左右后轮行驶方向标志都无效
          if (trmgear == 4) {
            veh_data.spd_rl = -vel_rl;
            veh_data.spd_rr = -vel_rr;
          } else {
            veh_data.spd_rl = vel_rl;
            veh_data.spd_rr = vel_rr;
          }
        }

        veh_data.yaw_rate = veh_msg_->ego_motion_status().da_in_yawrate_sg();
        dr_->insert_veh(veh_data);
      }
    }
    veh_dispose_.exchange(false);
  };
  ins_reader_cb_ = [this](const std::shared_ptr<Ins>& ins_msg_) {
    RETURN_IF_MSG_INVALID(ins_msg_);
    PERFORMANCE_TRACE_START(DR, ins);
    if (ins_dispose_.exchange(true)) {
      AINFO << "new message arrived while ins_reader_cb_ not finished yet!";
      return;
    }
    if (ins_msg_->has_header() &&
        ins_msg_->header().has_measurement_timestamp() &&
        ins_msg_->has_linear_velocity() &&
        ins_msg_->linear_velocity().has_x() &&
        ins_msg_->linear_velocity().has_y() &&
        ins_msg_->linear_velocity().has_z() &&
        ins_msg_->has_orientation_rpy() &&
        ins_msg_->orientation_rpy().has_z()) {
      MsfData msf_data;
      msf_data.timestamp = ins_msg_->header().measurement_timestamp();
      msf_data.vel_x = ins_msg_->linear_velocity().x();
      msf_data.vel_y = ins_msg_->linear_velocity().y();
      msf_data.vel_z = ins_msg_->linear_velocity().z();
      msf_data.heading = ins_msg_->orientation_rpy().z();
      dr_->insert_msf(msf_data);
    }
    ins_dispose_.exchange(false);
  };
  msf_reader_cb_ = [this](const std::shared_ptr<Pose>& msf_msg_) {
    RETURN_IF_MSG_INVALID(msf_msg_);
    PERFORMANCE_TRACE_START(DR, msf);
    if (msf_dispose_.exchange(true)) {
      AINFO << "new message arrived while msf_reader_cb_ not finished yet!";
      return;
    }
    if (msf_msg_->has_header() &&
        msf_msg_->header().has_measurement_timestamp() &&
        msf_msg_->has_velocity() && msf_msg_->velocity().has_x() &&
        msf_msg_->velocity().has_y() && msf_msg_->velocity().has_z() &&
        msf_msg_->has_acc_bias() && msf_msg_->has_gyro_bias() &&
        msf_msg_->acc_bias().has_x() && msf_msg_->acc_bias().has_y() &&
        msf_msg_->acc_bias().has_z() && msf_msg_->gyro_bias().has_x() &&
        msf_msg_->gyro_bias().has_y() && msf_msg_->gyro_bias().has_z() &&
        msf_msg_->has_heading() && msf_msg_->has_zupt_count() &&
        msf_msg_->has_fusion_status()) {
      MsfData msf_data;
      msf_data.timestamp = msf_msg_->header().measurement_timestamp();
      msf_data.vel_x = msf_msg_->velocity().x();
      msf_data.vel_y = msf_msg_->velocity().y();
      msf_data.vel_z = msf_msg_->velocity().z();
      msf_data.bias_acc_x = msf_msg_->acc_bias().x();
      msf_data.bias_acc_y = msf_msg_->acc_bias().y();
      msf_data.bias_acc_z = msf_msg_->acc_bias().z();
      msf_data.bias_gyro_x = msf_msg_->gyro_bias().x();
      msf_data.bias_gyro_y = msf_msg_->gyro_bias().y();
      msf_data.bias_gyro_z = msf_msg_->gyro_bias().z();
      msf_data.heading = msf_msg_->heading();
      msf_data.msf_state = msf_msg_->fusion_status();
      msf_data.zupt_count = msf_msg_->zupt_count();
      if (msf_msg_->fusion_status() ==
              Pose::FusionStatus::Pose_FusionStatus_FULLSTATE ||
          msf_msg_->fusion_status() ==
              Pose::FusionStatus::Pose_FusionStatus_GPSONLY) {
        dr_->insert_msf(msf_data);
      }
    }
    msf_dispose_.exchange(false);
  };

  dr_result_cb_ = [this](const DrData& dr_data) {
    {
      // 检测如果有NaN值，则不输出
      if(
        std::isnan(dr_data.ori_w) || 
        std::isnan(dr_data.ori_x) || 
        std::isnan(dr_data.ori_y) || 
        std::isnan(dr_data.ori_z) || 
        std::isnan(dr_data.pos_x) || 
        std::isnan(dr_data.pos_y) || 
        std::isnan(dr_data.pos_z) || 
        std::isnan(dr_data.vel_x) || 
        std::isnan(dr_data.vel_y) || 
        std::isnan(dr_data.vel_z) || 
        std::isnan(dr_data.heading)
      ){
        AINFO_EVERY(100)<<"DR NaN detected!";
        return;
      }
    }
    // PERFORMANCE_TRACE_START(DR, dr_step); // 这个LOG可能导致DR卡顿，删除。
    dr_result_msg_->Clear();
    auto header_ = dr_result_msg_->mutable_header();
    auto pose_ = dr_result_msg_->mutable_pose();
    header_->set_frame_id("dr");
    header_->set_sequence_num(sequence_num);
    header_->set_measurement_timestamp(dr_data.timestamp);
    header_->set_publish_timestamp(apollo::cyber::Time::Now().ToSecond());
    pose_->mutable_position()->set_x(dr_data.pos_x);
    pose_->mutable_position()->set_y(dr_data.pos_y);
    pose_->mutable_position()->set_z(dr_data.pos_z);
    pose_->mutable_orientation()->set_qw(dr_data.ori_w);
    pose_->mutable_orientation()->set_qx(dr_data.ori_x);
    pose_->mutable_orientation()->set_qy(dr_data.ori_y);
    pose_->mutable_orientation()->set_qz(dr_data.ori_z);
    pose_->mutable_angular_velocity()->set_x(dr_data.gyro_x);
    pose_->mutable_angular_velocity()->set_y(dr_data.gyro_y);
    pose_->mutable_angular_velocity()->set_z(dr_data.gyro_z);
    pose_->mutable_linear_acceleration()->set_x(dr_data.acc_x);
    pose_->mutable_linear_acceleration()->set_y(dr_data.acc_y);
    pose_->mutable_linear_acceleration()->set_z(dr_data.acc_z);
    pose_->mutable_linear_velocity()->set_x(dr_data.vel_x);
    pose_->mutable_linear_velocity()->set_y(dr_data.vel_y);
    pose_->mutable_linear_velocity()->set_z(dr_data.vel_z);
    pose_->set_heading(dr_data.heading);
    dr_result_writer_->Write(
        std::make_shared<LocalizationEstimate>(*dr_result_msg_));
    {
      module_status_msg_->Clear();
      auto header_ = module_status_msg_->mutable_header();
      header_->set_module_name("DR");
      header_->set_measurement_timestamp(dr_data.timestamp);
      header_->set_publish_timestamp(apollo::cyber::Time::Now().ToSecond());
      header_->set_sequence_num(sequence_num);
      uint64_t fault_code = 0;
      if (dr_data.imu_state == SenSta::WARNNING) {
        fault_code |= 0x1;
      } else if (dr_data.imu_state == SenSta::ERROR ||
                 dr_data.imu_state == SenSta::FATAL) {
        fault_code |= 0x2;
      }
      if (dr_data.veh_state == SenSta::WARNNING) {
        fault_code |= 0x4;
      } else if (dr_data.veh_state == SenSta::ERROR ||
                 dr_data.veh_state == SenSta::FATAL) {
        fault_code |= 0x8;
      }
      module_status_msg_->set_fault_code(fault_code);
      module_status_writer_->Write(
          std::make_shared<ModuleStatus>(*module_status_msg_));
    }
    sequence_num++;
  };
}

void DRComponent::parse_config(const std::string& config_file_path) {
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file_path);
  } catch (YAML::BadFile& e) {
    AERROR << "[config error] filter config file read error!";
    return;
  }
  try {
    enable_dr_daemon = config["enable_dr_daemon"].as<bool>();
  } catch (...) {
    AERROR << "[config error] key 'enable_dr_daemon' parse error!";
  }
  try {
    enable_msf_info = config["enable_msf_info"].as<bool>();
  } catch (...) {
    AERROR << "[config error] key 'enable_msf_info' parse error!";
  }
  try {
    rt_priority_ = config["rt_priority"].as<int>();
  } catch (...) {
    AERROR << "[config error] key 'rt_priority' parse error!";
  }
  try {
    rt_priority_delay_s_ = config["rt_priority_delay_s"].as<int>();
  } catch (...) {
    AERROR << "[config error] key 'rt_priority_delay_s' parse error!";
  }
  AINFO << "enable_dr_daemon: " << enable_dr_daemon << ", "
        << "enable_msf_info: " << enable_msf_info << ", "
        << "rt_priority: " << rt_priority_ << ", "
        << "rt_priority_delay_s: " << rt_priority_delay_s_;
}
}  // namespace dr
}  // namespace byd
