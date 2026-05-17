#include "dr.h"

#include <sched.h>
#include <sys/resource.h>

#include "cyber/time/time.h"
#include "cyber/task/task.h"
namespace byd {
namespace dr {

void DEAD_RECKONING::insert_imu(const ImuData& imu_data_) {
  std::unique_lock<std::mutex> lock(data_mutex);
  imu_msg_gap_timer.reset();
  this->imu_data = imu_data_;
}

void DEAD_RECKONING::insert_gps(const GpsData& gps_data_) {
  std::unique_lock<std::mutex> lock(data_mutex);
  gps_msg_gap_timer.reset();
  this->gps_data = gps_data_;
}

void DEAD_RECKONING::insert_veh(const VehData& veh_data_) {
  std::unique_lock<std::mutex> lock(data_mutex);
  veh_msg_gap_timer.reset();
  this->veh_data = veh_data_;
}

void DEAD_RECKONING::insert_msf(const MsfData& msf_data_) {
  std::unique_lock<std::mutex> lock(data_mutex);
  msf_msg_gap_timer.reset();
  this->msf_data = msf_data_;
}

void DEAD_RECKONING::stop_drdaemon() {
  drdaemon_t_stop_sig.store(true);
  if (drdaemon_t.joinable()) {
    drdaemon_t.join();
  }
}

void DEAD_RECKONING::dr_step(double dt) {
  std::unique_lock<std::mutex> lock(data_mutex);

  dr_core.measurement_time_update(apollo::cyber::Time::Now().ToSecond());

  if (gps_data.status != RTK_FIX || msf_msg_gap_timer.dt() > 0.1 ||
      gps_msg_gap_timer.dt() > 1.0) {
    states.rtk_good.reset();
  }
  // std::cout << "msf dt: " << msf_msg_gap_timer.dt() << ", "
  // << "rtk good: " << states.rtk_good.dt() << std::endl;
  // if (msf_data.rtk_state != RTK_FIX) {
  //   states.rtk_good.reset();
  // }

  double spd_ = (veh_data.spd_rl + veh_data.spd_rr) / 2.0;

  double acc_lon_sm = as_lon(imu_data.acc_y);
  double acc_lat_sm = as_lat(imu_data.acc_x);

  // adaptions ~ compesation
  lpfout = vlpf({imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
                 imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z});
  // lpfout = {imu_data.acc_x,  imu_data.acc_y,  imu_data.acc_z,
  // imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z};

  if (step_count % VDSA_SKIP == 0 && imu_msg_gap_timer.dt() < 0.1) {
    auto vdsa_result = vdsa({0, spd_, 0}, {lpfout[0], lpfout[1], lpfout[2]},
                            {lpfout[3], lpfout[4], lpfout[5]}, dt * VDSA_SKIP,
                            acc_lon_sm, acc_lat_sm);
    if (vdsa_result.zupt) {
      if (adaption.bias_gyro.norm() < std::numeric_limits<double>::epsilon()) {
        adaption.bias_gyro = vdsa.gyro_bias;
      } else {
        adaption.bias_gyro = 0.99 * adaption.bias_gyro + 0.01 * vdsa.gyro_bias;
      }
      // std::cout << "gyro bias: " << adaption.bias_gyro.x() * 180 / M_PI <<
      // "\t"
      //           << adaption.bias_gyro.y() * 180 / M_PI << "\t"
      //           << adaption.bias_gyro.z() * 180 / M_PI << "\t"
      //           << adaption.veh_spd_factor << std::endl;
    }
    if (vdsa_result.lat_ac) {
      adaption.latitudinal_acc = vdsa.latitudinal_acc;
    }
    if (vdsa_result.lon_ac) {
      adaption.longitudinal_acc = vdsa.longitudinal_acc;
    }
    if (vdsa_result.alp_a) {
      adaption.alpha_factor = vdsa.alpha_factor;
    }
  }

  if (step_count % DA_SKIP == 0 && msf_msg_gap_timer.dt() < 0.1) {
    // std::cout << states.rtk_good.dt() << std::endl;
    auto da_result =
        da(dr_data, msf_data, spd_, dt * DA_SKIP, states.rtk_good.dt());
    // auto da_result = da(dr_data, msf_data, spd_, dt * DA_SKIP, 30.0);
    if (da_result.spd_sa) {
      if (std::abs(adaption.veh_spd_factor - 1.0) <=
          std::numeric_limits<double>::epsilon()) {
        adaption.veh_spd_factor = da.whl_spd_factor;
      } else {
        adaption.veh_spd_factor =
            0.99 * adaption.veh_spd_factor + 0.01 * da.whl_spd_factor;
      }
    }
    // if (da_result.yaw_ba) {
    //   adaption.bias_gyro.z() += 0.02 * da.yaw_bias;
    //   {  // bias estimate safe bound
    //     adaption.bias_gyro.z() = adaption.bias_gyro.z() > 0.08 / 180 * M_PI
    //                                  ? 0.08 / 180 * M_PI
    //                                  : adaption.bias_gyro.z();
    //     adaption.bias_gyro.z() = adaption.bias_gyro.z() < -0.08 / 180 * M_PI
    //                                  ? -0.08 / 180 * M_PI
    //                                  : adaption.bias_gyro.z();
    //   }
    //   std::cout << "gyro z bias: " << adaption.bias_gyro.z() * 180 / M_PI
    //             << ", " << da.yaw_bias * 180 / M_PI << ","
    //             << adaption.veh_spd_factor << std::endl;
    // }

    if (da_result.imu_bias) {
      // adaption.bias_acc = 0.9 * adaption.bias_acc + 0.1 * da.acc_bias;
      adaption.bias_gyro = 0.9 * adaption.bias_gyro + 0.1 * da.gyro_bias;
      {  // bias estimate safe bound
        adaption.bias_gyro.x() = adaption.bias_gyro.x() > 0.08 / 180 * M_PI
                                     ? 0.08 / 180 * M_PI
                                     : adaption.bias_gyro.x();
        adaption.bias_gyro.x() = adaption.bias_gyro.x() < -0.08 / 180 * M_PI
                                     ? -0.08 / 180 * M_PI
                                     : adaption.bias_gyro.x();

        adaption.bias_gyro.y() = adaption.bias_gyro.y() > 0.08 / 180 * M_PI
                                     ? 0.08 / 180 * M_PI
                                     : adaption.bias_gyro.y();
        adaption.bias_gyro.y() = adaption.bias_gyro.y() < -0.08 / 180 * M_PI
                                     ? -0.08 / 180 * M_PI
                                     : adaption.bias_gyro.y();

        adaption.bias_gyro.z() = adaption.bias_gyro.z() > 0.2 / 180 * M_PI
                                     ? 0.2 / 180 * M_PI
                                     : adaption.bias_gyro.z();
        adaption.bias_gyro.z() = adaption.bias_gyro.z() < -0.2 / 180 * M_PI
                                     ? -0.2 / 180 * M_PI
                                     : adaption.bias_gyro.z();
      }
      // std::cout << "gyro bias: " << adaption.bias_gyro.x() * 180 / M_PI <<
      // "\t"
      //           << adaption.bias_gyro.y() * 180 / M_PI << "\t"
      //           << adaption.bias_gyro.z() * 180 / M_PI << "\t"
      //           << adaption.veh_spd_factor << std::endl;
    }
  }
  dr_core.adaption_update(adaption);

  dr_core.dt_update(dt);
  if (imu_msg_gap_timer.dt() < IMU_MSG_EXPECTED_GAP * 100) {
    dr_core.attitude_update(
        {imu_data.acc_x, imu_data.acc_y, imu_data.acc_z},
        {imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z});
  } else {
    dr_core.attitude_update({0.0, 0.0, 1.0}, {0.0, 0.0, veh_data.yaw_rate});
  }

  dr_core.position_update({0, spd_, 0});

  this->dr_data = dr_core.get_dr_result();

  {
    double msg_gap_imu_dt = imu_msg_gap_timer.dt();
    double msg_gap_veh_dt = veh_msg_gap_timer.dt();
    if (msg_gap_imu_dt <= IMU_MSG_EXPECTED_GAP * 10) {
      this->dr_data.imu_state = SenSta::OK;
    } else if (msg_gap_imu_dt > IMU_MSG_EXPECTED_GAP * 10 &&
               msg_gap_imu_dt <= IMU_MSG_EXPECTED_GAP * 100) {
      this->dr_data.imu_state = SenSta::WARNNING;
    } else if (msg_gap_imu_dt > IMU_MSG_EXPECTED_GAP * 100 &&
               msg_gap_imu_dt <= IMU_MSG_EXPECTED_GAP * 1000) {
      this->dr_data.imu_state = SenSta::ERROR;
    } else if (msg_gap_imu_dt > IMU_MSG_EXPECTED_GAP * 1000) {
      this->dr_data.imu_state = SenSta::FATAL;
    }
    if (msg_gap_veh_dt <= VEH_MSG_EXPECTED_GAP * 10) {
      this->dr_data.veh_state = SenSta::OK;
    } else if (msg_gap_veh_dt > VEH_MSG_EXPECTED_GAP * 10 &&
               msg_gap_veh_dt <= VEH_MSG_EXPECTED_GAP * 100) {
      this->dr_data.veh_state = SenSta::WARNNING;
    } else if (msg_gap_veh_dt > VEH_MSG_EXPECTED_GAP * 100 &&
               msg_gap_veh_dt <= VEH_MSG_EXPECTED_GAP * 1000) {
      this->dr_data.veh_state = SenSta::ERROR;
    } else if (msg_gap_veh_dt > VEH_MSG_EXPECTED_GAP * 1000) {
      this->dr_data.veh_state = SenSta::FATAL;
    }
  }

  step_count++;
}

void DEAD_RECKONING::start_dr_daemon(std::function<void(const DrData&)> cb,
                                     double step_dt,
                                     int priority, int delay_s) {
  drdaemon_t = std::thread([this, cb, step_dt]() {
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    // int policy;
    // sched_param sch;
    // pthread_getschedparam(pthread_self(), &policy, &sch);
    // printf("sched policy %d, thread priority %d\n", policy,
    // sch.sched_priority);

    STATE_TIMER dr_timer;
    while (true) {
      if (drdaemon_t_stop_sig.load()) {
        break;
      }
      auto dt = dr_timer.dt();
      dr_timer.reset();
      dr_step(dt);
      cb(dr_data);
      // { // 这个代码段的AWARN LOG可能导致DR卡顿，删除。
      //   auto dr_step_cost = dr_timer.dt();
      //   // log unusual time consumption cases of [DR step]
      //   if (dt > 0.03 || dt < 0.005) AWARN << "unusual dr step gap: " << dt;
      //   if (dr_step_cost > 0.01)
      //     AWARN << "unusual dr step cost: " << dr_step_cost;
      // }
      auto t_ = step_dt - dr_timer.dt();
      if (t_ > 0 && t_ <= step_dt) {
        std::this_thread::sleep_for(std::chrono::microseconds(int(t_ * 1e6)));
      } else {
        std::this_thread::sleep_for(std::chrono::microseconds(2000));
      }
      // if (t_ > 0.002) {
      //   std::this_thread::sleep_for(
      //       std::chrono::microseconds(int(t_ / 10.0 * 9.0 * 1e6)));
      // }
      // while (step_dt - dr_timer.dt() > 0) {
      // }
    }
  });
  // drdaemon_t.detach();
  apollo::cyber::Async(&DEAD_RECKONING::set_dr_daemon_prior, this, priority, delay_s);
  apollo::cyber::scheduler::Instance()->SetInnerThreadAttr("drdaemon_rt", &drdaemon_t);
}

void DEAD_RECKONING::set_dr_daemon_prior(int priority, int delay_s) {
  // sleep to avoid blocked in RWLock while initializing
  usleep(delay_s * 1000000);

  // set rt priority
  struct sched_param sp;
  memset(reinterpret_cast<void*>(&sp), 0, sizeof(sp));
  sp.sched_priority = priority;
  if(drdaemon_t_stop_sig.load()){
    return;
  }
  auto sched_result =
      pthread_setschedparam(drdaemon_t.native_handle(), SCHED_RR, &sp);
  if (sched_result) {
    AWARN << "DR failed to set real-time scheduling, error code: "
          << sched_result;
  } else {
    AINFO << "DR set real-time scheduling success.";
  }
}

DrData DEAD_RECKONING::get_result() {
  std::unique_lock<std::mutex> lock(data_mutex);
  return dr_data;
}

void DEAD_RECKONING::set_static_bound(
    const Eigen::Matrix<double, 9, 1>& static_bound) {
  this->vdsa.cfg.set_static_bound(static_bound);
}

}  // namespace dr
}  // namespace byd
