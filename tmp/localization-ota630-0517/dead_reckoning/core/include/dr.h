#include <functional>
#include <mutex>

#include "data_type.h"
#include "dr_core.h"
#include "dr_interface.h"
#include "filter.h"
#include "time_trigger.h"

#define RTK_FIX 6

#define IMU_MSG_EXPECTED_GAP 0.01
#define VEH_MSG_EXPECTED_GAP 0.01

namespace byd {
namespace dr {

using namespace byd::dr_data;

using DR = class DEAD_RECKONING : public DR_interface {
 private:
  typedef struct STATES {
    STATE_TIMER rtk_good;
    STATE_TIMER msf_good;
  } States;

  States states;

 private:
  std::mutex data_mutex;
  GpsData gps_data;
  ImuData imu_data;
  VehData veh_data;
  MsfData msf_data;
  DrData dr_data;

 private:
  STATE_TIMER gps_msg_gap_timer;
  STATE_TIMER imu_msg_gap_timer;
  STATE_TIMER veh_msg_gap_timer;
  STATE_TIMER msf_msg_gap_timer;

 public:
  DEAD_RECKONING() : drdaemon_t_stop_sig(false) {
    gps_msg_gap_timer.reset();
    imu_msg_gap_timer.reset();
    veh_msg_gap_timer.reset();
    msf_msg_gap_timer.reset();
  }
  // DEAD_RECKONING(const Eigen::Matrix<double, 9, 1>& data) {
  //   vdsa.cfg.set_static_bound(data);
  // }

 private:
  std::thread drdaemon_t;

 private:
  std::atomic<bool> drdaemon_t_stop_sig;

 public:
  virtual void stop_drdaemon() override final;

 public:
  dr_core::DR_CORE::Adaption adaption;
  std::array<double, 6UL> lpfout{0};

 private:
  dr_core::DR_CORE dr_core;

 private:
  STATE_TIMER dr_timer;

 public:
  virtual void insert_imu(const ImuData &) override final;
  virtual void insert_veh(const VehData &) override final;
  virtual void insert_gps(const GpsData &) override final;
  virtual void insert_msf(const MsfData &) override final;

 public:
  virtual void start_dr_daemon(std::function<void(const DrData &)> cb,
      double dt, int priority, int delay_s) override final;

 private:
  void set_dr_daemon_prior(int priority, int delay_s);

 public:
  virtual void dr_step(double dt) override final;
  virtual DrData get_result() override final;

 public:
  virtual void set_static_bound(
      const Eigen::Matrix<double, 9, 1> &static_bound) override final;

 private:
  VLPF<6> vlpf;
  const int VDSA_SKIP = 4;
  const int DA_SKIP = 50;
  uint64_t step_count = 0;

 public:
  VDSA vdsa;
  DA da;
  AS as_lon, as_lat;
};
}  // namespace dr
}  // namespace byd
