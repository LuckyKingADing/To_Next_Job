#pragma once

#include <functional>
#include <memory>

#include "data_type.h"

namespace byd {
namespace dr {

using namespace byd::dr_data;

class DR_interface {
 public:

  virtual ~DR_interface() = default; //adding a virtual destructor
  virtual void insert_imu(const ImuData &) = 0;
  virtual void insert_veh(const VehData &) = 0;
  virtual void insert_gps(const GpsData &) = 0;
  virtual void insert_msf(const MsfData &) = 0;

 public:
  virtual void start_dr_daemon(std::function<void(const DrData &)> cb,
                               double dt, int priority, int delay_s) = 0;
  virtual void stop_drdaemon() = 0;

 public:
  virtual void dr_step(double dt) = 0;
  virtual DrData get_result() = 0;

 public:
  virtual void set_static_bound(
      const Eigen::Matrix<double, 9, 1> &static_bound) = 0;

 public:
  static std::unique_ptr<DR_interface> create();
};
}  // namespace dr
}  // namespace byd
