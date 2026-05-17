#pragma once
#include "Eigen/Dense"
#include "fail_detect_interface.h"
#include "local_trans.h"
#include <deque>

namespace byd {
namespace tcmsf {
namespace fail_detect {

struct DRINFO {
    Eigen::Vector3d pos;
    DRINFO() {
        pos.setZero();
    }
};

struct TCMSFINFO {
    Eigen::Vector3d blh;
    double          speed;
    uint64_t        rtk_status;
    TCMSFINFO() {
        blh.setZero();
        speed      = 0.0;
        rtk_status = 0;
    }
};

struct IMUINFO {
    Eigen::Vector3d gyro;
    Eigen::Vector3d acc;
    IMUINFO() {
        gyro.setZero();
        acc.setZero();
    }
};

struct kinematicsINFO {
    double    timestamp;
    DRINFO    dr;
    TCMSFINFO tcmsf;
    IMUINFO   imu;
    kinematicsINFO() {
        timestamp = 0.0;
    }
};

class IMUFailDetectImpl : public IMUFailDetect {
private:
    static constexpr uint64_t MAXIMUM_DEQUE_SIZE        = 100;
    static constexpr double   MINIMUM_SAMPLING_INTERVAL = 0.015;

private:
    std::deque<kinematicsINFO> info_deque_;

private:
    template <typename T>
    void insert(T msg_, std::deque<T> &deque_, double dt_) {
        if (deque_.size() == 0) {
            deque_.push_back(msg_);
        } else {
            if (std::abs(dt_) > MINIMUM_SAMPLING_INTERVAL) {
                deque_.push_back(msg_);
            }
        }
        while (deque_.size() > MAXIMUM_DEQUE_SIZE) {
            deque_.pop_front();
        }
    }

private:
    byd::geo::LocalTrans local_trans_;

public:
    virtual void insert_kinematics_info(const MSF::KinematicDataPtr, const MSF::StatePtr, const MSF::ImuDataPtr) override final;
    virtual bool calc_gyro_scale_z(double &delta_heading_ref_, double &delta_heading_, double &scale_) override final;
    virtual bool imu_fatal_error_detect(bool &is_imu_fail_) override final;
};
} // namespace fail_detect
} // namespace tcmsf
} // namespace byd