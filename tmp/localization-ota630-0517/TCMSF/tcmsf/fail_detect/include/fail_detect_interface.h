#pragma once

#include "base_type.h"
#include "cyber/common/log.h"
#include "state_type.h"
#include <memory>

namespace byd {
namespace tcmsf {
namespace fail_detect {

class IMUFailDetect {
public:
    virtual ~IMUFailDetect() {
        AINFO << "exit IMUFailDetect interface";
    };

public:
    static std::unique_ptr<IMUFailDetect> create();

public:
    virtual void insert_kinematics_info(const MSF::KinematicDataPtr, const MSF::StatePtr, const MSF::ImuDataPtr) = 0;
    virtual bool calc_gyro_scale_z(double &delta_heading_ref_, double &delta_heading_, double &scale_)           = 0;
    virtual bool imu_fatal_error_detect(bool &is_imu_fail_)                                                      = 0;
};
} // namespace fail_detect
} // namespace tcmsf
} // namespace byd
