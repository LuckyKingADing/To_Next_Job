#pragma once

#include "imu_issue_fallback_interface.h"

namespace MSF::IIF {

class ImuIssueFallbackImpl : public ImuIssueFallbackInterface {

private:
    std::shared_ptr<Imu>     pre_imu_msg = std::make_shared<Imu>();
    std::shared_ptr<VehInfo> cur_veh_msg = std::make_shared<VehInfo>();

private:
    constexpr static double GYRO_SUDDEN_JUMP_BOUND_                 = 4000.0;
    constexpr static double GYRO_SUDDEN_JUMP_LIKELY_DUARTION_BOUND_ = 0.1;

public:
    virtual std::shared_ptr<Imu> insert_imu(const std::shared_ptr<Imu>) override final;
    virtual int                  insert_veh(const std::shared_ptr<VehInfo>) override final;

private:
    double imu_gyro_issue_measurement_timestamp = 0.0;
    void   imu_gyro_sudden_jump_probe(const std::shared_ptr<Imu>);
};

} // namespace MSF::IIF