#pragma once

#include "troublemaker_interface.h"

namespace TroubleMaker {

class Modification {
private:
    Eigen::Matrix3d Rotate;
    Eigen::Vector3d Move;
    Eigen::Vector3d Scale;

public:
    Modification();
    Modification(const Eigen::Vector3d, const Eigen::Vector3d, const Eigen::Vector3d);

public:
    void operator()(Eigen::Vector3d &vec) {
        vec = vec.eval().array() * Scale.array();
        vec = Rotate * vec.eval();
        vec = vec.eval() + Move;
    }
};

class SensorModifyImpl : public SensorModifyBase {

private:
    Modification gyro_mod_;
    Modification acc_mod_;
    Modification gnss_mod_;
    Modification veh_mod_;
    Modification vis_mod_;

public:
    SensorModifyImpl();

public:
    virtual bool sensor_modify(ImuData &) override final;
    virtual bool sensor_modify(GnssData &) override final;
    virtual bool sensor_modify(VehicleData &) override final;
    virtual bool sensor_modify(VisionFusionData &) override final;
};
} // namespace TroubleMaker