#include "troublemaker_impl.h"
#include "rigid_transform.h"

namespace TroubleMaker {

SensorModifyImpl::SensorModifyImpl() {

    Eigen::Vector3d imu_misalign_{-0.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI, -0.0 / 180.0 * M_PI};

    gyro_mod_ = Modification(imu_misalign_, Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{1.0, 1.0, 1.0});
    acc_mod_  = Modification(imu_misalign_, Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{1.0, 1.0, 1.0});
    gnss_mod_ = Modification(Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{1.0, 1.0, 1.0});
    veh_mod_  = Modification(Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{1.0, 1.0, 1.0});
    vis_mod_  = Modification(Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{1.0, 1.0, 1.0});
}

Modification::Modification() {
    Rotate.setIdentity();
    Move.setZero();
    Scale.setOnes();
}
Modification::Modification(Eigen::Vector3d r_xyz, Eigen::Vector3d m_xyz, Eigen::Vector3d s_xyz) {
    Rotate = INS::euler2quaternion(r_xyz).toRotationMatrix();
    Move   = m_xyz;
    Scale  = s_xyz;
}

bool SensorModifyImpl::sensor_modify(ImuData &imu_data_) {
    gyro_mod_(imu_data_.gyro);
    acc_mod_(imu_data_.acc);
    return true;
}
bool SensorModifyImpl::sensor_modify(GnssData &) {
    return true;
}
bool SensorModifyImpl::sensor_modify(VehicleData &) {
    return true;
}
bool SensorModifyImpl::sensor_modify(VisionFusionData &) {
    return true;
}

} // namespace TroubleMaker