#pragma once

#include <Eigen/Dense>
#include <memory>

namespace analysis {

// DR
struct KinematicData {
    double             measurement_timestamp; // In second.
    Eigen::Vector3d    vel;
    Eigen::Vector3d    pos;
    Eigen::Quaterniond att;
    double             ego_longitude_vel;
    KinematicData() {
        measurement_timestamp = 0.0;
        vel                   = Eigen::Vector3d::Zero();
        pos                   = Eigen::Vector3d::Zero();
        att                   = Eigen::Quaterniond::Identity();
        ego_longitude_vel     = 0.0;
    }
    KinematicData operator-(const KinematicData &motion_) {
        // 约定减法表示增量
        // B-A => A到B的增量
        // 姿态上：B-A => C_A2B = B*A^-1
        KinematicData rslt;
        rslt.measurement_timestamp = this->measurement_timestamp - motion_.measurement_timestamp;
        rslt.vel                   = this->vel - motion_.vel;
        rslt.pos                   = this->pos - motion_.pos;
        rslt.ego_longitude_vel     = this->ego_longitude_vel - motion_.ego_longitude_vel;
        rslt.att                   = this->att * motion_.att.conjugate();
        return rslt;
    }
};

using KinematicDataPtr = std::shared_ptr<KinematicData>;

} // namespace analysis