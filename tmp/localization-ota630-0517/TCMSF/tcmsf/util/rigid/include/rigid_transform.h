#pragma once

#include "Eigen/Eigen"
#include "Eigen/Geometry"

namespace INS {

/// Returns the 3D cross product Skew symmetric matrix of a given 3D vector.
template <class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Skew(const Eigen::MatrixBase<Derived> &vec) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
}

inline Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond &q) {
    Eigen::Vector3d angles;

    // (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(2)        = std::atan2(sinr_cosp, cosr_cosp);

    // (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles(1) = std::asin(sinp);

    // (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(0)        = std::atan2(siny_cosp, cosy_cosp);

    return angles.reverse();
}

inline Eigen::Quaterniond euler2quaternion(const Eigen::Vector3d euler) {
    Eigen::Quaterniond Q =
        Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX());
    return Q;
}

inline Eigen::Quaterniond rv2q(const Eigen::Vector3d &rv) {
    double          n  = rv.norm();
    double          n2 = n * n;
    double          w = 0.0, s = 0.0;
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    if (n < 1e-4) {
        w = 1 - n2 * (1.0 / 8.0 - n2 / 384.0);
        s = 1.0 / 2.0 - n2 * (1.0 / 48.0 - n2 / 3840.0);
    } else {
        w = cos(n / 2);
        s = sin(n / 2) / n;
    }
    v = s * rv;
    Eigen::Quaterniond q(w, v.x(), v.y(), v.z());
    return q;
}

inline Eigen::Matrix3d rv2m(const Eigen::Vector3d &rv) {
    Eigen::Matrix3d M;
    M = rv2q(rv);
    return M;
}

struct FrameTrans {
public:
    const Eigen::Matrix3d NED2ENU = (Eigen::Matrix3d() << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0).finished();
    const Eigen::Matrix3d RFU2FRD = (Eigen::Matrix3d() << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0).finished();
    const Eigen::Matrix3d FLU2FRD = (Eigen::Matrix3d() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0).finished();
    const Eigen::Matrix3d FLU2RFU = (Eigen::Matrix3d() << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
} static frame_trans;

} // namespace INS