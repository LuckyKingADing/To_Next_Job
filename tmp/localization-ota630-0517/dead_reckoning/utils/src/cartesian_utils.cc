#include "cartesian_utils.h"

namespace byd {
namespace dr {

CoordinateTransformation::CoordinateTransformation() {
  NED2ENU << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
  RFU2FRD << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
  FLU2FRD << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
  FLU2RFU << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
}

Eigen::Matrix3d CoordinateTransformation::euler2matrix(
    const Eigen::Vector3d &euler) {
  Eigen::AngleAxisd X(euler(0), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Y(euler(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Z(euler(2), Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d mat;
  mat = Z * Y * X;
  return mat;
}

Eigen::Quaterniond CoordinateTransformation::euler2quaternion(
    const Eigen::Vector3d &euler) {
  Eigen::AngleAxisd X(euler(0), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Y(euler(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Z(euler(2), Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q;
  q = Z * Y * X;
  return q;
}

Eigen::Vector3d CoordinateTransformation::matrix2euler(
    const Eigen::Matrix3d &mat) {
  Eigen::Quaterniond q;
  q = mat;
  return quaternion2euler(q);
}

Eigen::Vector3d CoordinateTransformation::quaternion2euler(
    const Eigen::Quaterniond &q) {
  Eigen::Vector3d angles;

  // (x-axis rotation)
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  angles(2) = std::atan2(sinr_cosp, cosr_cosp);

  // (y-axis rotation)
  double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1)
    angles(1) =
        std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    angles(1) = std::asin(sinp);

  // (z-axis rotation)
  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  angles(0) = std::atan2(siny_cosp, cosy_cosp);

  return angles.reverse();
}

void CoordinateTransformation::pboxAtt_from_FRD2NED_to_FLU2ENU(
    const Eigen::Vector3d &eul_, Eigen::Vector3d &eul, Eigen::Quaterniond &q) {
  auto mat = euler2matrix(eul_);
  //       ENU    NED    FRD      RFU
  //    ( C    * C    * C    ) * C
  //       NED    FRD    RFU      FLU
  q = (NED2ENU * mat * RFU2FRD) * FLU2RFU;
  eul = quaternion2euler(q);
}

void CoordinateTransformation::pboxAtt_from_FRD2NED_to_RFU2ENU(
    const Eigen::Vector3d &eul_, Eigen::Vector3d &eul, Eigen::Quaterniond &q) {
  auto mat = euler2matrix(eul_);
  //      ENU    NED    FRD
  //     C    * C    * C
  //      NED    FRD    RFU
  q = NED2ENU * mat * RFU2FRD;
  eul = quaternion2euler(q);
}

/* 从四元数得到Yaw角
 * Yaw定义：(-pi,pi)，指北为零，东为正
 * 说明：FRD相对于导航坐标系NED的heading
 * heading指的车头方向
 */
void CoordinateTransformation::quaternion_from_RFU2ENU_to_Yaw_NE(
    const Eigen::Quaterniond &quat_RFU2ENU, double *yaw) {
  Eigen::Quaterniond quat_FRD2NED;
  quaternion_from_RFU2ENU_to_FRD2NED(quat_RFU2ENU, quat_FRD2NED);
  auto euler = quaternion2euler(quat_FRD2NED);
  *yaw = euler.z();
}

/* 从四元数得到Yaw角
 * Yaw定义：(-pi,pi)，指东为零，北为正
 * 说明：车身坐标系FLU相对于导航坐标系ENU的heading
 * heading指的车头方向
 */
void CoordinateTransformation::quaternion_from_RFU2ENU_to_Yaw_EN(
    const Eigen::Quaterniond &quat_RFU2ENU, double *yaw) {
  Eigen::Quaterniond quat_FLU2ENU;
  quaternion_from_RFU2ENU_to_FLU2ENU(quat_RFU2ENU, quat_FLU2ENU);
  auto euler = quaternion2euler(quat_FLU2ENU);
  *yaw = euler.z();
}

/*
 * 四元数转换：从RFU2ENU到FLU2NED
 */
void CoordinateTransformation::quaternion_from_RFU2ENU_to_FLU2NED(
    const Eigen::Quaterniond &quat_RFU2ENU, Eigen::Quaterniond &quat_FLU2NED) {
  auto mat_RFU2ENU = quat_RFU2ENU.toRotationMatrix();
  auto mat_FLU2NED = NED2ENU.transpose() * mat_RFU2ENU * FLU2RFU;
  quat_FLU2NED = mat_FLU2NED;
}

/*
 * 四元数转换：从RFU2ENU到FRD2NED
 */
void CoordinateTransformation::quaternion_from_RFU2ENU_to_FRD2NED(
    const Eigen::Quaterniond &quat_RFU2ENU, Eigen::Quaterniond &quat_FRD2NED) {
  auto mat_RFU2ENU = quat_RFU2ENU.toRotationMatrix();
  auto mat_FRD2NED =
      (NED2ENU.transpose()) * mat_RFU2ENU * (RFU2FRD.transpose());
  quat_FRD2NED = mat_FRD2NED;
}

/*
 * 四元数转换：从RFU2ENU到FLU2ENU
 */
void CoordinateTransformation::quaternion_from_RFU2ENU_to_FLU2ENU(
    const Eigen::Quaterniond &quat_RFU2ENU, Eigen::Quaterniond &quat_FLU2ENU) {
  auto mat_RFU2ENU = quat_RFU2ENU.toRotationMatrix();
  auto mat_FLU2ENU = mat_RFU2ENU * FLU2RFU;
  quat_FLU2ENU = mat_FLU2ENU;
}
}  // namespace dr
}  // namespace byd