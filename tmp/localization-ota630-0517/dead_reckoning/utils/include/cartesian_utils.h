#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace byd {
namespace dr {

/*
 * 约定：
 *       1. 欧拉角XYZ范围
 *         X -> (  -pi,   pi)
 *         Y -> (-pi/2, pi/2)
 *         Z -> (  -pi,   pi)
 *       2. 角度使用弧度制
 *
 * 坐标系定义：
 *       1. 车体坐标系为 FLU（原点为车体后轮轴中心）
 *       2. 本地导航坐标系为 ENU （原点为车体后轮轴中心）
 *       3. 地图坐标系为 UTM （x轴指东，y轴指北）
 *       4. PBOX使用的坐标系 FRD
 *
 * 目前0409车PBOX输出坐标系：
 *       1. IMU输出为 FRD（原点为PBOX重心）
 *       2. INS输出为 FRD（原点为车体后轮轴中心）
 */
class CoordinateTransformation {
 public:
  CoordinateTransformation();

 public:
  Eigen::Matrix3d NED2ENU;
  Eigen::Matrix3d RFU2FRD;
  Eigen::Matrix3d FLU2FRD;
  Eigen::Matrix3d FLU2RFU;

 public:
  const double ARC_TO_DEG = 180.0 / M_PI;
  const double DEG_TO_ARC = M_PI / 180.0;

 public:
  /* 输入：欧拉角 [x,y,z]
   * 输出：旋转矩阵
   * 说明：旋转先后顺序为 x -> y -> z
   */
  Eigen::Matrix3d euler2matrix(const Eigen::Vector3d &euler);

  /* 输入：欧拉角 [x,y,z]
   * 输出：四元数
   * 说明：旋转先后顺序为 x -> y -> z
   */
  Eigen::Quaterniond euler2quaternion(const Eigen::Vector3d &euler);

  /* 输入：旋转矩阵
   * 输出：欧拉角 [x,y,z]
   * 说明：旋转先后顺序为 x -> y -> z
   * 因为一个旋转矩阵对应两个同序欧拉角，
   * 此处做一个额外约束，即欧拉角Y范围控制为(-pi/2,pi/2)
   */
  Eigen::Vector3d matrix2euler(const Eigen::Matrix3d &mat);

 public:
  /*
   * 输入：四元数
   * 输出：欧拉角 [x,y,z]
   * 说明：旋转先后顺序为 x -> y -> z
   * 因为一个旋转矩阵对应两个同序欧拉角，
   * 此处做一个额外约束，即欧拉角Y范围控制为(-pi/2,pi/2)
   */
  Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond &q);

 public:
  /* 初始欧拉角： eul_ [x, y, z]
   * 目前pbox输出的欧拉角表示FRD相对于NED的姿态，
   * 需要转换为FLU相对于ENU的姿态
   * 转换后欧拉角：eul [x, y, z]
   * 相应的四元数：q [w, x, y, z]
   */
  void pboxAtt_from_FRD2NED_to_FLU2ENU(const Eigen::Vector3d &eul_,
                                       Eigen::Vector3d &eul,
                                       Eigen::Quaterniond &q);

  /* 初始欧拉角： eul_ [x, y, z]
   * 目前pbox输出的欧拉角表示FRD相对于NED的姿态，
   * 需要转换为RFU相对于ENU的姿态
   * 转换后欧拉角：eul [x, y, z]
   * 相应的四元数：q [w, x, y, z]
   */
  void pboxAtt_from_FRD2NED_to_RFU2ENU(const Eigen::Vector3d &eul_,
                                       Eigen::Vector3d &eul,
                                       Eigen::Quaterniond &q);

  /* 从四元数得到Yaw角
   * Yaw定义：(-pi,pi)，指北为零，东为正
   */
  void quaternion_from_RFU2ENU_to_Yaw_NE(const Eigen::Quaterniond &quat_RFU2ENU,
                                         double *yaw);

  /* 从四元数得到Yaw角
   * Yaw定义：(-pi,pi)，指东为零，北为正
   */
  void quaternion_from_RFU2ENU_to_Yaw_EN(const Eigen::Quaterniond &quat_RFU2ENU,
                                         double *yaw);

  /*
   * 四元数转换：从RFU2ENU到FLU2NED
   */
  void quaternion_from_RFU2ENU_to_FLU2NED(
      const Eigen::Quaterniond &quat_RFU2ENU, Eigen::Quaterniond &quat_FLU2NED);

  /*
   * 四元数转换：从RFU2ENU到FRD2NED
   */
  void quaternion_from_RFU2ENU_to_FRD2NED(
      const Eigen::Quaterniond &quat_RFU2ENU, Eigen::Quaterniond &quat_FRD2NED);

  /*
   * 四元数转换：从RFU2ENU到FLU2ENU
   */
  void quaternion_from_RFU2ENU_to_FLU2ENU(
      const Eigen::Quaterniond &quat_RFU2ENU, Eigen::Quaterniond &quat_FLU2ENU);
};
}  // namespace dr
}  // namespace byd