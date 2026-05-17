#include "base_type.h"
#include "fmt/format.h"
#include "rigid_transform.h"

#include <iomanip>

namespace MSF {
// 定义了输出打印方式，主要用于调试
std::ostream &operator<<(std::ostream &out, const KinematicData &motion) {
    Eigen::Vector3d eulr_ = INS::quaternion2euler(motion.att) * 180 / M_PI;

    auto s_ = fmt::format(
        "t {: >6.4f} │ p {: >6.3f} {: >6.3f} {: >6.3f} │ v {: >6.3f} {: >6.3f} {: >6.3f} │ eulr_ {:6.3f} {:6.3f} {:6.3f} │ {:6.3f}",
        motion.measurement_timestamp,
        motion.pos.x(),
        motion.pos.y(),
        motion.pos.z(),
        motion.vel.x(),
        motion.vel.y(),
        motion.vel.z(),
        eulr_.x(),
        eulr_.y(),
        eulr_.z(),
        motion.ego_longitude_vel);
    out << s_;
    return out;
}

// 计算查询点到中心线的最近投影
//
// 几何推导：
// 给定线段AB（起点seg_start，终点seg_end）和查询点P（query_pt）
//
// 1. 线段向量：v = B - A
// 2. 单位方向向量：u = v / |v|（指向前进方向）
// 3. 从起点到查询点的向量：w = P - A
// 4. P在线段方向的投影距离：t_raw = w · u（点积）
//    - t_raw < 0: P在A点之前，投影点为A
//    - t_raw > |v|: P在B点之后，投影点为B
//    - 否则：投影点在线段内部
// 5. 投影点：Q = A + t * u，其中 t = clamp(t_raw, 0, |v|)
// 6. 距离向量：d_vec = Q - P（从查询点指向投影点）
// 7. 距离：dist = |d_vec|
//
// 左右判断（2D叉积）：
// cross_z = u × d_vec = u_x * d_vec_y - u_y * d_vec_x
// - cross_z > 0: 投影点Q在前进方向左侧（相对于查询点P）
// - cross_z < 0: 投影点Q在前进方向右侧（相对于查询点P）
//
// 符号约定（以查询点为原点，前进方向为参考）：
// - 投影点在右侧 → distance为正
// - 投影点在左侧 → distance为负
// - signed_dist = (cross_z < 0) ? +dist : -dist
//
// 边界序号约定：boundary_0、boundary_1（默认0为左、1为右）
void DbData::compute_projection_to_line(const Eigen::Vector2d &query_pt, const Eigen::Vector2d *line, DbProjection &result) const {
    bool found_inside_projection = false;

    for (int s_idx = 0; s_idx < BOUNDARY_POINT_NUM - 1; ++s_idx) {
        const Eigen::Vector2d &A = line[s_idx];     // 线段起点
        const Eigen::Vector2d &B = line[s_idx + 1]; // 线段终点

        // 线段向量 v = B - A
        Eigen::Vector2d v = B - A;
        double          L = v.norm(); // 线段长度
        if (L < 1e-10)
            continue;

        // 单位方向向量 u = v / L（前进方向）
        Eigen::Vector2d u = v / L;

        // 从起点到查询点的向量 w = P - A
        Eigen::Vector2d w     = query_pt - A;
        double          t_raw = w.dot(u); // P在AB方向的投影距离（无限制）

        // 限制投影点在线段范围内：t = clamp(t_raw, 0, L)
        double t = std::max(0.0, std::min(L, t_raw));

        // 投影点 Q = A + t * u
        Eigen::Vector2d Q = A + t * u;
        // 距离向量 d_vec = Q - P（从查询点指向投影点）
        Eigen::Vector2d d_vec = Q - query_pt;
        double          dist  = d_vec.norm();

        // 2D叉积判断左右：cross_z = u × d_vec = u_x * d_vec_y - u_y * d_vec_x
        double cross_z = u.x() * d_vec.y() - u.y() * d_vec.x();
        // 符号：投影点在右侧为正（cross_z < 0），左侧为负（cross_z > 0）
        double signed_dist = (cross_z < 0.0) ? dist : -dist;

        // 判断投影是否在线段内部
        bool is_inside = (t_raw >= 0.0 && t_raw <= L);
        if (is_inside) {
            found_inside_projection = true;
        }

        // 更新最近投影（绝对距离更小则更新）
        if (dist < std::abs(result.distance)) {
            result.proj_point  = Q;
            result.distance    = signed_dist;
            result.segment_idx = s_idx;
            result.is_valid    = is_inside;
        }
    }

    // 若所有投影都在线段端点外，则标记为无效
    if (!found_inside_projection) {
        result.is_valid = false;
    }
}

void DbData::debug_print_boundary_in_veh() const {
    AINFO << "=== DriveBoundary Debug ===";
    AINFO << "Timestamp: " << std::fixed << std::setprecision(4) << measurement_timestamp;

    // 打印 dr2veh_matrix 及其逆矩阵
    AINFO << "dr2veh_matrix (4x4):";
    for (int i = 0; i < 4; ++i) {
        AINFO << "  [" << std::fixed << std::setprecision(6)
              << dr2veh_matrix(i, 0) << ", "
              << dr2veh_matrix(i, 1) << ", "
              << dr2veh_matrix(i, 2) << ", "
              << dr2veh_matrix(i, 3) << "]";
    }
    Eigen::Matrix4d veh2dr_matrix = dr2veh_matrix.inverse();
    AINFO << "veh2dr_matrix (inverse):";
    for (int i = 0; i < 4; ++i) {
        AINFO << "  [" << std::fixed << std::setprecision(6)
              << veh2dr_matrix(i, 0) << ", "
              << veh2dr_matrix(i, 1) << ", "
              << veh2dr_matrix(i, 2) << ", "
              << veh2dr_matrix(i, 3) << "]";
    }

    // 构建Z置零后的转换矩阵，用于将DR系点（z=0）转换到Veh系
    // 因为DR系数据不带高度信息，直接使用原始矩阵会产生Z轴差异
    Eigen::Matrix4d veh2dr_matrix_z_zero = veh2dr_matrix;
    veh2dr_matrix_z_zero(2, 3)           = 0.0; // 将位移Z置零
    Eigen::Matrix4d dr2veh_for_print     = veh2dr_matrix_z_zero.inverse();
    AINFO << "dr2veh_for_print (with Z translation zeroed):";
    for (int i = 0; i < 4; ++i) {
        AINFO << "  [" << std::fixed << std::setprecision(6)
              << dr2veh_for_print(i, 0) << ", "
              << dr2veh_for_print(i, 1) << ", "
              << dr2veh_for_print(i, 2) << ", "
              << dr2veh_for_print(i, 3) << "]";
    }

    // 打印 veh2dr_translation
    AINFO << "veh2dr_translation: (" << std::fixed << std::setprecision(3)
          << veh2dr_translation.x() << ", "
          << veh2dr_translation.y() << ", "
          << veh2dr_translation.z() << ")";

    // 打印边界0形点（转换到车辆坐标系）
    AINFO << "Boundary 0 (in vehicle frame):";
    for (int j = 0; j < BOUNDARY_POINT_NUM; ++j) {
        const Eigen::Vector2d &pt_dr = boundary_0[j];
        Eigen::Vector4d        pt_homogeneous(pt_dr.x(), pt_dr.y(), 0.0, 1.0);
        Eigen::Vector4d        pt_veh = dr2veh_for_print * pt_homogeneous;
        AINFO << "  pt[" << j << "]: DR(" << std::fixed << std::setprecision(3)
              << pt_dr.x() << ", " << pt_dr.y() << ") -> Veh("
              << pt_veh.x() << ", " << pt_veh.y() << ")";
    }

    // 打印边界1形点（转换到车辆坐标系）
    AINFO << "Boundary 1 (in vehicle frame):";
    for (int j = 0; j < BOUNDARY_POINT_NUM; ++j) {
        const Eigen::Vector2d &pt_dr = boundary_1[j];
        Eigen::Vector4d        pt_homogeneous(pt_dr.x(), pt_dr.y(), 0.0, 1.0);
        Eigen::Vector4d        pt_veh = dr2veh_for_print * pt_homogeneous;
        AINFO << "  pt[" << j << "]: DR(" << std::fixed << std::setprecision(3)
              << pt_dr.x() << ", " << pt_dr.y() << ") -> Veh("
              << pt_veh.x() << ", " << pt_veh.y() << ")";
    }

    // 打印中心线形点（转换到车辆坐标系）
    AINFO << "Center line (in vehicle frame):";
    for (int j = 0; j < BOUNDARY_POINT_NUM; ++j) {
        const Eigen::Vector2d &pt_dr = center_line[j];
        Eigen::Vector4d        pt_homogeneous(pt_dr.x(), pt_dr.y(), 0.0, 1.0);
        Eigen::Vector4d        pt_veh = dr2veh_for_print * pt_homogeneous;
        AINFO << "  pt[" << j << "]: DR(" << std::fixed << std::setprecision(3)
              << pt_dr.x() << ", " << pt_dr.y() << ") -> Veh("
              << pt_veh.x() << ", " << pt_veh.y() << ")";
    }

    // 打印投影结果（转换到车辆坐标系）
    const Eigen::Vector2d &proj_dr = projection.proj_point;
    Eigen::Vector4d        proj_homogeneous(proj_dr.x(), proj_dr.y(), 0.0, 1.0);
    Eigen::Vector4d        proj_veh = dr2veh_for_print * proj_homogeneous;
    AINFO << "Projection to center line: DR(" << std::fixed << std::setprecision(3)
          << proj_dr.x() << ", " << proj_dr.y() << ") -> Veh("
          << proj_veh.x() << ", " << proj_veh.y() << "), distance="
          << projection.distance << ", segment_idx="
          << projection.segment_idx << ", valid="
          << (projection.is_valid ? "true" : "false");

    AINFO << "=== End DriveBoundary Debug ===";
}

std::string DbData::to_csv(double distance_smoothed_override) const {
    // veh2dr_translation: 车辆坐标系到DR坐标系的位移 (tx, ty, tz)
    auto t = fmt::format(        //
        "{:.6f},{:.6f},{:.6f}",  // tx, ty, tz
        veh2dr_translation.x(),  // tx: DR系X方向位移
        veh2dr_translation.y(),  // ty: DR系Y方向位移
        veh2dr_translation.z()); // tz: DR系Z方向位移

    // boundary_0: 左边界形点 (pt0_x, pt0_y, pt1_x, pt1_y, ..., pt4_x, pt4_y)
    auto b0 = fmt::format(                                                       //
        "{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}", // pt0~pt4 (x,y)
        boundary_0[0].x(), boundary_0[0].y(),                                    // pt0: 第1个形点 (x,y)
        boundary_0[1].x(), boundary_0[1].y(),                                    // pt1: 第2个形点 (x,y)
        boundary_0[2].x(), boundary_0[2].y(),                                    // pt2: 第3个形点 (x,y)
        boundary_0[3].x(), boundary_0[3].y(),                                    // pt3: 第4个形点 (x,y)
        boundary_0[4].x(), boundary_0[4].y());                                   // pt4: 第5个形点 (x,y)

    // boundary_1: 右边界形点 (pt0_x, pt0_y, pt1_x, pt1_y, ..., pt4_x, pt4_y)
    auto b1 = fmt::format(                                                       //
        "{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}", // pt0~pt4 (x,y)
        boundary_1[0].x(), boundary_1[0].y(),                                    // pt0: 第1个形点 (x,y)
        boundary_1[1].x(), boundary_1[1].y(),                                    // pt1: 第2个形点 (x,y)
        boundary_1[2].x(), boundary_1[2].y(),                                    // pt2: 第3个形点 (x,y)
        boundary_1[3].x(), boundary_1[3].y(),                                    // pt3: 第4个形点 (x,y)
        boundary_1[4].x(), boundary_1[4].y());                                   // pt4: 第5个形点 (x,y)

    // 使用传入的 distance_smoothed_override，如果为 NaN 则使用原值
    double distance_smoothed_val = std::isnan(distance_smoothed_override)
                                       ? projection.distance_smoothed
                                       : distance_smoothed_override;

    // projection: 投影点结果 (proj_x, proj_y, distance, distance_smoothed, segment_idx, is_valid)
    auto p = fmt::format(                    //
        "{:.6f},{:.6f},{:.6f},{:.6f},{},{}", // proj_point, distance, segment_idx, is_valid
        projection.proj_point.x(),           // proj_x: 投影点X坐标 (DR系)
        projection.proj_point.y(),           // proj_y: 投影点Y坐标 (DR系)
        projection.distance,                 // distance: 到中心线距离 (右侧为正)
        distance_smoothed_val,               // distance_smoothed: 平滑后的距离（可覆盖）
        projection.segment_idx,              // segment_idx: 所在线段索引
        projection.is_valid ? 1 : 0);        // is_valid: 投影是否有效 (1有效/0无效)

    return fmt::format("{},{},{},{}", t, b0, b1, p);
}

} // namespace MSF