#include "fail_detect_interface_impl.h"
#include "cyber/common/log.h"

namespace byd {
namespace tcmsf {
namespace fail_detect {

// 将角度（弧度）归一化到 [-pi,pi]
inline double NormalizeArcAngle(const double arc_angle) {
    double a = std::fmod(arc_angle + M_PI, 2.0 * M_PI);
    if (a < 0) {
        a += M_PI;
    } else {
        a -= M_PI;
    }
    return a;
}

// 将角度（度）归一化到 [-180.0,180.0]
inline double NormalizeDegAngle(const double deg_angle) {
    double a = std::fmod(deg_angle + 180.0, 2.0 * 180.0);
    if (a < 0) {
        a += 180.0;
    } else {
        a -= 180.0;
    }
    return a;
}

void IMUFailDetectImpl::insert_kinematics_info(const MSF::KinematicDataPtr dr_ptr_, const MSF::StatePtr tcmsf_ptr_, const MSF::ImuDataPtr imu_ptr_) {
    kinematicsINFO info;
    info.timestamp        = imu_ptr_->measurement_timestamp;
    info.dr.pos           = dr_ptr_->pos;
    info.imu.acc          = imu_ptr_->acc;
    info.imu.gyro         = imu_ptr_->gyro;
    info.tcmsf.blh        = tcmsf_ptr_->lla;
    info.tcmsf.speed      = tcmsf_ptr_->vel.norm();
    info.tcmsf.rtk_status = tcmsf_ptr_->rtk_status;
    double dt_            = 0.0;
    if (info_deque_.size() != 0) {
        dt_ = info.timestamp - info_deque_.back().timestamp;
        insert<kinematicsINFO>(info, info_deque_, dt_);
    } else {
        insert<kinematicsINFO>(info, info_deque_, 1e10);
    }
}

bool IMUFailDetectImpl::calc_gyro_scale_z(double &delta_heading_ref_, double &delta_heading_, double &scale_) {

    if (
        info_deque_.size() != MAXIMUM_DEQUE_SIZE ||                                                                                   // 队列不满认为无效
        std::abs(info_deque_.back().timestamp - info_deque_.front().timestamp) > MAXIMUM_DEQUE_SIZE * MINIMUM_SAMPLING_INTERVAL * 2.0 // 帧不稳定性超过100%，认为无效
    ) {
        return false;
    }
    if (std::all_of(info_deque_.begin(), info_deque_.end(), [](const kinematicsINFO &info) { return (info.tcmsf.rtk_status == 6 && info.tcmsf.speed > 10.0); })) {
        // 全部为固定解状态
        // 且速度需要持续保持大于5m/s
        if (info_deque_.size() < 10) {
            // 队列最少要有10个元素
            return false;
        }
        // 分别在队列首尾取第一、五个位置，计算航向角
        auto   blh_f0    = info_deque_.begin()->tcmsf.blh;
        auto   blh_f     = (info_deque_.begin() + 4)->tcmsf.blh;
        auto   blh_e     = info_deque_.rbegin()->tcmsf.blh;
        auto   blh_e0    = (info_deque_.rbegin() + 4)->tcmsf.blh;
        double blh_hdg_f = local_trans_.HeadingFromBLH(blh_f0, blh_f);
        double blh_hdg_e = local_trans_.HeadingFromBLH(blh_e0, blh_e);
        double blh_dhdg  = blh_hdg_e - blh_hdg_f;

        auto   dr_f0    = info_deque_.begin()->dr.pos;
        auto   dr_f     = (info_deque_.begin() + 4)->dr.pos;
        auto   dr_e     = info_deque_.rbegin()->dr.pos;
        auto   dr_e0    = (info_deque_.rbegin() + 4)->dr.pos;
        double dr_hdg_f = local_trans_.HeadingFromLocalPos(dr_f0, dr_f);
        double dr_hdg_e = local_trans_.HeadingFromLocalPos(dr_e0, dr_e);
        double dr_dhdg  = dr_hdg_e - dr_hdg_f;

        // AINFO_EVERY(100) << "blh_hdg_f " << blh_hdg_f << " blh_hdg_e " << blh_hdg_e << " blh_dhdg " << blh_dhdg;
        // AINFO_EVERY(100) << "dr_hdg_f " << dr_hdg_f << " dr_hdg_e " << dr_hdg_e << " dr_dhdg " << dr_dhdg;

        blh_dhdg = NormalizeDegAngle(blh_dhdg);
        dr_dhdg  = NormalizeDegAngle(dr_dhdg);

        if (
            std::abs(blh_dhdg) > 3.0 &&  //
            std::abs(blh_dhdg) < 45.0 && //
            std::abs(dr_dhdg) > 3.0 &&   //
            std::abs(dr_dhdg) < 45.0     //
        ) {
            // 队列长度约 2 秒，只考虑两秒内变化 3 度以上 和 45 度以内的情况，进行Z误差系数的计算
            double dhdg_ = dr_dhdg - blh_dhdg;

            {
                // 队列长度约为2秒
                // 这里假设DR和MSF航向的偏差量是个小于90度的值
                // 把航向偏差映射到-180~180
                dhdg_ = NormalizeDegAngle(dhdg_);

                if (std::abs(dhdg_) > 90.0) {
                    return false;
                }
            }

            double dhdg_scale_ = dhdg_ / blh_dhdg;

            // 成功计算出偏差量，对相应状态进行赋值
            delta_heading_ref_ = blh_dhdg;
            delta_heading_     = dr_dhdg;
            scale_             = dhdg_scale_;
            info_deque_.clear();
            return true;
        }
    }
    return false;
}

bool IMUFailDetectImpl::imu_fatal_error_detect(bool &is_imu_fail_) {
    return false;
}
} // namespace fail_detect
} // namespace tcmsf
} // namespace byd