#include "zupt_processor.h"
#include "earth.h"
#include "fmt/format.h"
#include "processor_debug.h"

namespace MSF {

bool ZuptProcessor::UpdateStateByZupt(StatePtr state_ptr, KfPtr<21, 3> kf_ptr) {

    zupt_count_++;
    static constexpr uint64_t ZUPT_SKIP_ = 5;
    if (zupt_count_ % ZUPT_SKIP_ != 0) {
        return false;
    }

    Eigen::Matrix<double, 3, 21> H;
    Eigen::Matrix<double, 3, 1>  innovation;
    Eigen::Matrix<double, 3, 3>  V;
    Eigen::Matrix<bool, 21, 1>   idx;

    if (state_ptr->vel.block<2, 1>(0, 0).norm() < 1.0e-2) {
        continous_zero_speed_count++;
    } else {
        continous_zero_speed_count = 0;
    }

    bool zupt_imu_bias_estimate_ =
        continous_zero_speed_count > 1 &&
        (state_ptr->gyro - state_ptr->gyro_bias).norm() < 0.2 / 180.0 * M_PI;

    if (zupt_imu_bias_estimate_) {
        process_control_sgt.fusion_status.continous_zupt_imu_bias_estimate_count++;
    }

    // ----------------
    // fuse zero position
    // 此处零速更新，使得位置往零速的初态上靠

    Eigen::Vector3d dlla_     = state_ptr->lla - process_control_sgt.lla0_zupt;
    double          distance_ = localtrans.DPos_LLAtoEgo(state_ptr->lla, dlla_, state_ptr->att).block<2, 1>(0, 0).norm();
    if (distance_ > 0.5 && state_ptr->vel.norm() < 0.1 && distance_ < 1e3) {
        AINFO << "zupt pos change too big, reset back";
        state_ptr->lla = process_control_sgt.lla0_zupt;
    }

    // ------------------
    // fuse zero velocity
    {
        // innovation.setZero();
        H.setZero();
        innovation          = state_ptr->vel;
        H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix<double, 3, 1> zupt_velocity_cov_additional = state_ptr->vel_ego.cwiseAbs2();
    for (int i = 0; i < 3; i++) {
        auto            &inno_                     = zupt_velocity_cov_additional(i);
        constexpr double vel_additional_std_bound_ = 0.02 * 0.02;
        if (inno_ > vel_additional_std_bound_) {
            inno_ = vel_additional_std_bound_;
        }
    }

    Eigen::Matrix<double, 3, 1> zupt_velocity_cov_std;
    zupt_velocity_cov_std << 0.002, 0.01, 0.005;

    Eigen::Matrix<double, 3, 1> zupt_velocity_cov = zupt_velocity_cov_std.cwiseAbs2();

    V = (zupt_velocity_cov + zupt_velocity_cov_additional).asDiagonal();

    V = state_ptr->C_b2n * V * state_ptr->C_b2n.transpose();

    kf_ptr->kf_update(H, V, innovation);

    {

        state_ptr->vel -= kf_ptr->dx.block<3, 1>(3, 0);

        idx << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
        CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);
        if (zupt_imu_bias_estimate_) {
            Eigen::Vector3d datt  = constrain(kf_ptr->dx.block<3, 1>(0, 0), MaxEulrAnglePerMeasurementUpdate);
            datt.z()              = 0.0;
            Eigen::Quaterniond dq = INS::euler2quaternion(datt);
            state_ptr->att        = dq * state_ptr->att;
            state_ptr->acc_bias += kf_ptr->dx.block<3, 1>(12, 0);
            StateConstrain(state_ptr->acc_bias, constrain_acc_bias);
            idx << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1;
            CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);
        }
    }

    // -----------------
    // fuse zero anguler velocity
    {
        // innovation.setZero();
        H.setZero();
        innovation            = state_ptr->gyro_bias - state_ptr->gyro;
        H.block<3, 3>(0, 9)   = Eigen::Matrix3d::Identity();
        Eigen::Vector3d g_t   = state_ptr->gyro * 180 / M_PI;
        Eigen::Vector3d g_tb  = state_ptr->gyro_bias * 180 / M_PI;
        Eigen::Vector3d inn_t = innovation * 180 / M_PI;

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
        debug::debug_sgt.zup_state.line(
            fmt::format("{:>14.3f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f},{:>6.4f}\n", //
                        state_ptr->measurement_timestamp, g_t.x(), g_t.y(), g_t.z(), inn_t.x(), inn_t.y(), inn_t.z(), g_tb.x(), g_tb.y(), g_tb.z(), state_ptr->vel.norm()));
#endif
    }

    Eigen::Matrix<double, 3, 1> zupt_anguler_velocity_cov_additional = innovation.cwiseAbs2();
    for (int i = 0; i < 3; i++) {
        auto            &inno_                      = zupt_anguler_velocity_cov_additional(i);
        constexpr double gyro_additional_std_bound_ = (0.1 / 180.0 * M_PI) * (0.1 / 180.0 * M_PI);
        if (inno_ > gyro_additional_std_bound_) {
            inno_ = gyro_additional_std_bound_;
        }
    }

    Eigen::Matrix<double, 3, 1> zupt_anguler_velocity_cov_std;
    zupt_anguler_velocity_cov_std << 0.01 / 180.0 * M_PI, 0.01 / 180.0 * M_PI, 0.01 / 180.0 * M_PI;

    Eigen::Matrix<double, 3, 1> zupt_anguler_velocity_cov = zupt_anguler_velocity_cov_std.cwiseAbs2();

    V = (zupt_anguler_velocity_cov + zupt_anguler_velocity_cov_additional).asDiagonal();

    kf_ptr->kf_update(H, V, innovation);

    if (zupt_imu_bias_estimate_) {
        state_ptr->gyro_bias -= kf_ptr->dx.block<3, 1>(9, 0);

        idx << 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1;
        CovarianceUpdateByIdx(kf_ptr->P, kf_ptr->cov, idx, true);

        StateConstrain(state_ptr->gyro_bias, constrain_gyro_bias);
    }

    if (zupt_imu_bias_estimate_) {
        Eigen::Quaterniond dq_    = state_ptr->att.conjugate() * MSF::process_control_sgt.att0_zupt;
        Eigen::Vector3d    deulr_ = INS::quaternion2euler(dq_);
        H.setZero();
        double dhdg_ = deulr_.z();
        if (dhdg_ >= M_PI) {
            dhdg_ = dhdg_ - 2.0 * M_PI;
        }
        if (dhdg_ <= -M_PI) {
            dhdg_ = dhdg_ + 2.0 * M_PI;
        }
        innovation = Eigen::Vector3d{0.0, 0.0, dhdg_};
        // Compute jacobian.
        H.block<1, 1>(2, 2) << 1.0;

        double          hdg_std_ = 10.0 / 180.0 * M_PI;
        Eigen::Vector3d att_cov  = std::pow(hdg_std_, 2) * Eigen::Vector3d::Ones();

        Eigen::Vector3d att_cov_additional = innovation.cwiseAbs2();

        V = (att_cov + att_cov_additional).asDiagonal();

        kf_ptr->kf_update(H, V, innovation);

        Eigen::Vector3d    datt = kf_ptr->dx.block<3, 1>(0, 0);
        Eigen::Quaterniond dq   = INS::rv2q(datt);
        state_ptr->att          = state_ptr->att * dq;
    }

    process_control_sgt.fusion_status.imu_update_count_after_gnss_update = 0;

    return true;
}

} // namespace MSF