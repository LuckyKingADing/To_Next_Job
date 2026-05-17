#include "imu_issue_fallback_impl.h"
#include "Eigen/Eigen"
#include "cyber/common/log.h"
#include "fmt/format.h"
#include <atomic>
#include <fstream>
namespace MSF::IIF {

std::shared_ptr<Imu> ImuIssueFallbackImpl::insert_imu(const std::shared_ptr<Imu> imu_msg_) {

    bool msg_key_sufficient =
        imu_msg_->has_header() &&
        imu_msg_->header().has_measurement_timestamp() &&
        imu_msg_->header().has_publish_timestamp() &&
        imu_msg_->has_accel() &&
        imu_msg_->accel().has_x() &&
        imu_msg_->accel().has_y() &&
        imu_msg_->accel().has_z() &&
        imu_msg_->has_gyro() &&
        imu_msg_->gyro().has_x() &&
        imu_msg_->gyro().has_y() &&
        imu_msg_->gyro().has_z() &&
        imu_msg_->has_imu_status();

    if (!msg_key_sufficient) {
        // 如果消息字段不足，直接返回原消息
        return imu_msg_;
    }

    bool sensor_status_good =
        imu_msg_->imu_status() == 1;

    if (!sensor_status_good) {
        // 如果传感器给了异常状态，直接返回原消息
        return imu_msg_;
    }

    imu_gyro_sudden_jump_probe(imu_msg_);

    // 将当前帧IMU消息保存为上一帧消息
    // 这里可能需要看看是否需要复制一份消息，复制一份应该健壮性最好，但是可能损失一些性能
    pre_imu_msg = std::make_shared<Imu>(*imu_msg_);
    // pre_imu_msg = imu_msg_;

    {
        // 这里查看是否有IMU异常，如果出现IMU异常，则复制IMU消息，并对gyro的值进行修正，然后返回
        // gyro的x、y置零，z使用底盘的yawrate嫁接

        double issue_dt_            = imu_msg_->header().measurement_timestamp() - imu_gyro_issue_measurement_timestamp;
        auto   cur_veh_msg_tmp      = std::atomic_load(&cur_veh_msg);
        double yawrate_veh_deg_p_s_ = cur_veh_msg_tmp->ego_motion_status().da_in_yawrate_sg() * 180.0 / M_PI;
        if (std::fabs(issue_dt_) < GYRO_SUDDEN_JUMP_LIKELY_DUARTION_BOUND_) {

            auto imu_msg_mod_ = std::make_shared<Imu>(*imu_msg_);

            imu_msg_mod_->mutable_gyro()->set_x(0.0);
            imu_msg_mod_->mutable_gyro()->set_y(0.0);
            imu_msg_mod_->mutable_gyro()->set_z(yawrate_veh_deg_p_s_);

            // 返回修改后的imu消息
            return imu_msg_mod_;
        }
    }

    // 默认返回原消息
    return imu_msg_;
}

int ImuIssueFallbackImpl::insert_veh(const std::shared_ptr<VehInfo> veh_msg_) {
    bool msg_key_sufficient =
        veh_msg_->has_header() &&
        veh_msg_->header().has_measurement_timestamp() &&
        veh_msg_->header().has_publish_timestamp() &&
        veh_msg_->has_ego_motion_status() &&
        veh_msg_->ego_motion_status().has_da_in_yawrate_sg() &&
        veh_msg_->ego_motion_status().has_da_in_yawrate_v_u8() &&
        veh_msg_->ego_motion_status().has_da_in_latacc_sg() &&
        veh_msg_->ego_motion_status().has_da_in_latacc_v_u8() &&
        veh_msg_->ego_motion_status().has_da_in_lgtacc_sg() &&
        veh_msg_->ego_motion_status().has_da_in_lgtacc_v_u8() &&
        veh_msg_->has_brake_system_status() &&
        veh_msg_->brake_system_status().has_da_in_vdcactive_u8() &&
        veh_msg_->brake_system_status().has_da_in_tcsactive_u8() &&
        veh_msg_->brake_system_status().has_da_in_espactive_u8();

    if (!msg_key_sufficient) {
        // 如果消息异常返回-1
        return -1;
    }

    // 根据VDC、TCS、ESP三个系统的状态判断底盘轮速是否可信
    bool is_vdc_active = veh_msg_->brake_system_status().da_in_vdcactive_u8() == 0x1;
    bool is_tcs_active = veh_msg_->brake_system_status().da_in_tcsactive_u8() == 0x1;
    bool is_esp_active = veh_msg_->brake_system_status().da_in_espactive_u8() == 0x1;

    // 可能打滑的条件
    bool may_slip = is_vdc_active || is_tcs_active || is_esp_active;

    // 判断几个值状态位是否可信
    bool is_yawrate_ok = veh_msg_->ego_motion_status().da_in_yawrate_v_u8() == 0x1;
    bool is_latacc_ok  = veh_msg_->ego_motion_status().da_in_latacc_v_u8() == 0x1;
    bool is_lgtacc_ok  = veh_msg_->ego_motion_status().da_in_lgtacc_v_u8() == 0x1;

    bool sensor_status_good =
        !may_slip &&
        is_yawrate_ok &&
        is_latacc_ok &&
        is_lgtacc_ok;
    if (!sensor_status_good) {
        // 如果消息异常返回-1
        return -1;
    }

    // 保存当前有效的底盘消息
    // 这里可能需要看看是否需要复制一份消息，复制一份应该健壮性最好，但是可能损失一些性能
    // 底盘的数据量较大，这里使用浅拷贝。
    std::atomic_store(&cur_veh_msg, veh_msg_);

    return 0;
}

void ImuIssueFallbackImpl::imu_gyro_sudden_jump_probe(const std::shared_ptr<Imu> cur_imu_msg) {

    auto cur_veh_msg_tmp = std::atomic_load(&cur_veh_msg);

    double imu_dt     = pre_imu_msg->header().measurement_timestamp() - cur_imu_msg->header().measurement_timestamp();
    double imu_veh_dt = cur_veh_msg_tmp->header().measurement_timestamp() - cur_imu_msg->header().measurement_timestamp();

    // imu 前后帧间隔超过一定值，或小于一定值，不做处理
    if (std::fabs(imu_dt) > 0.05 || std::fabs(imu_dt) < 0.005) {
        return;
    }

    // 当前imu和veh间隔超过一定值，不做处理
    if (std::fabs(imu_veh_dt) > 0.5) {
        return;
    }

    Eigen::Vector3d pre_gyro;
    Eigen::Vector3d cur_gyro;
    pre_gyro << pre_imu_msg->gyro().x(), pre_imu_msg->gyro().y(), pre_imu_msg->gyro().z();
    cur_gyro << cur_imu_msg->gyro().x(), cur_imu_msg->gyro().y(), cur_imu_msg->gyro().z();

    Eigen::Vector3d gyro_jump = (cur_gyro - pre_gyro) / imu_dt;

    if ((gyro_jump.cwiseAbs().array() > GYRO_SUDDEN_JUMP_BOUND_).any()) {
        imu_gyro_issue_measurement_timestamp = cur_imu_msg->header().measurement_timestamp();
        AWARN << "imu gyro sudden jump detected! timestamp: " << fmt::format("{:>14.5f}", imu_gyro_issue_measurement_timestamp);
    }

    // {
    //     // 记录到文本文件中，以便于分析
    //     // 仅调试使用，实车需要注释掉
    //     static std::fstream fs_;
    //     static auto         fs_call_once_ = [this] {
    //         // 执行一次的逻辑
    //         fs_.open("data/tmp/iif_state.csv", std::ios::out);
    //         return 0;
    //     }(); // 定义后立即执行
    //     (void)fs_call_once_;

    //     double yawrate_veh_deg_p_s_ = cur_veh_msg->ego_motion_status().da_in_yawrate_sg() * 180.0 / M_PI;

    //     bool mod_ = std::fabs(cur_imu_msg->header().measurement_timestamp() - imu_gyro_issue_measurement_timestamp) < GYRO_SUDDEN_JUMP_LIKELY_DUARTION_BOUND_;

    //     fs_ << fmt::format(                                                                                          //
    //         "{:>14.5f},{:>5.5f},{:>5.5f},{:>5.5f},{:>5.5f},{:>5.5f},{:>5.5f},{:>5.5f},{:>5.5f},{:>5.5f},{:>5.5f}\n", //
    //         cur_imu_msg->header().measurement_timestamp(),                                                           //
    //         cur_gyro.x(), cur_gyro.y(), cur_gyro.z(),                                                                //
    //         gyro_jump.x(), gyro_jump.y(), gyro_jump.z(),                                                             //
    //         yawrate_veh_deg_p_s_,                                                                                    //
    //         mod_ ? 0.0 : cur_gyro.x(),                                                                               //
    //         mod_ ? 0.0 : cur_gyro.y(),                                                                               //
    //         mod_ ? yawrate_veh_deg_p_s_ : cur_gyro.z());
    // }
}

} // namespace MSF::IIF