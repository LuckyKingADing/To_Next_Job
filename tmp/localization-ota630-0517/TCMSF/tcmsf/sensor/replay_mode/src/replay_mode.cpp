#include "replay_mode.h"

namespace byd {
namespace replay_mode {
ReplayModeSGT::ReplayModeSGT(const std::string &record_file_path_) {
    // 仅进行一次操作
    static auto replay_mode_parse_record_call_once_ = [this, &record_file_path_] {
        if (!pose_info_.first) {
            apollo::cyber::record::RecordReader  reader(record_file_path_);
            apollo::cyber::record::RecordMessage msg_;
            while (reader.ReadMessage(&msg_)) {
                if (msg_.channel_name == TCMSF_TOPIC_NAME_) {
                    pose_info_.second->ParseFromString(msg_.content);
                    pose_info_.first.store(true);
                    AINFO << fmt::format("Get TCMSF state from record file, t: {:14.4f}", pose_info_.second->header().measurement_timestamp());
                    break;
                }
            }
        }
        return 0;
    }();
    (void)replay_mode_parse_record_call_once_;
}

void ReplayModeSGT::sync_info_to_state(std::shared_ptr<MSF::State> state_ptr_, MSF::KfPtr<21, 3> kf_ptr, std::function<void()> ready_cb_) {
    if (pose_info_.first.load()) {
        // 仅进行一次操作
        static auto sync_info_to_state_call_once_ = [this, &state_ptr_, &kf_ptr, &ready_cb_] {
            AINFO << "[replay mode] set state from record msg";
            auto &msg_                        = pose_info_.second;
            state_ptr_->measurement_timestamp = msg_->header().measurement_timestamp();
            state_ptr_->att.w()               = msg_->attitude().qw();
            state_ptr_->att.x()               = msg_->attitude().qx();
            state_ptr_->att.y()               = msg_->attitude().qy();
            state_ptr_->att.z()               = msg_->attitude().qz();
            state_ptr_->vel.x()               = msg_->velocity().x();
            state_ptr_->vel.y()               = msg_->velocity().y();
            state_ptr_->vel.z()               = msg_->velocity().z();
            state_ptr_->lla.x()               = msg_->position().lat() / 180.0 * M_PI;
            state_ptr_->lla.y()               = msg_->position().lon() / 180.0 * M_PI;
            state_ptr_->lla.z()               = msg_->position().height();
            state_ptr_->gyro_bias.x()         = msg_->gyro_bias().x();
            state_ptr_->gyro_bias.y()         = msg_->gyro_bias().y();
            state_ptr_->gyro_bias.z()         = msg_->gyro_bias().z();
            state_ptr_->acc_bias.x()          = msg_->acc_bias().x();
            state_ptr_->acc_bias.y()          = msg_->acc_bias().y();
            state_ptr_->acc_bias.z()          = msg_->acc_bias().z();
            state_ptr_->vehicle_bias.x()      = msg_->vehicle_bias().x();
            state_ptr_->vehicle_bias.y()      = msg_->vehicle_bias().y();
            state_ptr_->vehicle_bias.z()      = msg_->vehicle_bias().z();
            state_ptr_->map_bias.x()          = msg_->map_bias().x();
            state_ptr_->map_bias.y()          = msg_->map_bias().y();
            state_ptr_->map_bias.z()          = msg_->map_bias().z();
            state_ptr_->mileage               = msg_->mileage();

            state_ptr_->align_type    = (MSF::State::AlignType)msg_->align_status();
            state_ptr_->fusion_status = (MSF::State::FusionStatus)msg_->fusion_status();

            AINFO << "[replay mode] set KF P from record msg";
            Eigen::Matrix<double, 21, 1> Pxx = Eigen::Matrix<double, 21, 1>::Zero();
            Pxx << //
                msg_->attitude_std().x(),
                msg_->attitude_std().y(),
                msg_->attitude_std().z(),
                msg_->velocity_std().x(),
                msg_->velocity_std().y(),
                msg_->velocity_std().z(),
                msg_->position_std().x(),
                msg_->position_std().y(),
                msg_->position_std().z(),
                msg_->gyro_bias_std().x(),
                msg_->gyro_bias_std().y(),
                msg_->gyro_bias_std().z(),
                msg_->acc_bias_std().x(),
                msg_->acc_bias_std().y(),
                msg_->acc_bias_std().z(),
                msg_->vehicle_bias_std().x(),
                msg_->vehicle_bias_std().y(),
                msg_->vehicle_bias_std().z(),
                msg_->map_bias_std().x(),
                msg_->map_bias_std().y(),
                msg_->map_bias_std().z();
            kf_ptr->P = Pxx.cwiseAbs2().asDiagonal();

            AINFO << "[replay mode] additional operation by ready_cb_";
            if (ready_cb_) {
                ready_cb_();
            }
            return 0;
        }();
        (void)sync_info_to_state_call_once_;
    }
}

std::pair<bool, double> ReplayModeSGT::get_init_timestamp() {
    return std::make_pair<bool, double>(pose_info_.first, pose_info_.second->header().measurement_timestamp());
}

} // namespace replay_mode
} // namespace byd