#pragma once
#include "cyber/common/log.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"
#include "fmt/format.h"
#include "kalman_filter.h"
#include "modules/msg/localization_msgs/tcmsf.pb.h"
#include "state_type.h"
#include <atomic>
#include <string>

// #define __TCMSF_DEBUG__ENABLE_REPLAY_MODE

namespace byd {
namespace replay_mode {

static const std::string TCMSF_TOPIC_NAME_ = "/localization/tcmsf";

using byd::modules::tcmsf::Pose;

class ReplayModeSGT {
public:
    static ReplayModeSGT &getInstance(const std::string &record_file_path_ = "") {
        static ReplayModeSGT inst(record_file_path_);
        return inst;
    }

    ReplayModeSGT(const ReplayModeSGT &)            = delete;
    ReplayModeSGT &operator=(const ReplayModeSGT &) = delete;

    ~ReplayModeSGT() {
        AINFO << "exit replay mode sgt";
    }

private:
    std::pair<std::atomic_bool, std::unique_ptr<Pose>> pose_info_{false, std::make_unique<Pose>()};

private:
    ReplayModeSGT(const std::string &record_file_path_);

public:
    void                    sync_info_to_state(std::shared_ptr<MSF::State> state_ptr_, MSF::KfPtr<21, 3> kf_ptr, std::function<void()> ready_cb_);
    std::pair<bool, double> get_init_timestamp();
};

} // namespace replay_mode
} // namespace byd
