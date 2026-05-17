#pragma once

#include "Eigen/Dense"
#include "modules/localization/src/TCMSF/tcmsf/sensor/base/include/base_type.h"

#include <deque>

namespace byd {
namespace tcmsf {

// 时序上的补偿
class KinematicCompensation {
private:
    // 普通队列，多线程操作需要上锁
    const size_t                   MOTION_MAX_QUEUE_SIZE = 200;
    const double                   MIN_SAMPLING_GAP      = 0.015;
    std::deque<MSF::KinematicData> motion_queue;
    std::recursive_mutex           motion_queue_lock;

public:
    // 队列enque
    void insert(const MSF::KinematicData &);
    // 计算补偿量
    std::pair<int, MSF::KinematicData> delta(double t1, double t2, bool use_veh_frame);
    // 插值得到对应时间的DR值
    std::pair<int, MSF::KinematicData> pin(double t);
};

} // namespace tcmsf
} // namespace byd