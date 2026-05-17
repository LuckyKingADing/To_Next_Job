#pragma once
#include "kinematic_types.h"

#include <deque>
#include <mutex>

namespace analysis {

// 时序上的补偿
class KinematicCompensation {
private:
    // 普通队列，多线程操作需要上锁
    const size_t              MOTION_MAX_QUEUE_SIZE = 50000000;
    const double              MIN_SAMPLING_GAP      = 0.015;
    std::deque<KinematicData> motion_queue;
    std::recursive_mutex      motion_queue_lock;

public:
    // 队列enque
    void insert(const KinematicData &);
    // 计算补偿量
    std::pair<int, KinematicData> delta(double t1, double t2, bool use_veh_frame);
    void                          crop_by_timestamp(double min, double max, double gap = 0.1);
};

} // namespace analysis