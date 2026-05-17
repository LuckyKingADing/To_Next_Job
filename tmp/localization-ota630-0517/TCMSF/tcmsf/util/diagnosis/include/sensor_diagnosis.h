#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>

namespace byd {
namespace tcmsf {
namespace diagnosis {
// TODO: 函数功能未完成
class CartModeDetect {
public:
    struct KinematicInfo {
        double time_stamp;
        double angular_rate;
        double acceleration;
        KinematicInfo() {
            time_stamp   = 0.0;
            angular_rate = 0.0;
            acceleration = 0.0;
        }
        KinematicInfo(double time_stamp_, double angular_rate_, double acceleration_) {
            time_stamp   = time_stamp_;
            angular_rate = angular_rate_;
            acceleration = acceleration_;
        }
    };

private:
    static constexpr uint64_t MAX_QUEUE_SIZE = 100;

    std::deque<KinematicInfo> kinematic_queue;

    void clear() {
        kinematic_queue.clear();
    }
    void insert(const KinematicInfo &kinematic_info_) {
        kinematic_queue.push_back(kinematic_info_);
        while (kinematic_queue.size() > MAX_QUEUE_SIZE) {
            kinematic_queue.pop_front();
        }
    }

private:
    double mean_angular_rate = 0.0;
    double mean_acceleration = 0.0;

public:
    bool operator()(const KinematicInfo &kinematic_info_) {
        if (kinematic_queue.size() == 0) {
            insert(kinematic_info_);
            return false;
        } else {
            if (std::abs(kinematic_queue.back().time_stamp - kinematic_info_.time_stamp) > 1.0) {
                clear();
                return false;
            }
            if (kinematic_queue.size() == MAX_QUEUE_SIZE) {

            } else {
                insert(kinematic_info_);
                return false;
            }
        }
        return false;
    }
};

} // namespace diagnosis
} // namespace tcmsf
} // namespace byd