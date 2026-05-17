#pragma once

#include "Eigen/Eigen"
#include <deque>
#include <limits>
#include <math.h>
#include <stdint.h>

namespace MSF {

class Mean {
private:
    const uint64_t              BUF_SIZE;
    uint64_t                    idx;
    Eigen::Vector3d             mean;
    std::deque<Eigen::Vector3d> queue;

public:
    Mean(uint64_t buf_size_) :
        BUF_SIZE(buf_size_) {
        mean = Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d operator()(Eigen::Vector3d x_) {
        queue.push_back(x_);
        if (queue.size() > BUF_SIZE) {
            queue.pop_front();
        }
        uint64_t size = queue.size();
        for (auto &q_ : queue) {
            mean += q_ / double(size);
        }

        return mean;
    }
    void clear() {
        mean.setZero();
        queue.clear();
    }
};
} // namespace MSF