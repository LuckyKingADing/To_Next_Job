#pragma once

#include <deque>
#include <limits>
#include <math.h>
#include <stdint.h>

namespace MSF {

// class MSE {
// private:
//     const uint64_t              BUF_SIZE;
//     uint64_t                    idx;
//     Eigen::Vector3d             mean;
//     Eigen::Vector3d             mse;
//     std::deque<Eigen::Vector3d> queue;

// public:
//     MSE(uint64_t buf_size_) : BUF_SIZE(buf_size_) {
//         idx  = 0;
//         mean = Eigen::Vector3d::Zero();
//         mse  = Eigen::Vector3d::Zero();
//     }

//     std::pair<Eigen::Vector3d, Eigen::Vector3d> operator()(Eigen::Vector3d x_) {
//         idx++;
//         queue.push_back(x_);
//         if (queue.size() > BUF_SIZE) {
//             queue.pop_front();
//         }
//         uint64_t size = queue.size();
//         for (auto &q_ : queue) {
//             mean += q_ / double(size);
//         }
//         for (auto &q_ : queue) {
//             mse += (q_ - mean).array().pow(2).matrix() / double(size);
//         }

//         return std::make_pair(mean, mse);
//     }
//     void clear() {
//         idx = 0;
//         mean.setZero();
//         mse.setZero();
//         queue.clear();
//     }
// };

class IGG3 {
private:
    double k0 = 1.0, k1 = 2.0;

public:
    IGG3(double k0_, double k1_) { // k1 >= 2*k0 > k0_ > 0; Typical values k0=1.0~1.5，k1=2.0~3.0
        k0 = k0_;
        if (k1_ < 2.0 * k0) {
            k1 = 2.0 * k0;
        } else {
            k1 = k1_;
        }
    }
    IGG3() {}

    double operator()(double err_) {
        double w   = 1e-10;
        double err = std::abs(err_);
        if (err < k0) {
            w = 1.0;
        } else if (err < k1) {
            w = k0 / err * ((k1 - err) / (k1 - k0)) * ((k1 - err) / (k1 - k0));
        } else {
            w = 1e-10;
        }
        return w;
    }
};
} // namespace MSF