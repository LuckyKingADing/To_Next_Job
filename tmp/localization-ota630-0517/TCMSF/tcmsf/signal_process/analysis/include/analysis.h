#pragma once

#include "Eigen/Eigen"
#include <array>
#include <chrono>
#include <deque>
#include <iostream>
#include <limits>

namespace MSF {
using VDSA = class VEHICEL_DRIVING_STATUS_ANALYSIS {
public:
    const size_t ACC_LONLAT_SKIP  = 5;
    size_t       acc_lonlat_count = 0;

public:
    struct RESULT {
        size_t lat_ac : 1; // Latitudinal acceleration compensation
        size_t lon_ac : 1; // Longitudinal acceleration compensation
        size_t alp_a : 1;  // alpha adaption
        RESULT() { *this = 0ul; }
        RESULT &operator=(size_t r) noexcept {
            *(size_t *)this = r;
            return *this;
        }
        explicit operator bool() const noexcept { return *(bool *)this; }
    };

private:
    static const size_t BUFFER_SIZE = 42;
    std::deque<double>  vel_queue;

private:
    double acc_lon_sm_pre = 0.0;
    double acc_lat_sm_pre = 0.0;

public:
    double dacc_lon_sm = 0.0;
    double dacc_lat_sm = 0.0;

public:
    RESULT operator()(const Eigen::Vector3d &vel_, const Eigen::Vector3d &gyro_, double dt, double acc_lon_sm, double acc_lat_sm);

public:
    // Eigen::Vector3d gyro_bias;
    double latitudinal_acc  = 0;
    double longitudinal_acc = 0;
    double alpha_factor     = 0.01;
};

using AS = class ACC_SMOOTHER {
public:
    static const size_t WINDOW = 12;

private:
    size_t                     idx = 0;
    std::array<double, WINDOW> acc_buf{};

public:
    double operator()(double acc) {
        idx++;
        if (idx >= WINDOW) {
            idx = 0;
        }
        acc_buf[idx]   = acc;
        double acc_sum = 0.0;
        for (auto &acc : acc_buf) {
            acc_sum += acc;
        }
        return acc_sum / WINDOW;
    }
};
} // namespace MSF