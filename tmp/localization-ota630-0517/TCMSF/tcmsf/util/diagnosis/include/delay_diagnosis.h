#pragma once
#include <cmath>
#include <cstdint>
#include <functional>

namespace byd {
namespace tcmsf {
namespace diagnosis {

class SensorDelayDiagnosis {

private:
    double valid_bound  = 1.0;
    double filter_bound = 0.2;

private:
    static constexpr uint64_t MAX_MEAN_SIZE = 1000;
    uint64_t                  mean_num      = 0;
    double                    mean_delay    = 0.0;

public:
    SensorDelayDiagnosis() {}
    SensorDelayDiagnosis(double valid_bound_, double filter_bound_) {
        set_bound(valid_bound_, filter_bound_);
    }
    void set_bound(double valid_bound_, double filter_bound_) {
        valid_bound  = valid_bound_;
        filter_bound = filter_bound_;
    }

private:
    bool mean(double dt_) {

        if (std::abs(dt_) > valid_bound) {
            return false;
        }

        mean_num++;
        if (mean_num == 0) {
            mean_num = 1;
        }
        if (mean_num >= MAX_MEAN_SIZE) {
            mean_num = MAX_MEAN_SIZE;
        }
        mean_delay = mean_delay + (1.0 / (mean_num + 1e-5)) * (dt_ - mean_delay);
        return mean_num >= MAX_MEAN_SIZE;
    }

public:
    bool diagnosis(double dt_, bool mean_update_, std::function<bool(double)> block_func = nullptr) {
        bool mean_rslt = false;
        if (mean_update_) {
            mean_rslt = mean(dt_);
        }
        if (mean_rslt) { // 均值有效时的过滤函数
            if (std::abs(dt_ - mean_delay) > filter_bound) {
                return false;
            }
        } else if (block_func) { // 均值无效时的过滤函数
            if (block_func(dt_)) {
                return false;
            }
        }

        // 默认通过
        return true;
    }

    double get_mean_delay() {
        return mean_delay;
    }
};

} // namespace diagnosis
} // namespace tcmsf
} // namespace byd