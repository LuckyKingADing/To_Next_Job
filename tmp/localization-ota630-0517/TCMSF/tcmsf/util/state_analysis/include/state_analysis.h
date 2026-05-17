#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <vector>

namespace statistics {
struct StateInfo {
    double time;
    int    state;
    StateInfo() {
        time  = 0.0;
        state = 0;
    }
    StateInfo(double t_, int s_) {
        time  = t_;
        state = s_;
    }
};

class StateStatistics {
private:
    std::deque<StateInfo>        stateq;
    uint64_t                     STATE_SEQ_MAX_SIZE = 0;
    std::map<uint64_t, uint64_t> statem;

public:
    void     insert(const StateInfo &);
    uint64_t state_count(uint64_t);
    void     clear();
    void     set_state_max_num(uint64_t);
    uint64_t total_state_count();

    // 只会降采样，不会上采样，如果本身间隔大的话，间隔会超过秒
    void insert_per_sec(const StateInfo &);
};

class StateMeanStd {
private:
    std::deque<Eigen::Vector3d> state_queue;
    uint64_t                    MAX_QUEUE_SIZE = 0;
    Eigen::Vector3d             mean_{0.0, 0.0, 0.0};
    Eigen::Vector3d             std_{0.0, 0.0, 0.0};

public:
    void insert(const Eigen::Vector3d &state_) {
        while (state_queue.size() > MAX_QUEUE_SIZE) {
            state_queue.pop_front();
        }
        state_queue.push_back(state_);
    }
    bool calc_mean_std() {
        if (state_queue.size() == (MAX_QUEUE_SIZE + 1)) {
            mean_.setZero();
            std_.setZero();
            for (auto &s_ : state_queue) {
                mean_ = mean_ + s_;
            }
            mean_ = mean_ / (double)(MAX_QUEUE_SIZE + 1);
            for (auto &s_ : state_queue) {
                std_ = std_ + (s_ - mean_).array().pow(2).matrix();
            }
            std_ = std_ / (double)(MAX_QUEUE_SIZE + 1);
            std_ = std_.cwiseSqrt();
            return true;
        } else {
            return false;
        }
    }
    Eigen::Vector3d mean() {
        return mean_;
    }
    Eigen::Vector3d std() {
        return std_;
    }
    void clear() {
        state_queue.clear();
    }
    void set_max_size(uint64_t size_) {
        MAX_QUEUE_SIZE = size_;
    }
};

struct Vec3dInfo {
    double          time;
    Eigen::Vector3d state;
    Vec3dInfo() {
        time  = 0.0;
        state = Eigen::Vector3d::Zero();
    }
    Vec3dInfo(double t_, Eigen::Vector3d s_) {
        time  = t_;
        state = s_;
    }
};
class StateRecursiveStdMean {
private:
    static constexpr uint64_t WINDOW = 20;

    uint64_t rec_count;

    Vec3dInfo pre_state;
    Vec3dInfo cur_state;
    Vec3dInfo mean_;
    Vec3dInfo var_;

public:
    StateRecursiveStdMean() {
        reset();
    }
    Vec3dInfo get_mean() {
        return mean_;
    }
    Vec3dInfo get_std() {
        Vec3dInfo std_ = var_;
        std_.state     = var_.state.cwiseSqrt();
        return std_;
    }
    void reset() {
        rec_count = 0;
        pre_state.state.setZero();
        cur_state.state.setZero();
        mean_.state.setZero();
        var_.state.setZero();
        pre_state.time = 0.0;
        cur_state.time = 0.0;
        mean_.time     = 0.0;
        var_.time      = 0.0;
    }
    uint64_t get_window() {
        return WINDOW;
    }

public:
    bool operator()(const Vec3dInfo &v3info) {
        // GNSS帧间隔过大，重置
        if (std::abs(pre_state.time - cur_state.time) > 2.0 && rec_count != 0) {
            reset();
            return false;
        }

        // 这里不约束rec_count的溢出行为
        // 问题不大
        rec_count++;

        cur_state = v3info;
        pre_state = cur_state;

        if (rec_count == 0) {
            return false;
        }

        if (rec_count <= WINDOW) {

            double n = (double)rec_count;

            var_.state  = (n - 1) / (n * n) * ((v3info.state - mean_.state).cwiseAbs2()) + (n - 1) / n * var_.state;
            mean_.state = mean_.state + (v3info.state - mean_.state) / n;

            if (rec_count == WINDOW) {
                return true;
            }
        } else {
            reset();
        }
        return false;
    }
};

} // namespace statistics