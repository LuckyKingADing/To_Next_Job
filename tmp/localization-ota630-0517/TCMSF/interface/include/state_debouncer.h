#pragma once

#include <functional>
#include <iostream>
#include <optional>
#include <unordered_map>

namespace byd {
namespace tcmsf {

template <typename StateType>
class TimestampedStateDebouncer {
public:
    using Callback = std::function<void(StateType state, double timestamp, bool is_stable)>;

    /**
     * @brief 构造函数
     * @param target_state 需要防抖的特定目标状态
     * @param callback 状态变化回调
     * @param debounce_duration 防抖持续时间（基于时间戳单位）
     */
    TimestampedStateDebouncer(StateType target_state, double debounce_duration, Callback callback = {}) :
        target_state_(target_state), debounce_duration_(debounce_duration), callback_(callback), current_state_(), is_target_stable_(false), last_debounce_start_time_(0.0) {
    }

    /**
     * @brief 处理带时间戳的状态数据帧
     * @param new_state 新状态值
     * @param timestamp 状态时间戳
     */
    void process_state_frame(StateType new_state, double timestamp) {
        // 处理状态变化逻辑
        handle_state_change(new_state, timestamp);
    }

    /**
     * @brief 获取当前稳定状态
     * @return 当前稳定状态，如果不稳定返回std::nullopt
     */
    std::optional<StateType> get_stable_state() const {
        if (is_target_stable_) {
            return target_state_;
        }
        return std::nullopt;
    }

    /**
     * @brief 检查目标状态是否稳定
     * @return 是否处于稳定的目标状态
     */
    bool is_target_state_stable() const {
        return is_target_stable_;
    }

    /**
     * @brief 设置防抖持续时间
     * @param duration 新的防抖持续时间
     */
    void set_debounce_duration(double duration) {
        debounce_duration_ = duration;
    }

    /**
     * @brief 强制状态转换（用于外部干预）
     * @param new_state 新状态
     * @param timestamp 时间戳
     * @param immediate 是否立即生效（跳过防抖）
     */
    void force_state_transition(StateType new_state, double timestamp, bool immediate = false) {
        if (immediate) {
            // 立即切换状态，跳过防抖逻辑
            StateType previous_stable_state = current_state_;
            current_state_                  = new_state;
            is_target_stable_               = (new_state == target_state_);

            if (callback_) {
                callback_(new_state, timestamp, is_target_stable_);
            }

            // 重置防抖计时
            if (new_state == target_state_) {
                last_debounce_start_time_ = timestamp;
            }
        } else {
            // 正常处理状态帧
            process_state_frame(new_state, timestamp);
        }
    }

private:
    void handle_state_change(StateType new_state, double timestamp) {
        StateType previous_state = current_state_;
        current_state_           = new_state;

        if (new_state == target_state_) {
            // 进入目标状态，需要防抖判断
            handle_enter_target_state(timestamp);
        } else {
            // 退出目标状态，立即处理
            handle_exit_target_state(previous_state, timestamp);
        }
    }

    void handle_enter_target_state(double timestamp) {
        if (!is_target_stable_) {
            // 第一次检测到目标状态或之前不稳定
            if (last_debounce_start_time_ == 0.0) {
                // 开始防抖计时
                last_debounce_start_time_ = timestamp;
            } else {
                // 检查是否已经稳定持续了足够长时间
                double duration_in_target = timestamp - last_debounce_start_time_;
                if (duration_in_target >= debounce_duration_) {
                    // 状态稳定，确认进入目标状态
                    is_target_stable_ = true;
                    if (callback_) {
                        callback_(target_state_, timestamp, true);
                    }
                }
                // 否则继续等待，不触发回调
            }
        }
        // 如果已经是稳定状态，不需要额外处理
    }

    void handle_exit_target_state(StateType previous_state, double timestamp) {
        // 立即退出目标状态
        if (is_target_stable_ || last_debounce_start_time_ > 0.0) {
            bool was_stable           = is_target_stable_;
            is_target_stable_         = false;
            last_debounce_start_time_ = 0.0; // 重置防抖计时

            if (callback_ && was_stable) {
                // 只有之前是稳定状态才触发退出回调
                callback_(current_state_, timestamp, false);
            }
        }
    }

    StateType target_state_;      // 目标防抖状态
    Callback  callback_;          // 状态变化回调
    double    debounce_duration_; // 防抖持续时间

    StateType current_state_;            // 当前状态
    bool      is_target_stable_;         // 目标状态是否稳定
    double    last_debounce_start_time_; // 最后一次开始防抖的时间戳
};

} // namespace tcmsf
} // namespace byd
