#include "kinematic_compensation.h"
#include <functional>
#include <glog/logging.h>
#include <iomanip>

namespace analysis {

/*
  运动学补偿，采用DR消息进行时序补偿
*/

// 压入自车运动信息队列，即DR信息
// 保证数据间隔大于MIN_SAMPLING_GAP
// 保证数据帧在时间上是有序的
void KinematicCompensation::insert(const KinematicData &motion_data) {

    std::unique_lock<std::recursive_mutex> lock(motion_queue_lock);

    // 如果当前数据与上一帧数据的时间间隔小于最小采样间隔，则抛弃
    if (!motion_queue.empty() &&
        std::abs(motion_data.measurement_timestamp - motion_queue.back().measurement_timestamp) < MIN_SAMPLING_GAP) {
        return;
    }

    // 如果新数据的时间戳小于队列末尾时间戳，则抛弃新数据
    if (!motion_queue.empty() &&
        motion_queue.back().measurement_timestamp > motion_data.measurement_timestamp) {
        // // 有可能是上一帧数据有问题，也可能是当前帧数据有问题，如果差的比较多，则抛弃队列末尾数据，避免被一个异常值持续block
        // if (std::abs(motion_queue.back().measurement_timestamp - motion_data.measurement_timestamp) > 10000.0) {
        //     motion_queue.pop_back();
        // }
        LOG(WARNING) << "[time disorder detected] KinematicData";
        return;
    }
    motion_queue.push_back(motion_data);

    // 维持队列长度
    while (motion_queue.size() > MOTION_MAX_QUEUE_SIZE) {
        motion_queue.pop_front();
    }
}

// 依据时间戳，计算运动状态增量
// delta motion, t1 to t2
// delta = motion_t2 - motion_t1
// use_veh_frame 控制最终的输出结果是以t1时刻的自车系为参照，还是以DR坐标系为参照
std::pair<int, KinematicData> KinematicCompensation::delta(double t1, double t2, bool use_veh_frame = false) {

    int           rslt_ = 0x0;
    KinematicData delta_1to2;

    std::unique_lock<std::recursive_mutex> lock(motion_queue_lock);
    if (motion_queue.empty()) {
        LOG(WARNING) << "KinematicCompensation, empty queue";
        rslt_ |= 0x1; // empty queue
        return std::make_pair(rslt_, delta_1to2);
    }

    // 插值，依据两帧KinematicData数据和时间戳，进行前拓、插值、后拓
    auto interpolation_func = [&rslt_, this](const KinematicData &data1, const KinematicData &data2, double t) -> KinematicData {
        KinematicData interpolation_data;

        double dt_left  = t - data1.measurement_timestamp;
        double dt_right = t - data2.measurement_timestamp;
        double dt12     = data2.measurement_timestamp - data1.measurement_timestamp;

        // 前拓超过3倍最小采样间隔，认为无法保证精度
        if (dt_left < -MIN_SAMPLING_GAP * 3) {
            LOG(INFO) << "dt_left: " << std::setprecision(6) << dt_left;
            rslt_ |= 0x2; // [left] out of range
        }

        // 后拓超过3倍最小采样间隔，认为无法保证精度
        if (dt_right > MIN_SAMPLING_GAP * 3) {
            LOG(INFO) << "dt_right: " << std::setprecision(6) << dt_right;
            rslt_ |= 0x4; // [right] out of range
        }

        // 数据帧间隔很小，认为无法保证精度
        // 返回零值
        if (std::abs(dt12) < 0.01) {
            rslt_ |= 0x8; // sample gap too small
            return interpolation_data;
        }

        interpolation_data.measurement_timestamp = t;
        interpolation_data.pos                   = data1.pos + (data2.pos - data1.pos) * dt_left / dt12;
        interpolation_data.vel                   = data1.vel + (data2.vel - data1.vel) * dt_left / dt12;
        interpolation_data.att                   = data1.att.slerp(dt_left / dt12, data2.att);
        interpolation_data.ego_longitude_vel     = data1.ego_longitude_vel + (data2.ego_longitude_vel - data1.ego_longitude_vel) * dt_left / dt12;

        return interpolation_data;
    };

    // 在队列里面找到时间戳的位置，并插值
    auto pin_func = [this, interpolation_func, &rslt_](double t) -> KinematicData {
        // KinematicData的顺序关系由时间戳确定
        auto t_cmp_func = [](const KinematicData &data, double t) -> bool { return data.measurement_timestamp < t; };
        auto lb         = std::lower_bound(motion_queue.begin(), motion_queue.end(), t, t_cmp_func);

        KinematicData motion;
        if (motion_queue.size() == 1) {
            motion = motion_queue.at(0);
            rslt_ |= 0x10; // no enough sample for interpolation
            return motion;
        }
        if (lb == motion_queue.begin()) {
            // 如果是首个元素，则需要使用首个和后一个元素做插值
            motion = interpolation_func(*lb, *(lb + 1), t);
        } else if (lb == motion_queue.end()) {
            // 如果没有满足的元素，则使用最后两个元素做插值
            motion = interpolation_func(*(lb - 2), *(lb - 1), t);
        } else {
            // 寻常情景，使用前后两个元素做插值
            motion = interpolation_func(*(lb - 1), *lb, t);
        }
        return motion;
    };

    auto p1    = pin_func(t1);
    auto p2    = pin_func(t2);
    delta_1to2 = p2 - p1;

    // 返回补偿状态码和结果
    if (use_veh_frame) {
        // 自车系
        Eigen::Matrix3d mat_p1         = p1.att.toRotationMatrix().transpose();
        KinematicData   delta_1to2_veh = delta_1to2;
        delta_1to2_veh.pos             = mat_p1 * delta_1to2.pos;
        delta_1to2_veh.vel             = mat_p1 * delta_1to2.vel;

        return std::make_pair(rslt_, delta_1to2_veh);

    } else {
        // DR 坐标系
        return std::make_pair(rslt_, delta_1to2);
    }
}

void KinematicCompensation::crop_by_timestamp(double min, double max, double gap) {
    std::unique_lock<std::recursive_mutex> lock(motion_queue_lock);
    while (motion_queue.front().measurement_timestamp < min - gap) {
        motion_queue.pop_front();
    }
    while (motion_queue.back().measurement_timestamp > max + gap) {
        motion_queue.pop_back();
    }
}

} // namespace analysis