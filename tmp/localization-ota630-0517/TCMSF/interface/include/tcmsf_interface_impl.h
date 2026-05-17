#pragma once

#include "delay_diagnosis.h"
#include "fail_detect_interface.h"
#include "kinematic_compensation.h"
#include "modules/localization/src/TCMSF/tcmsf/processor/include/processor_interface.h"
#include "modules/localization/src/TCMSF/tcmsf/sensor/base/include/base_type.h"
#include "modules/localization/src/TCMSF/tcmsf/sensor/base/include/tcmsf_config.h"
#include "modules/localization/src/TCMSF/tcmsf/signal_process/signal_filter/include/signal_filter.h"
#include "modules/localization/src/TCMSF/tcmsf/util/geo/include/geo_trans.h"
#include "modules/localization/src/TCMSF/third_party/concurrentqueue-1.0.4/include/blockingconcurrentqueue.h"
#include "modules/localization/src/TCMSF/troublemaker/interface/include/troublemaker_interface.h"
#include "persistence_vehicle_info.h"
#include "tcmsf_interface.h"

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>

#include "Eigen/Dense"

#include "state_debouncer.h"

namespace byd {
namespace tcmsf {

using namespace moodycamel;

// TCMSF针对具体传感器组成的实现
class TCMSF_IMPL : public TCMSF {

public:
    TCMSF_IMPL(const std::string &lever_file_);
    ~TCMSF_IMPL();

private:
    const size_t GNSS_MAX_QUEUE_SIZE    = 2;
    const size_t VEHICLE_MAX_QUEUE_SIZE = 10;
    const size_t IMU_MAX_QUEUE_SIZE     = 10;
    const size_t MAP_MAX_QUEUE_SIZE     = 3;
    const size_t VISION_MAX_QUEUE_SIZE  = 2;
    const size_t DB_MAX_QUEUE_SIZE      = 2;

private:
    KinematicCompensation kinematic_compensation;
    geo::Trans            geotrans;

private:
    std::unique_ptr<processor::Processor> fusion_ptr_ = nullptr;

private:
    std::atomic<bool> fusion_shall_exit;
    std::thread       fusion_thread;

private:
    MSF::ImuData  pre_imu_data;
    MSF::GnssData pre_gnss_data;
    MSF::StatePtr msf_state_ptr = nullptr; // 融合定位节点的主要状态

private:
    config::Parameters       &parameters_sgt = config::Parameters::getInstance(config::TCMSF_CONFIG_FILE_DIR_);
    persistence::VehicleInfo &vehinfo_sgt    = persistence::VehicleInfo::getInstance(parameters_sgt.get_vehicle_info_file_path(), parameters_sgt.get_is_persistence_enable());

private:
    // 传感器消息队列
    // 采用无锁并发队列，支持多线程enqueue dequeue
    // 其中IMU队列采用带阻塞功能的实现，用于控制融合定位的调度
    // 其他队列均为非阻塞队列
    moodycamel::BlockingConcurrentQueue<MSF::ImuData>  imu_queue;
    moodycamel::ConcurrentQueue<MSF::GnssData>         gnss_queue;
    moodycamel::ConcurrentQueue<MSF::VehicleData>      vehicle_queue;
    moodycamel::ConcurrentQueue<MSF::MapPosData>       map_pos_queue;
    moodycamel::ConcurrentQueue<MSF::VisionFusionData> vision_fusion_queue;
    moodycamel::ConcurrentQueue<MSF::DbData>           db_queue;

public:
    // 消息队列生产者
    virtual int insert_msg(std::shared_ptr<Imu>) override final;
    virtual int insert_msg(std::shared_ptr<Gps>) override final;
    virtual int insert_msg(std::shared_ptr<VehInfo>) override final;
    virtual int insert_msg(std::shared_ptr<LocalizationEstimate>) override final;
    virtual int insert_msg(std::shared_ptr<VFResult>) override final;
    virtual int insert_msg(std::shared_ptr<SDMapMatchResult>) override final;
    virtual int insert_msg(std::shared_ptr<DriveBoundary>) override final;

public:
    // 输出结果
    virtual int output_msg(std::shared_ptr<Pose>) override final;

    uint32_t sequence_num;

private:
    // 轮速低通滤波器
    // 轮速使用轮脉冲计数的方式计算速度，有很大的抖动噪声
    // 尝试使用低通滤波器来处理
    // 这个低通滤波器会产生大约160ms的相位延迟
    MSF::VLPF<2>            veh_lpf        = MSF::VLPF<2>(100.0, 1.0);
    static constexpr double veh_lpf_delay_ = -0.16;

private:
    // 使用IMU和轮速进行DR递推
    MSF::KinematicDataPtr inner_motion_state_ptr_ = nullptr; // 内部递推状态 RFU
    MSF::KinematicDataPtr cur_dr_ptr_             = nullptr; // 输出状态 FLU

    std::function<MSF::KinematicData(const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc, const Eigen::Vector3d &vel, double measurement_timestamp)> dead_reckoning = nullptr;

private:
    // 消息队列消费者
    void RegisterConsumer();

private:
    // 已获取到的最新的传感器信息
    MSF::ImuData          cur_imu_data;
    MSF::GnssData         cur_gnss_data;
    MSF::VehicleData      cur_veh_data;
    MSF::VisionFusionData cur_vis_data;
    int                   last_status_valid = 0; //上一次定位有效性：0-无效，1-fix，2-fix以后的非fix，3-无效以后的非fix//只有1|2有效
    MSF::MapPosData       cur_map_data;

    // 消息队列消费函数
    // 可能队列里面没有消息，需要try消费
    std::function<bool(bool &)> try_fuse_imu  = nullptr;
    std::function<bool()>       try_fuse_gnss = nullptr;
    std::function<bool()>       try_fuse_veh  = nullptr;
    std::function<bool()>       try_fuse_vis  = nullptr;
    std::function<bool()>       try_fuse_map  = nullptr;
    std::function<bool()>       try_fuse_db   = nullptr;

public:
    // 在线版本采用独立线程调度队列消费者
    virtual int start_fusion_daemon(std::function<void()>) override final;

    // 离线版本手动单步调度队列消费者
    virtual int offline_mode_step(std::function<void()>) override final;

private:
    diagnosis::SensorDelayDiagnosis gnss_delay_diagnosis;
    diagnosis::SensorDelayDiagnosis vehicle_delay_diagnosis;
    diagnosis::SensorDelayDiagnosis vision_delay_diagnosis;

private:
    std::unique_ptr<fail_detect::IMUFailDetect> imu_fail_detect_ptr = nullptr;

    double                  gyro_z_scale_               = 0.0;
    static constexpr double GYRO_SCALE_MEAN_WINDOW_SIZE = 100.0;

private:
    std::unique_ptr<TroubleMaker::SensorModifyBase> sensor_modify_ptr_ = nullptr;

private:
    uint64_t imu_msg_count_;

private:
    TimestampedStateDebouncer<MSF::State::AlignType> coarse_align_debouncer{MSF::State::AlignType::COARSE_ALIGN, 3.0};
};

} // namespace tcmsf
} // namespace byd