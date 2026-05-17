/*


▄▄▄█████▓    ▄████▄      ███▄ ▄███▓     ██████      █████▒
▓  ██▒ ▓▒   ▒██▀ ▀█     ▓██▒▀█▀ ██▒   ▒██    ▒    ▓██   ▒
▒ ▓██░ ▒░   ▒▓█    ▄    ▓██    ▓██░   ░ ▓██▄      ▒████ ░
░ ▓██▓ ░    ▒▓▓▄ ▄██▒   ▒██    ▒██      ▒   ██▒   ░▓█▒  ░
  ▒██▒ ░    ▒ ▓███▀ ░   ▒██▒   ░██▒   ▒██████▒▒   ░▒█░
  ▒ ░░      ░ ░▒ ▒  ░   ░ ▒░   ░  ░   ▒ ▒▓▒ ▒ ░    ▒ ░
    ░         ░  ▒      ░  ░      ░   ░ ░▒  ░ ░    ░
  ░         ░           ░      ░      ░  ░  ░      ░ ░
            ░ ░                ░            ░
            ░


*/

#include "tcmsf_interface_impl.h"

#include "rigid_transform.h"

#include <algorithm>
#include <filesystem>
#include <pthread.h>

#include "fmt/format.h"

#include "modules/localization/src/TCMSF/tcmsf/util/calc/include/calc.h"

// #include "motion_status.h"
#include "processor_debug.h"

#include "cyber/task/task.h"

#ifdef USE_DIMW_LIBS
#include "cfgmgr/cfgmgr.h"
#endif

namespace byd {
namespace tcmsf {

TCMSF_IMPL::TCMSF_IMPL(const std::string &lever_file_) :
    fusion_shall_exit(false) {

    if (lever_file_ != "") {

        std::string lever_file_path_ = lever_file_;

        // 首先解析默认的imu配置
        auto default_cfg_parsed_ = parameters_sgt.update_vehicle_lever(lever_file_path_);
        AINFO << " parse default imu cfg: " << lever_file_path_ << " | result: " << default_cfg_parsed_;

#ifdef USE_DIMW_LIBS
        // 这里根据底软的信息判断IMU的类型，如果是气囊IMU，则需要更替配置文件
        {
            AINFO << "use DIMW to determine imu type";

            dimw::cfgmgr::BasicInfo basic_info;

            bool ret = dimw::cfgmgr::getBasicInfo(basic_info);
            if (ret) {
                if (basic_info.sensor.imu.imuType == dimw::cfgmgr::ImuType::AirBagIMU) {
                    AINFO << "air bag imu, update imu config file path";

                    namespace fs                  = std::filesystem;
                    fs::path old_imu_cfg_fs_path_ = lever_file_path_;
                    fs::path new_imu_cfg_fs_path_ = old_imu_cfg_fs_path_.parent_path() / "imu_config_acu.json";

                    lever_file_path_ = new_imu_cfg_fs_path_;

                    AINFO << "use air bag imu config file path: " << lever_file_path_;
                } else {
                    AINFO << "not air bag imu, use original imu config file path: " << lever_file_path_;
                }
            } else {
                AERROR << "DIMW getBasicInfo failed!";
            }
        }
#endif

        // 为了保证兼容性，这里保留解析判断是否acu_imu之后的imu配置
        auto parsed_ = parameters_sgt.update_vehicle_lever(lever_file_path_);
        AINFO << " update lever info from file: " << lever_file_path_ << " | result: " << parsed_;
    }

    fusion_ptr_                      = processor::Processor::create();
    msf_state_ptr                    = std::make_shared<MSF::State>();
    msf_state_ptr->lever_imu2vehicle = parameters_sgt.get_lever_imu2vehicle();
    msf_state_ptr->lever_imu2gnss    = parameters_sgt.get_lever_imu2gnss();
    auto veh_state_                  = vehinfo_sgt.get_state();

    // 这里防护一下
    // 万一读到了NAN值，则置零，避免后续污染整个滤波器的状态
    if (veh_state_.isNaN()) {
        AWARN << "vehicle state NaN detected, set to zero";
        veh_state_.setZero();
    }

    inner_motion_state_ptr_ = std::make_shared<MSF::KinematicData>();
    cur_dr_ptr_             = std::make_shared<MSF::KinematicData>();

    msf_state_ptr->vehicle_bias.x() = veh_state_.pitch_misalignment;
    msf_state_ptr->vehicle_bias.y() = veh_state_.wheel_speed_scale_error;

    // 感觉这个yaw安装角偏差，收敛的并不准
    // 可能持久化的值，并不一定比默认值（零值）好
    // 这里暂时先去掉这个参数的持久化，在观察观察
    // msf_state_ptr->vehicle_bias.z() = veh_state_.yaw_misalignment;
    msf_state_ptr->vehicle_bias.z() = 0.0;

    sequence_num   = 1;
    imu_msg_count_ = 1;
    MSF::StateConstrain(
        msf_state_ptr->vehicle_bias,
        {parameters_sgt.get_constrain_euler_angle_imu2vehicle().x(),
         parameters_sgt.get_constrain_wheel_speed_bias(),
         parameters_sgt.get_constrain_euler_angle_imu2vehicle().z()});

    // 这里同步更新下安装角偏差四元数
    msf_state_ptr->q_imu2vehicle = INS::euler2quaternion({msf_state_ptr->vehicle_bias.x(), 0.0, msf_state_ptr->vehicle_bias.z()});

    gnss_delay_diagnosis.set_bound(parameters_sgt.get_sensor_delay_valid_bound(), parameters_sgt.get_sensor_delay_filter_bound_gnss());
    vehicle_delay_diagnosis.set_bound(parameters_sgt.get_sensor_delay_valid_bound(), parameters_sgt.get_sensor_delay_filter_bound_vehicle());
    vision_delay_diagnosis.set_bound(parameters_sgt.get_sensor_delay_valid_bound(), parameters_sgt.get_sensor_delay_filter_bound_vision());

    sensor_modify_ptr_ = TroubleMaker::SensorModifyBase::create();

    imu_fail_detect_ptr = fail_detect::IMUFailDetect::create();

    RegisterConsumer();

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
    AWARN << "---- DEBUG MODE ENABLED, THERE WILL BE SOME FILE IO !!! ----";
#endif

#if (defined __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES) && (defined __TCMSF_ENABLE_TROUBLE_MAKER_)
    AWARN << "---- TROUBLE TEST MODE ENABLED, MODIFIED RAW SENSOR DATA WILL BE USED !!! ----";
#endif
}

TCMSF_IMPL::~TCMSF_IMPL() {
    // 退出状态置为true
    fusion_shall_exit.store(true);
    if (fusion_thread.joinable()) {
        // 等待融合主线程退出
        fusion_thread.join();
    }
    AINFO << "TCMSF_IMPL exit";
}

// 注册消息消费函数
void TCMSF_IMPL::RegisterConsumer() {
    /*
      注册内部DR函数
      内部DR函数与外部DR模块的功能基本一致，能够保证平面姿态稳定性，主要区别在速度递推上，内部DR为了能够进行速度的补偿（如轮速）
      速度采用IMU ACC积分得到，也就是内部DR的速度只保证相对精度，长期结果是发散的，但是位置的递推不会使用积分得到的速度，所以位置能够保证相对精度
    */

    // 要求该函数的输入均是修正过的量
    // 该DR为IMU系下的递推
    // IMU系与车体系之间有个安装角偏差，在使用的时候，需要注意这个区别。
    dead_reckoning = [this](const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc, const Eigen::Vector3d &vel_, double measurement_timestamp) -> MSF::KinematicData {
        Eigen::Vector3d    pos0               = inner_motion_state_ptr_->pos;
        Eigen::Quaterniond att0               = inner_motion_state_ptr_->att;
        Eigen::Vector3d    vel0               = inner_motion_state_ptr_->vel;
        double             ego_longitude_vel0 = inner_motion_state_ptr_->ego_longitude_vel;
        double             dt                 = measurement_timestamp - inner_motion_state_ptr_->measurement_timestamp;
        if (std::abs(dt) > 10.0) {
            // 认为dt大于10sec是错误的
            dt = 0.0;
        }
        Eigen::Vector3d theta = gyro * dt;

        bool fine_state_ = MSF::process_control_sgt.msf_align_type == MSF::process_control_sgt.ALIGNED ||
                           MSF::process_control_sgt.msf_align_type == MSF::process_control_sgt.FINE_ALIGN ||
                           MSF::process_control_sgt.msf_align_type == MSF::process_control_sgt.COARSE_ALIGN;

        // 更新内部DR状态
        // 使用加速度更新速度
        // 此处速度为一个发散的积分量，只用于短时间内的速度补偿
        if (fine_state_) {
            vel0               = vel0 + att0.toRotationMatrix() * (acc + msf_state_ptr->gravity) * dt;
            ego_longitude_vel0 = ego_longitude_vel0 + (acc + msf_state_ptr->gravity).y() * dt;
        } else {
            // 如果融合状态很差的话，假设车辆处于水平状态
            Eigen::Vector3d g0_{0.0, 0.0, -9.8};
            vel0               = vel0 + att0.toRotationMatrix() * (acc + g0_) * dt;
            ego_longitude_vel0 = ego_longitude_vel0 + (acc + g0_).y() * dt;
        }

        // 使用角速度更新姿态
        att0 = att0 * INS::rv2q(theta);
        att0.normalize();
        // 使用速度更新位置
        Eigen::Vector3d vel = att0.toRotationMatrix() * vel_;
        pos0                = pos0 + vel * dt;

        // 更新内部DR状态
        inner_motion_state_ptr_->measurement_timestamp = measurement_timestamp;
        inner_motion_state_ptr_->att                   = att0;
        inner_motion_state_ptr_->pos                   = pos0;
        inner_motion_state_ptr_->vel                   = vel0;
        inner_motion_state_ptr_->ego_longitude_vel     = ego_longitude_vel0;
        // 内部DR坐标系定义为右-前-上
        // 为了与外部一致，转换为前-左-上输出
        MSF::KinematicData motion;
        motion.measurement_timestamp = measurement_timestamp;
        Eigen::Quaterniond att{INS::frame_trans.FLU2RFU.transpose() * att0.toRotationMatrix() * INS::frame_trans.FLU2RFU};
        Eigen::Vector3d    pos_world = INS::frame_trans.FLU2RFU.transpose() * pos0;
        Eigen::Vector3d    vel_world = INS::frame_trans.FLU2RFU.transpose() * vel0;
        motion.att                   = att;
        motion.pos                   = pos_world;
        motion.vel                   = vel_world;
        motion.ego_longitude_vel     = ego_longitude_vel0;

        return motion;
    };

    /*
      注册预测更新及量测更新函数【消息队列消费函数】
      每个消息消费函数执行的时候，会检查消息队列里面是否有消息，有消息则消费
    */

    try_fuse_imu = [this](bool &msf_ready) -> bool {
        // 使用IMU消息驱动整个融合定位的更新
        // 从队列中获取IMU消息，若无消息，阻塞 0.1 s
        if (!imu_queue.wait_dequeue_timed(cur_imu_data, 100000)) {
            return false;
        }
        MSF::ImuDataPtr imu_ptr        = std::make_shared<MSF::ImuData>();
        double          dt             = cur_imu_data.measurement_timestamp - pre_imu_data.measurement_timestamp;
        imu_ptr->measurement_timestamp = cur_imu_data.measurement_timestamp;
        imu_ptr->publish_timestamp     = cur_imu_data.publish_timestamp;
        imu_ptr->sequence_num          = cur_imu_data.sequence_num;

        // AINFO << std::fixed << std::setprecision(4) << "imu_ptr " << imu_ptr->measurement_timestamp << " " << imu_ptr->measurement_timestamp - process_control_sgt.last_valid_dt;

        // 依据IMU前后两帧数据，梯形积分得到速度和角度增量
        // 后续SINS的更新，基于增量。
        imu_ptr->acc  = (pre_imu_data.acc + cur_imu_data.acc) * dt / 2.0;
        imu_ptr->gyro = (pre_imu_data.gyro + cur_imu_data.gyro) * dt / 2.0;

        pre_imu_data = cur_imu_data;

        // IMU帧间时间差判断，异常时不进行预测更新
        if (std::abs(dt) < 1e-3 || std::abs(dt) > 0.5) {
            AWARN << "abnormal IMU timestamp gap.";
            return false;
        } else {

            {
                imu_msg_count_++;
                // 每隔一段时间触发一次参数持久化的写入
                // 30min 并且 里程超过 5km 并且 稳定系数大于0.5
                static constexpr uint64_t PERSISTENCE_VEHICLE_INFO_ = 100 * 60 * 30; // 30 min
                if (imu_msg_count_ % PERSISTENCE_VEHICLE_INFO_ == 0 && msf_state_ptr->mileage > 5000.0 && msf_state_ptr->state_stability_index_ratio_ > 0.5) {
                    vehinfo_sgt.async_persistence_once();
                }
            }

            msf_ready = fusion_ptr_->ProcessImuData(imu_ptr, dt, msf_state_ptr);

            // 姿态参考，考虑到该算法的核心是imu的加计信息，轮速起到辅助作用，故放置在imu信息更新过程中
            fusion_ptr_->AttitudeReference(cur_imu_data, cur_veh_data, dt, msf_state_ptr, inner_motion_state_ptr_);

            if (parameters_sgt.get_use_internal_dr_info()) {
                // 使用内部DR，可控性更好。
                // 随着内部DR功能完善，不建议再使用外部DR了。
                // 依据IMU和轮速信息，内部维护DR状态递推序列
                Eigen::Vector3d gyro_ = cur_imu_data.gyro;
                Eigen::Vector3d vel_{0.0, (cur_veh_data.speed_rl + cur_veh_data.speed_rr) / 2.0, 0.0};
                Eigen::Vector3d acc_ = cur_imu_data.acc;

                if (msf_state_ptr->vehicle_bias.norm() == 0.0 &&
                    (MSF::process_control_sgt.msf_align_type != MSF::process_control_sgt.ALIGNED && MSF::process_control_sgt.msf_align_type != MSF::process_control_sgt.FINE_ALIGN)) {
                    // 刚启动的时候，不进行精细化的补偿
                } else {
                    // 这里做个DR递推速度信息源的选择
                    // 比如车辆出现了明显打滑，这个时候使用轮速递推会出现问题
                    // 此时可以使用融合定位的速度进行递推
                    // 但是通常来讲，轮速是个稳定的速度来源，常规场景下使用轮速会更具有鲁棒性
                    if (cur_veh_data.tire_slip) {
                        // 使用融合后的速度递推位置
                        // 这里需要将速度从IMU系（导航系下表示）转换到自车坐标系下
                        vel_ = msf_state_ptr->GetEgoVel();
                    } else {
                        // 使用轮速递推位置
                        // 进行精细化的补偿。
                        vel_ = (1.0 + msf_state_ptr->wheel_spd_scale_bias_) * vel_;
                    }

                    // 修正gyro零偏
                    // 这里额外加一个重力参考的修正，避免DR在平面上的姿态发散
                    gyro_ = gyro_ - msf_state_ptr->gyro_bias + msf_state_ptr->dr_phi_b_gravity / dt;
                    // 修正acc零偏
                    acc_ = acc_ - msf_state_ptr->acc_bias;
                    // 修正安装角
                    Eigen::Matrix3d T_vehicle2imu = msf_state_ptr->q_imu2vehicle.toRotationMatrix().transpose();
                    vel_                          = T_vehicle2imu * vel_;
                }
                // 为了与外部DR一致，兼容性考虑，dead_reckoning函数输出的轴向定义是FLU
                *cur_dr_ptr_ = dead_reckoning(gyro_, acc_, vel_, imu_ptr->measurement_timestamp);
                kinematic_compensation.insert(*cur_dr_ptr_);

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
                {
                    auto dr_eulr_ = INS::quaternion2euler(cur_dr_ptr_->att);
                    auto dr_str =
                        fmt::format("{:>14.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>14.10f},{:>14.10f},{:>6.3f}\n", //

                                    cur_dr_ptr_->measurement_timestamp,  // 1
                                    cur_dr_ptr_->pos.x(),                // 2
                                    cur_dr_ptr_->pos.y(),                // 3
                                    cur_dr_ptr_->pos.z(),                // 4
                                    cur_dr_ptr_->vel.x(),                // 5
                                    cur_dr_ptr_->vel.y(),                // 6
                                    cur_dr_ptr_->vel.z(),                // 7
                                    dr_eulr_.x() * 180 / M_PI,           // 8
                                    dr_eulr_.y() * 180 / M_PI,           // 9
                                    dr_eulr_.z() * 180 / M_PI,           // 10
                                    msf_state_ptr->lla.x() * 180 / M_PI, // 11
                                    msf_state_ptr->lla.y() * 180 / M_PI, // 12
                                    msf_state_ptr->lla.z()               // 13
                        );
                    if (msf_state_ptr->lla.x() == 0.0) {
                    } else {
                        debug::debug_sgt.inner_dr_state.line(dr_str);
                    }
                }
#endif

                auto dt_imu_veh = std::abs(cur_veh_data.measurement_timestamp - cur_imu_data.measurement_timestamp);
                if (std::abs(dt_imu_veh) > 0.3) {
                    AWARN_EVERY(100) << "vehicle not sync in dead reckoning, latest dt: " << std::setprecision(6) << dt_imu_veh;
                }

                if (false) { // 暂时不启用。
                    // 只有在使用内部DR的时候，才进行故障检测
                    imu_fail_detect_ptr->insert_kinematics_info(cur_dr_ptr_, msf_state_ptr, imu_ptr);
                    double delta_heading_ref_ = 0.0, delta_heading_ = 0.0, scale_ = 0.0;
                    if (imu_fail_detect_ptr->calc_gyro_scale_z(delta_heading_ref_, delta_heading_, scale_) //
                        && msf_state_ptr->zupt_imu_bias_estimate_ok_count > 1                              //
                    ) {
                        gyro_z_scale_ = gyro_z_scale_ * ((GYRO_SCALE_MEAN_WINDOW_SIZE - 1.0) / GYRO_SCALE_MEAN_WINDOW_SIZE) + scale_ * (1.0 / GYRO_SCALE_MEAN_WINDOW_SIZE);
                        AERROR_IF(std::abs(gyro_z_scale_) > 0.02) << "[imu fail detect] imu gyro z scale error estimated: " << gyro_z_scale_;
                    }
                    // AINFO << "gyro z scale: " << gyro_z_scale_;
                }
            }
        }
        return true;
    };

    try_fuse_gnss = [this]() -> bool {
        pre_gnss_data = cur_gnss_data;
        // GNSS频率远低于IMU，预期延迟100ms以上
        // 使用最新的GNSS消息进行量测更新
        while (gnss_queue.size_approx() > 1) {
            gnss_queue.try_dequeue(cur_gnss_data);
        }
        if (gnss_queue.try_dequeue(cur_gnss_data)) {

            if (cur_gnss_data.status == 0) {
                // 剔除无效GNSS消息
                msf_state_ptr->rtk_status = 0;
                return false;
            }

            double cur_dt_ = cur_gnss_data.measurement_timestamp - cur_imu_data.measurement_timestamp;
            if (parameters_sgt.get_sensor_delay_diagnosis_enable()) {
                if (!gnss_delay_diagnosis.diagnosis(cur_dt_, cur_gnss_data.vel.norm() > 5.0, [](double dt_) -> bool {if(dt_>0){AWARN << "GNSS is newer than IMU"; return true;}else{return false;} })) {
                    double delay_mean_ = gnss_delay_diagnosis.get_mean_delay();
                    AWARN_EVERY(10) << "ABNORMAL GNSS delay: " << cur_dt_ << " mean: " << delay_mean_;
                    // 这里放宽过滤条件，暂时只做个检查。有一些高负载场景，GNSS消息延迟大似乎也比较常见，避免过度剔除数据。
                    // 因为做了时间补偿，稍大一点的延迟应该也还好。
                    // return false;
                    if (std::fabs(cur_dt_) > 3.0 * std::fabs(delay_mean_)) {
                        // 大于三倍均值，认为是错误的消息，剔除
                        AWARN_EVERY(10) << "cur gnss dt larger than 3.0 * dt_mean, drop.";
                        return false;
                    }
                }
            }
            if (std::abs(cur_dt_) > 1.0) {
                // 剔除延迟超过1s的数据
                AWARN << "time gap between gnss and imu too large: dt = " << cur_dt_;
                return false;
            }

            // GNSS时间补偿，通过DR的信息将系统状态拉到GNSS时间，进行融合
            auto delta_motion = kinematic_compensation.delta(cur_gnss_data.measurement_timestamp, cur_imu_data.measurement_timestamp, true);

            MSF::GnssDataPtr gnss_ptr = std::make_shared<MSF::GnssData>(cur_gnss_data);

            // 判断时间补偿状态，如果没有错误码，认为补偿ok，如果有错误码，不进行补偿，报错误码
            if (delta_motion.first == 0) {

                // 获得补偿量，并进行GNSS更新
                auto delta_ptr = std::make_shared<MSF::KinematicData>(delta_motion.second);

                // 此处对GNSS的航向角进行一个补偿
                if (pre_gnss_data.measurement_timestamp != 0 &&                                               //
                    std::abs(pre_gnss_data.measurement_timestamp - cur_gnss_data.measurement_timestamp) < 1.0 //
                ) {
                    auto delta_motion_gnss = kinematic_compensation.delta(pre_gnss_data.measurement_timestamp, cur_gnss_data.measurement_timestamp, false);
                    if (delta_motion_gnss.first == 0) {
                        if (std::abs(delta_motion_gnss.second.measurement_timestamp) < 1.0) {
                            // 理论上由轨迹得到的航向角会有转向不足的问题
                            gnss_ptr->hdg = gnss_ptr->hdg - 0.5 * INS::quaternion2euler(delta_motion_gnss.second.att).z();
                        } else {
                            AWARN << "delta between pre gnss and cur gnss is too large: " << std::abs(delta_motion_gnss.second.measurement_timestamp) << " seconds";
                        }
                    } else {
                        AINFO << "gnss kinematic compensation (pre gnss and cur gnss) error code: " << delta_motion_gnss.first;
                    }
                }
                auto dr_gpst_ = kinematic_compensation.pin(gnss_ptr->measurement_timestamp);
                fusion_ptr_->ProcessGnssData(gnss_ptr, msf_state_ptr, delta_ptr, std::make_shared<MSF::KinematicData>(dr_gpst_.second));

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
                {
                    auto delta_eulr_ = INS::quaternion2euler(delta_motion.second.att);
                    auto kcp_str =
                        fmt::format("{:>14.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>6.3f},{:>7.5f},{:>6.3f}\n", //
                                    gnss_ptr->measurement_timestamp,                                                                // 1
                                    delta_motion.second.measurement_timestamp,                                                      // 2
                                    delta_motion.second.vel.x(),                                                                    // 3
                                    delta_motion.second.vel.y(),                                                                    // 4
                                    delta_motion.second.vel.z(),                                                                    // 5
                                    delta_motion.second.pos.x(),                                                                    // 6
                                    delta_motion.second.pos.y(),                                                                    // 7
                                    delta_motion.second.pos.z(),                                                                    // 8
                                    delta_motion.second.ego_longitude_vel,                                                          // 9
                                    delta_eulr_.z() * 180.0 / M_PI                                                                  // 10
                        );
                    debug::debug_sgt.kcp_state.line(kcp_str);
                }
#endif

            } else {
                AINFO << "Gnss kinematic compensation error code: " << delta_motion.first;
                return false;
            }
        }
        return true;
    };

    try_fuse_veh = [this]() -> bool {
        // 底盘信息需要在内部DR递推的时候使用，这里不能清零。
        // cur_veh_data = {};

        // 设计上，底盘轮速频率与IMU一致
        // 但实际中，底盘频率可能略高或者略低于IMU
        // 这里的处理逻辑是：在底盘消息队列里面首先选取0.02秒之内的消息、其次选取新于IMU的底盘消息、都不满足则选取最后一帧底盘消息
        // 额外的限制条件是底盘消息与IMU之间的时间差小于 IMU_VEHICLE_TIME_GAP_BOUND_

        static constexpr double IMU_VEHICLE_TIME_GAP_BOUND_ = 0.08;

        while (vehicle_queue.try_dequeue(cur_veh_data)) {

            auto dt_ = cur_veh_data.measurement_timestamp - cur_imu_data.measurement_timestamp;

            auto dt_imu_veh = std::abs(dt_);

            // 取0.02s内的轮速，或者轮速比IMU更新，或者取队列最后一个（满足时间差在 IMU_VEHICLE_TIME_GAP_BOUND_ 以内）
            if (dt_imu_veh < 0.02 ||
                (dt_ > 0.0 && dt_imu_veh < IMU_VEHICLE_TIME_GAP_BOUND_) ||
                (vehicle_queue.size_approx() == 0 && dt_imu_veh < IMU_VEHICLE_TIME_GAP_BOUND_)) {

                MSF::VehicleDataPtr veh_ptr = std::make_shared<MSF::VehicleData>(cur_veh_data);

                MSF::KinematicDataPtr delta_motion_ptr = nullptr;

                // 不使用低通，相位延迟无法准确估计
                if (false) {
                    // 使用低通滤波后的轮速
                    veh_ptr->speed_rl              = veh_ptr->speed_rl_lp;
                    veh_ptr->speed_rr              = veh_ptr->speed_rr_lp;
                    veh_ptr->measurement_timestamp = veh_ptr->measurement_timestamp + veh_ptr->lpf_delay;
                }

                double dt_ = std::abs(veh_ptr->measurement_timestamp - cur_imu_data.measurement_timestamp);

                if (dt_ > 0.005) {
                    auto delta_motion = kinematic_compensation.delta(veh_ptr->measurement_timestamp, cur_imu_data.measurement_timestamp, true);
                    if (delta_motion.first == 0) {
                        delta_motion_ptr = std::make_shared<MSF::KinematicData>(delta_motion.second);
                    } else {
                        AINFO << "kinematic compensation error code: " << delta_motion.first;
                        return false;
                    }
                }
                // AINFO << "fuse vehicle";
                fusion_ptr_->ProcessVehicleData(veh_ptr, msf_state_ptr, delta_motion_ptr);
                vehinfo_sgt.update_state({msf_state_ptr->vehicle_bias.x(), msf_state_ptr->vehicle_bias.y(), msf_state_ptr->vehicle_bias.z()});
                break;
            } else if (vehicle_queue.size_approx() == 0) {
                // 最后一个仍不满足的话，报warn
                AWARN_EVERY(100) << "vehicle not sync, latest dt: " << std::setprecision(6) << dt_imu_veh;
            }
        }
        return true;
    };

    try_fuse_vis = [this]() -> bool {
        // 设计上来讲，VF消息频率远低于IMU，预期延迟100ms以上
        // 使用最新的VF消息进行量测更新
        while (vision_fusion_queue.size_approx() > 1) {
            vision_fusion_queue.try_dequeue(cur_vis_data);
        }
        if (vision_fusion_queue.try_dequeue(cur_vis_data)) {

            if (cur_vis_data.pos_offset.norm() < 1e-10) {
                // 如果视觉匹配给的很小的话，认为是错误的，过滤掉。
                return false;
            }

            double dt_imu_vis = std::abs(cur_vis_data.measurement_timestamp - cur_imu_data.measurement_timestamp);
            if (dt_imu_vis > 0.5) {
                AWARN << "delta between cur imu and cur vision fusion is too large: " << dt_imu_vis << " seconds";
            }

            // vision fusion时间补偿，通过DR的信息将系统状态拉到vf时间，进行融合
            auto delta_motion = kinematic_compensation.delta(cur_vis_data.measurement_timestamp, cur_imu_data.measurement_timestamp, true);

            MSF::VisionFusionDataPtr vis_ptr = std::make_shared<MSF::VisionFusionData>(cur_vis_data);

            // 判断时间补偿状态，如果没有错误码，认为补偿ok，如果有错误码，不进行补偿，报错误码
            if (delta_motion.first == 0) {

                // 获得补偿量，并进行GNSS更新
                auto delta_ptr = std::make_shared<MSF::KinematicData>(delta_motion.second);

                fusion_ptr_->ProcessVisionData(vis_ptr, msf_state_ptr, delta_ptr);

            } else {
                AINFO << "Vision kinematic compensation error code: " << delta_motion.first;
                return false;
            }
        }
        return true;
    };

    try_fuse_map = [this]() -> bool {
        // sd map match消息20Hz，频率低于IMU消息
        // 使用最新的map消息进行量测更新
        while (map_pos_queue.size_approx() > 1) {
            map_pos_queue.try_dequeue(cur_map_data);
        }
        if (map_pos_queue.try_dequeue(cur_map_data)) {
            double dt_imu_map = std::fabs(cur_map_data.measurement_timestamp - cur_imu_data.measurement_timestamp);
            if (dt_imu_map > 1.0) {
                AWARN << "delta between sdmap and imu too large: " << dt_imu_map;
            }

            // map fusion时间补偿，通过DR的信息将系统状态拉到map时间，进行融合
            auto delta_motion = kinematic_compensation.delta(cur_map_data.measurement_timestamp, cur_imu_data.measurement_timestamp, true);

            MSF::MapPosDataPtr map_ptr = std::make_shared<MSF::MapPosData>(cur_map_data);

            // 判断时间补偿状态，如果没有错误码，认为补偿ok，如果有错误码，不进行补偿，报错误码
            if (delta_motion.first == 0) {
                auto delta_ptr = std::make_shared<MSF::KinematicData>(delta_motion.second);
                fusion_ptr_->ProcessMapData(map_ptr, msf_state_ptr, delta_ptr);
            } else {
                AINFO << "map kinematic compensation error code: " << delta_motion.first;
                return false;
            }
        }
        return true;
    };

    try_fuse_db = [this]() -> bool {
        // DriveBoundary消息，消费db队列，调用ProcessDbData
        MSF::DbData db_data;
        while (db_queue.size_approx() > 1) {
            db_queue.try_dequeue(db_data);
        }
        if (db_queue.try_dequeue(db_data)) {
            double dt_imu_db = std::fabs(db_data.projection.measurement_timestamp - cur_imu_data.measurement_timestamp);
            if (dt_imu_db > 2.0) {
                AWARN << "delta between DriveBoundary and imu too large: " << dt_imu_db;
            }
            MSF::DbDataPtr db_data_ptr = std::make_shared<MSF::DbData>(db_data);
            fusion_ptr_->ProcessDbData(db_data_ptr, msf_state_ptr);
        }
        return true;
    };
}

/*
  消息注入，采用并发队列的数据结构
  消息来了之后放入enqueue，堆积过多则丢弃dequeue
  每个消息注入的时候设置生产者token，以保证顺序一致
*/

int TCMSF_IMPL::insert_msg(std::shared_ptr<Imu> imu_msg_) {
    MSF::ImuData imu_data;
    if (imu_msg_->has_header() &&
        imu_msg_->header().has_measurement_timestamp() &&
        imu_msg_->header().has_publish_timestamp() &&
        imu_msg_->header().has_sequence_num() &&
        imu_msg_->has_accel() &&
        imu_msg_->accel().has_x() &&
        imu_msg_->accel().has_y() &&
        imu_msg_->accel().has_z() &&
        imu_msg_->has_gyro() &&
        imu_msg_->gyro().has_x() &&
        imu_msg_->gyro().has_y() &&
        imu_msg_->gyro().has_z() &&
        imu_msg_->has_imu_status()) {
        imu_data.measurement_timestamp = imu_msg_->header().measurement_timestamp();
        imu_data.publish_timestamp     = imu_msg_->header().publish_timestamp();
        imu_data.sequence_num          = imu_msg_->header().sequence_num();
        imu_data.acc << imu_msg_->accel().x(), imu_msg_->accel().y(), imu_msg_->accel().z();
        imu_data.acc = imu_data.acc * (msf_state_ptr->gravity.norm());
        imu_data.gyro << imu_msg_->gyro().x(), imu_msg_->gyro().y(), imu_msg_->gyro().z();
        imu_data.gyro = imu_data.gyro / 180.0 * M_PI;

        if (imu_msg_->imu_status() != 1) {
            AERROR << "imu msg is not valid, drop!";
            return -1;
        }

        if (imu_data.isNaN()) {
            AERROR << "imu nan detected, drop msg";
            return -1;
        }

    } else {
        AWARN << "Imu msg's info is not sufficient, missing necessary key";
        return -1;
    }

#if (defined __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES) && (defined __TCMSF_ENABLE_TROUBLE_MAKER_)
    AINFO_EVERY(3000) << "modify imu data";
    sensor_modify_ptr_->sensor_modify(imu_data);
#endif

    // auto &motion_status_ref = MSF::MotionStatus::GetInstance();
    // motion_status_ref.PushImuData(imu_data);
    // motion_status_ref.UpdateStatus();
    // // motion_status_ref.PrintStatus();

    moodycamel::ProducerToken imu_ptok(imu_queue);

    // 此处压到队列里面的IMU数据，为瞬态量测值并采用标准的单位。
    // 角速度 -> rad/s
    // 加速度 -> m/s^2
    imu_queue.enqueue(imu_ptok, std::move(imu_data));

    while (imu_queue.size_approx() > IMU_MAX_QUEUE_SIZE) {
        // 抛弃未处理的数据，保证队列大小小于 IMU_MAX_QUEUE_SIZE
        MSF::ImuData imudata_tmp;
        AWARN << "drop imu queue's item";
        imu_queue.try_dequeue(imudata_tmp);
    }
    return 0;
}

int TCMSF_IMPL::insert_msg(std::shared_ptr<Gps> gnss_msg_) {
    MSF::GnssData gnss_data;
    if (gnss_msg_->has_header() &&
        gnss_msg_->header().has_measurement_timestamp() &&
        gnss_msg_->header().has_publish_timestamp() &&
        gnss_msg_->header().has_sequence_num() &&
        gnss_msg_->has_position() &&
        gnss_msg_->position().has_lon() &&
        gnss_msg_->position().has_lat() &&
        gnss_msg_->position().has_height() &&
        gnss_msg_->has_position_std() &&
        gnss_msg_->position_std().has_lon() &&
        gnss_msg_->position_std().has_lat() &&
        gnss_msg_->position_std().has_height() &&
        gnss_msg_->has_heading() &&
        gnss_msg_->has_heading_std() &&
        gnss_msg_->has_linear_velocity() &&
        gnss_msg_->linear_velocity().has_x() &&
        gnss_msg_->linear_velocity().has_y() &&
        gnss_msg_->linear_velocity().has_z() &&
        gnss_msg_->has_linear_velocity_std() &&
        gnss_msg_->linear_velocity_std().has_x() &&
        gnss_msg_->linear_velocity_std().has_y() &&
        gnss_msg_->linear_velocity_std().has_z() &&
        gnss_msg_->has_position_status() &&
        gnss_msg_->has_num_sats()) {
        gnss_data.measurement_timestamp = gnss_msg_->header().measurement_timestamp();
        gnss_data.publish_timestamp     = gnss_msg_->header().publish_timestamp();
        gnss_data.sequence_num          = gnss_msg_->header().sequence_num();
        auto &pos                       = gnss_msg_->position();
        auto &pos_std                   = gnss_msg_->position_std();
        auto &vel                       = gnss_msg_->linear_velocity();
        auto &vel_std                   = gnss_msg_->linear_velocity_std();
        gnss_data.lla << pos.lat() / 180.0 * M_PI, pos.lon() / 180.0 * M_PI, pos.height();
        gnss_data.lla_cov << pos_std.lat(), pos_std.lon(), pos_std.height();
        // gnss_data.lla_cov << pos_std.lat(), pos_std.lon(), 10.0;
        gnss_data.vel << vel.x(), vel.y(), vel.z();
        gnss_data.vel_cov << vel_std.x(), vel_std.y(), vel_std.z();
        if (false) {
            // 旧版本，PBOX内的GNSS航向定义为，指南为零，南偏西为正，0~360
            gnss_data.hdg = gnss_msg_->heading() * M_PI / 180.0 - M_PI;
        } else {
            // N2A 域控航向的定义是，指北为零，北偏东为正，0~360
            gnss_data.hdg = gnss_msg_->heading() * M_PI / 180.0;
            if (gnss_data.hdg > M_PI) {
                gnss_data.hdg = gnss_data.hdg - 2 * M_PI;
            }
        }
        gnss_data.hdg_cov  = gnss_msg_->heading_std();
        gnss_data.num_sats = gnss_msg_->num_sats();

        // AINFO << std::fixed << std::setprecision(4) << "gnss " << gnss_data.measurement_timestamp << " " << gnss_msg_->sec_in_gps_week();

        /* 卫星状态定义如下
         *  enum GNSS_STATUS {
         *      INVALID = 0
         *      NONE    = 1,
         *      SINGLE  = 2,
         *      DGPS    = 3,
         *      PPP     = 4,
         *      FLOAT   = 5,
         *      FIX     = 6
         *  };
         */
        gnss_data.status     = (uint64_t)gnss_msg_->position_status();
        gnss_data.raw_status = (uint64_t)gnss_msg_->position_status();
        if (gnss_data.status > 6 || gnss_data.status < 0) {
            gnss_data.status = 0;
        }

        if (gnss_msg_->has_rtk_age()) {
            gnss_data.rtk_age = gnss_msg_->rtk_age();
        }

        if (gnss_data.isNaN()) {
            AERROR << "gnss nan detected, drop msg";
            return -1;
        }
        uint64_t status_valid_set = 1;
        // if (parameters_sgt.get_gnss_fusion_mode() != config::Parameters::GnssFusionMode::GNSS_LOOSE_COUPLE) status_valid_set = 6;
        if (gnss_data.status == 0)
            last_status_valid = 0;
        else if (gnss_data.status >= status_valid_set)
            last_status_valid = 1;
        else if (last_status_valid == 0 || last_status_valid == 3)
            last_status_valid = 3;
        else
            last_status_valid = 2;
        MSF::ProcessControl &process_control_sgt = MSF::ProcessControl::getInstance();
        //只有在有效且pps整数秒时刻的修正才能保证topic中的域控时间是经过了补偿的
        if ((last_status_valid == 1 || last_status_valid == 2) && static_cast<int>(gnss_msg_->sec_in_gps_week() * 1000) % 100 == 0) {
            process_control_sgt.last_valid_dt = gnss_data.measurement_timestamp - gnss_msg_->sec_in_gps_week();
        }

    } else {
        AWARN << "Gnss msg's info is not sufficient, missing necessary key";
        return -1;
    }
    if (gnss_data.lla.x() == 0.0 || gnss_data.lla.y() == 0.0) {
        // AWARN << "Gnss msg's info is not correct";
        return -2;
    }

    moodycamel::ProducerToken gnss_ptok(gnss_queue);
    // 压入GNSS数据
    // 经纬度采用弧度制
    // 航向角采用弧度制
    gnss_queue.enqueue(gnss_ptok, gnss_data);
    while (gnss_queue.size_approx() > GNSS_MAX_QUEUE_SIZE) {
        MSF::GnssData gnssdata_tmp;
        AINFO << "drop gnss queue's item";
        gnss_queue.try_dequeue(gnssdata_tmp);
    }
    return 0;
}

int TCMSF_IMPL::insert_msg(std::shared_ptr<VehInfo> veh_msg_) {
    MSF::VehicleData veh_data;
    if (veh_msg_->has_header() &&
        veh_msg_->header().has_measurement_timestamp() &&
        veh_msg_->header().has_publish_timestamp() &&
        veh_msg_->header().has_sequence_num() &&
        veh_msg_->has_ego_motion_status() &&
        veh_msg_->ego_motion_status().has_da_in_rlwhlspd_sg() &&
        veh_msg_->ego_motion_status().has_da_in_rlwhlspd_v_u8() &&
        veh_msg_->ego_motion_status().has_da_in_rrwhlspd_sg() &&
        veh_msg_->ego_motion_status().has_da_in_rrwhlspd_v_u8() &&
        veh_msg_->ego_motion_status().has_da_in_rlwhldrvdir_u8() &&
        veh_msg_->ego_motion_status().has_da_in_rrwhldrvdir_u8() &&
        veh_msg_->ego_motion_status().has_da_in_rlwhldrvdir_v_u8() &&
        veh_msg_->ego_motion_status().has_da_in_rrwhldrvdir_v_u8() &&
        veh_msg_->ego_motion_status().has_da_in_yawrate_sg() &&
        veh_msg_->ego_motion_status().has_da_in_latacc_sg() &&
        veh_msg_->ego_motion_status().has_da_in_latacc_v_u8() &&
        veh_msg_->ego_motion_status().has_da_in_lgtacc_sg() &&
        veh_msg_->ego_motion_status().has_da_in_lgtacc_v_u8() &&
        veh_msg_->ego_motion_status().has_da_in_flwhlplscnt_u8() &&
        veh_msg_->ego_motion_status().has_da_in_frwhlplscnt_u8() &&
        veh_msg_->ego_motion_status().has_da_in_rlwhlplscnt_u8() &&
        veh_msg_->ego_motion_status().has_da_in_rrwhlplscnt_u8() &&
        veh_msg_->has_propulsion_system_status() &&
        veh_msg_->propulsion_system_status().has_da_in_trmgearlvl_u8() &&
        veh_msg_->has_steering_system_status() &&
        veh_msg_->steering_system_status().has_da_in_strgwhlang_sg() &&
        veh_msg_->steering_system_status().has_da_in_strgwhlanggrd_sg()) {

        if (veh_msg_->has_brake_system_status() &&
            veh_msg_->brake_system_status().has_da_in_vdcactive_u8() &&
            veh_msg_->brake_system_status().has_da_in_tcsactive_u8() &&
            veh_msg_->brake_system_status().has_da_in_espactive_u8()) {
            // 根据VDC、TCS、ESP三个系统的状态判断底盘轮速是否可信
            bool is_vdc_active = veh_msg_->brake_system_status().da_in_vdcactive_u8() == 0x1;
            bool is_tcs_active = veh_msg_->brake_system_status().da_in_tcsactive_u8() == 0x1;
            bool is_esp_active = veh_msg_->brake_system_status().da_in_espactive_u8() == 0x1;
            if (is_vdc_active ||
                is_tcs_active ||
                is_esp_active) {
                veh_data.tire_slip = true;
                AWARN_EVERY(100) << "VDC or TCS or ESP is active, wheel speed may not valid";
            } else {
                veh_data.tire_slip = false;
            }
        }

        // 两后轮速度是否有效，如果无效则无法使用底盘消息
        if (veh_msg_->ego_motion_status().da_in_rlwhlspd_v_u8() &&
            veh_msg_->ego_motion_status().da_in_rrwhlspd_v_u8()) {
            veh_data.measurement_timestamp = veh_msg_->header().measurement_timestamp();
            veh_data.publish_timestamp     = veh_msg_->header().publish_timestamp();
            veh_data.sequence_num          = veh_msg_->header().sequence_num();

            // 后轮轮速在驱动层可能加了符号，也可能没加
            // 此处取轮速绝对值和轮速方向，合并为带符号的轮速
            auto dir_rl   = veh_msg_->ego_motion_status().da_in_rlwhldrvdir_u8();
            auto dir_rr   = veh_msg_->ego_motion_status().da_in_rrwhldrvdir_u8();
            auto dir_rl_v = veh_msg_->ego_motion_status().da_in_rlwhldrvdir_v_u8();
            auto dir_rr_v = veh_msg_->ego_motion_status().da_in_rrwhldrvdir_v_u8();

            auto trmgear = veh_msg_->propulsion_system_status().da_in_trmgearlvl_u8();

            auto vel_rl = std::abs(veh_msg_->ego_motion_status().da_in_rlwhlspd_sg());
            auto vel_rr = std::abs(veh_msg_->ego_motion_status().da_in_rrwhlspd_sg());

            if (dir_rl_v == 0 && dir_rr_v == 0) {
                if ((vel_rl > 1e-10 || vel_rr > 1e-10) && (dir_rl == 3 || dir_rr == 3)) {
                    // 这里考虑轮速方向的延迟
                    // 如果出现轮速有值，但是方向给静止的话，就使用挡位来判断轮速的方向
                    veh_data.speed_rl = (trmgear == 4) ? -vel_rl : vel_rl;
                    veh_data.speed_rr = (trmgear == 4) ? -vel_rr : vel_rr;
                } else {
                    // 左右后轮行驶方向状态均有效
                    veh_data.speed_rl = (dir_rl == 2) ? -vel_rl : vel_rl;
                    veh_data.speed_rr = (dir_rr == 2) ? -vel_rr : vel_rr;
                }
            } else if (dir_rl_v == 0 || dir_rr_v == 0) {
                // 左右后轮行驶方向状态一个无效，一个有效
                if (dir_rl_v == 0) {
                    veh_data.speed_rl = (dir_rl == 2) ? -vel_rl : vel_rl;
                    veh_data.speed_rr = (dir_rl == 2) ? -vel_rr : vel_rr;
                }
                if (dir_rr_v == 0) {
                    veh_data.speed_rr = (dir_rr == 2) ? -vel_rr : vel_rr;
                    veh_data.speed_rl = (dir_rr == 2) ? -vel_rl : vel_rl;
                }
            } else {
                // 左右后轮行驶方向标志都无效
                if (trmgear == 4) {
                    veh_data.speed_rl = -vel_rl;
                    veh_data.speed_rr = -vel_rr;
                } else {
                    veh_data.speed_rl = vel_rl;
                    veh_data.speed_rr = vel_rr;
                }
            }

            veh_data.yaw_rate = veh_msg_->ego_motion_status().da_in_yawrate_sg();

            auto spd_lp_         = veh_lpf({veh_data.speed_rl, veh_data.speed_rr});
            veh_data.speed_rl_lp = spd_lp_[0];
            veh_data.speed_rr_lp = spd_lp_[1];
            veh_data.lpf_delay   = veh_lpf_delay_;

            veh_data.steer      = veh_msg_->steering_system_status().da_in_strgwhlang_sg();
            veh_data.steer_rate = veh_msg_->steering_system_status().da_in_strgwhlanggrd_sg();
            veh_data.fl_pls_cnt = veh_msg_->ego_motion_status().da_in_flwhlplscnt_u8();
            veh_data.fr_pls_cnt = veh_msg_->ego_motion_status().da_in_frwhlplscnt_u8();
            veh_data.rl_pls_cnt = veh_msg_->ego_motion_status().da_in_rlwhlplscnt_u8();
            veh_data.rr_pls_cnt = veh_msg_->ego_motion_status().da_in_rrwhlplscnt_u8();
            if (veh_msg_->ego_motion_status().da_in_latacc_v_u8() == 1 && veh_msg_->ego_motion_status().da_in_lgtacc_v_u8() == 1) {
                veh_data.acc_lat = veh_msg_->ego_motion_status().da_in_latacc_sg();
                veh_data.acc_lgt = veh_msg_->ego_motion_status().da_in_lgtacc_sg();
            }

            if (veh_data.isNaN()) {
                AERROR << "vehicle info nan detected, drop msg";
                return -1;
            }

        } else {
            AWARN << "Vehicle msg's speed info is not valid";
            return -2;
        }
    } else {
        AWARN << "Vehicle msg's info is not sufficient, missing necessary key";
        return -1;
    }

    // auto &motion_status_ref = MSF::MotionStatus::GetInstance();
    // motion_status_ref.PushVehicleData(veh_data);

    moodycamel::ProducerToken veh_ptok(vehicle_queue);

    vehicle_queue.enqueue(veh_ptok, veh_data);
    while (vehicle_queue.size_approx() > VEHICLE_MAX_QUEUE_SIZE) {
        MSF::VehicleData vehdata_tmp;
        AINFO << "drop vehicle queue's item";
        vehicle_queue.try_dequeue(vehdata_tmp);
    }
    return 0;
}

int TCMSF_IMPL::insert_msg(std::shared_ptr<LocalizationEstimate> dr_msg_) {
    MSF::KinematicData kin_data;
    // 输入为DR坐标系，即FLU
    if (dr_msg_->has_header() &&
        dr_msg_->header().has_measurement_timestamp() &&
        dr_msg_->header().has_publish_timestamp() &&
        dr_msg_->header().has_sequence_num() &&
        dr_msg_->has_pose() &&
        // dr_msg_->pose().has_linear_velocity() &&
        // dr_msg_->pose().linear_velocity().has_x() &&
        // dr_msg_->pose().linear_velocity().has_y() &&
        // dr_msg_->pose().linear_velocity().has_z() &&
        dr_msg_->pose().has_orientation() &&
        dr_msg_->pose().orientation().has_qw() &&
        dr_msg_->pose().orientation().has_qx() &&
        dr_msg_->pose().orientation().has_qy() &&
        dr_msg_->pose().orientation().has_qz() &&
        dr_msg_->pose().has_position() &&
        dr_msg_->pose().position().has_x() &&
        dr_msg_->pose().position().has_y() &&
        dr_msg_->pose().position().has_z()) {
        kin_data.measurement_timestamp = dr_msg_->header().measurement_timestamp();
        kin_data.att.w()               = dr_msg_->pose().orientation().qw();
        kin_data.att.x()               = dr_msg_->pose().orientation().qx();
        kin_data.att.y()               = dr_msg_->pose().orientation().qy();
        kin_data.att.z()               = dr_msg_->pose().orientation().qz();
        kin_data.pos.x()               = dr_msg_->pose().position().x();
        kin_data.pos.y()               = dr_msg_->pose().position().y();
        kin_data.pos.z()               = dr_msg_->pose().position().z();
        // kin_data.vel.x()               = dr_msg_->pose().linear_velocity().x();
        // kin_data.vel.y()               = dr_msg_->pose().linear_velocity().y();
        // kin_data.vel.z()               = dr_msg_->pose().linear_velocity().z();

        if (kin_data.isNaN()) {
            AERROR << "DR nan detected, drop msg";
            return -1;
        }

    } else {
        AWARN << "dr msg's info is not sufficient, missing necessary key";
        return -1;
    }

    if (!parameters_sgt.get_use_internal_dr_info()) {
        // 采用外部DR状态递推序列
        kinematic_compensation.insert(std::move(kin_data));
    }
    return 0;
}

int TCMSF_IMPL::insert_msg(std::shared_ptr<VFResult> vis_msg_) {
    MSF::VisionFusionData vis_data;
    if (vis_msg_->has_header() &&
        vis_msg_->header().has_measurement_timestamp() &&
        vis_msg_->header().has_publish_timestamp() &&
        vis_msg_->header().has_sequence_num() &&
        vis_msg_->has_offset() &&
        vis_msg_->offset().has_offset() &&
        vis_msg_->offset().offset().has_x() &&
        vis_msg_->offset().offset().has_y() &&
        vis_msg_->offset().has_heading()) {
        vis_data.measurement_timestamp = vis_msg_->header().measurement_timestamp();
        vis_data.publish_timestamp     = vis_msg_->header().publish_timestamp();
        vis_data.sequence_num          = vis_msg_->header().sequence_num();

        double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
        x_   = vis_msg_->offset().offset().x();
        y_   = vis_msg_->offset().offset().y();
        yaw_ = vis_msg_->offset().heading() / 180.0 * M_PI;

        if (vis_msg_->has_offset_std() &&
            vis_msg_->offset_std().has_offset() &&
            vis_msg_->offset_std().has_heading() &&
            vis_msg_->offset_std().offset().has_x() &&
            vis_msg_->offset_std().offset().has_y()) {
            double x_std_   = std::abs(vis_msg_->offset_std().offset().x());
            double y_std_   = std::abs(vis_msg_->offset_std().offset().y());
            double hdg_std_ = std::abs(vis_msg_->offset_std().heading()) / 180.0 * M_PI;

            vis_data.pos_offset_std.x() = y_std_;
            vis_data.pos_offset_std.y() = x_std_;
            vis_data.pos_offset_std.z() = 1e10;
            vis_data.hdg_offset_std     = hdg_std_;
        }

        // 输入的为FLU，此处转换成RFU
        vis_data.pos_offset.x() = -y_;
        vis_data.pos_offset.y() = x_;
        vis_data.pos_offset.z() = 0;    // 认为视觉融合在平面上进行，z轴设为0
        vis_data.hdg_offset     = yaw_; // 转换成弧度

        if (vis_data.isNaN()) {
            AERROR << "VF nan detected, drop msg";
            return -1;
        }

    } else {
        AWARN << "vision fusion msg's info is not sufficient, missing necessary key";
        return -1;
    }

    moodycamel::ProducerToken vision_ptok(vision_fusion_queue);

    vision_fusion_queue.enqueue(vision_ptok, std::move(vis_data));

    while (vision_fusion_queue.size_approx() > VISION_MAX_QUEUE_SIZE) {
        // 抛弃未处理的数据，保证队列大小小于 VISION_MAX_QUEUE_SIZE
        MSF::VisionFusionData visdata_tmp;
        AWARN << "drop vision fusion queue's item";
        vision_fusion_queue.try_dequeue(visdata_tmp);
    }
    return 0;
}

int TCMSF_IMPL::insert_msg(std::shared_ptr<SDMapMatchResult> sd_msg_) {
    MSF::MapPosData map_pos_data;

    if (sd_msg_->has_header() &&
        sd_msg_->header().has_measurement_timestamp() &&
        sd_msg_->header().has_publish_timestamp() &&
        sd_msg_->header().has_sequence_num() &&
        sd_msg_->has_sd_err_reason() &&
        sd_msg_->has_projection_position() &&
        sd_msg_->has_is_off_route()) {
        map_pos_data.measurement_timestamp = sd_msg_->header().measurement_timestamp();
        map_pos_data.publish_timestamp     = sd_msg_->header().publish_timestamp();
        map_pos_data.sequence_num          = sd_msg_->header().sequence_num();
        map_pos_data.lla.x()               = sd_msg_->projection_position().lat();
        map_pos_data.lla.y()               = sd_msg_->projection_position().lon();

        bool sdmm_no_error_ = sd_msg_->sd_err_reason() == byd::modules::localization::SDErrReason::NO_ERROR;
        bool sdmm_no_mpp_   = sd_msg_->sd_err_reason() == byd::modules::localization::SDErrReason::NO_MPP;
        // 地图匹配NO_ERROR或者NO_MPP认为是ok的
        // 如果不是这两个状态则认为是错误的
        bool sdmm_not_ok_ = !(sdmm_no_error_ || sdmm_no_mpp_);

        if (sd_msg_->is_off_route() || // 偏航的
            sdmm_not_ok_               // 有错误的
        ) {
            // 过滤掉偏航的、有错误的消息
            AINFO_EVERY(100) << "SDMM off route or not good, drop msg";
            return -1;
        }
        if (map_pos_data.isNaN()) {
            AERROR << "SDMM nan detected, drop msg";
            return -1;
        }
    } else {
        AWARN << "map msg's info is not sufficient, missing necessary key";
        return -1;
    }

    moodycamel::ProducerToken map_ptok(map_pos_queue);
    map_pos_queue.enqueue(map_ptok, std::move(map_pos_data));
    while (map_pos_queue.size_approx() > MAP_MAX_QUEUE_SIZE) {
        // 抛弃未处理的数据，保证队列大小小于 MAP_MAX_QUEUE_SIZE
        MSF::MapPosData mapdata_tmp;
        AWARN << "drop map fusion queue's item";
        map_pos_queue.try_dequeue(mapdata_tmp);
    }
    return 0;
}

int TCMSF_IMPL::insert_msg(std::shared_ptr<DriveBoundary> db_msg_) {
    // 数据有效性检查（必须在两条边界）
    if (!db_msg_->has_header() || !db_msg_->header().has_measurement_timestamp()) {
        AWARN << "DriveBoundary msg missing header or measurement_timestamp";
        return -1;
    }
    if (db_msg_->boundary_raw_size() != 2) {
        AWARN << "DriveBoundary boundary count is not 2: " << db_msg_->boundary_raw_size();
        return -1;
    }
    if (db_msg_->boundary_raw(0).polyline_size() < MSF::DbData::BOUNDARY_POINT_NUM) {
        AWARN << "DriveBoundary boundary_0 polyline size less than 5: " << db_msg_->boundary_raw(0).polyline_size();
        return -1;
    }
    if (db_msg_->boundary_raw(1).polyline_size() < MSF::DbData::BOUNDARY_POINT_NUM) {
        AWARN << "DriveBoundary boundary_1 polyline size less than 5: " << db_msg_->boundary_raw(1).polyline_size();
        return -1;
    }
    if (db_msg_->dr2veh_matrix_size() != 16) {
        AWARN << "DriveBoundary dr2veh_matrix size is not 16: " << db_msg_->dr2veh_matrix_size();
        return -1;
    }

    MSF::DbData db_data;
    db_data.measurement_timestamp = db_msg_->header().measurement_timestamp();

    // 提取边界0形点（前5个）
    const auto &boundary_0_proto = db_msg_->boundary_raw(0);
    for (int i = 0; i < MSF::DbData::BOUNDARY_POINT_NUM; ++i) {
        const auto &pt            = boundary_0_proto.polyline(i);
        db_data.boundary_0[i] = Eigen::Vector2d(pt.x(), pt.y());
    }

    // 提取边界1形点（前5个）
    const auto &boundary_1_proto = db_msg_->boundary_raw(1);
    for (int i = 0; i < MSF::DbData::BOUNDARY_POINT_NUM; ++i) {
        const auto &pt            = boundary_1_proto.polyline(i);
        db_data.boundary_1[i] = Eigen::Vector2d(pt.x(), pt.y());
    }

    // 提取 dr2veh_matrix (4x4 矩阵，16个值按列存储)
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            db_data.dr2veh_matrix(i, j) = db_msg_->dr2veh_matrix(j * 4 + i);
        }
    }
    // 计算逆矩阵的位移部分
    db_data.veh2dr_translation = db_data.dr2veh_matrix.inverse().block<3, 1>(0, 3);

    // 计算投影点
    db_data.compute_projection();

    if (db_data.isNaN()) {
        AERROR << "DriveBoundary nan detected, drop msg";
        return -1;
    }

    // Debug: 打印转换后的形点坐标和投影结果
    // db_data.debug_print_boundary_in_veh();

    moodycamel::ProducerToken db_ptok(db_queue);
    db_queue.enqueue(db_ptok, std::move(db_data));
    while (db_queue.size_approx() > DB_MAX_QUEUE_SIZE) {
        MSF::DbData dbdata_tmp;
        AWARN << "drop drive boundary queue's item";
        db_queue.try_dequeue(dbdata_tmp);
    }
    return 0;
}

/*
  融合的结果输出
*/

int TCMSF_IMPL::output_msg(std::shared_ptr<Pose> tcmsf_result_msg_) {
    auto vehicle_state_ptr = std::make_shared<MSF::State>(msf_state_ptr->ToVehicle().WithSdmapBias());
    tcmsf_result_msg_->Clear();
    auto header_ = tcmsf_result_msg_->mutable_header();
    header_->set_sequence_num(sequence_num++);
    header_->set_measurement_timestamp(vehicle_state_ptr->measurement_timestamp);
    // 输出按照习惯，经纬度以度为单位
    tcmsf_result_msg_->mutable_attitude()->set_qw(vehicle_state_ptr->att.w());
    tcmsf_result_msg_->mutable_attitude()->set_qx(vehicle_state_ptr->att.x());
    tcmsf_result_msg_->mutable_attitude()->set_qy(vehicle_state_ptr->att.y());
    tcmsf_result_msg_->mutable_attitude()->set_qz(vehicle_state_ptr->att.z());
    tcmsf_result_msg_->mutable_velocity()->set_x(vehicle_state_ptr->vel.x());
    tcmsf_result_msg_->mutable_velocity()->set_y(vehicle_state_ptr->vel.y());
    tcmsf_result_msg_->mutable_velocity()->set_z(vehicle_state_ptr->vel.z());
    tcmsf_result_msg_->mutable_position()->set_lat(vehicle_state_ptr->lla.x() * 180.0 / M_PI);
    tcmsf_result_msg_->mutable_position()->set_lon(vehicle_state_ptr->lla.y() * 180.0 / M_PI);
    tcmsf_result_msg_->mutable_position()->set_height(vehicle_state_ptr->lla.z());
    tcmsf_result_msg_->mutable_gyro_bias()->set_x(vehicle_state_ptr->gyro_bias.x());
    tcmsf_result_msg_->mutable_gyro_bias()->set_y(vehicle_state_ptr->gyro_bias.y());
    tcmsf_result_msg_->mutable_gyro_bias()->set_z(vehicle_state_ptr->gyro_bias.z());
    tcmsf_result_msg_->mutable_acc_bias()->set_x(vehicle_state_ptr->acc_bias.x());
    tcmsf_result_msg_->mutable_acc_bias()->set_y(vehicle_state_ptr->acc_bias.y());
    tcmsf_result_msg_->mutable_acc_bias()->set_z(vehicle_state_ptr->acc_bias.z());
    tcmsf_result_msg_->mutable_vehicle_bias()->set_x(vehicle_state_ptr->vehicle_bias.x());
    tcmsf_result_msg_->mutable_vehicle_bias()->set_y(vehicle_state_ptr->vehicle_bias.y());
    tcmsf_result_msg_->mutable_vehicle_bias()->set_z(vehicle_state_ptr->vehicle_bias.z());
    if (parameters_sgt.get_enable_sd_map_bias_comp()) {
        // 如果使能了sd地图偏置补偿，则输出的map_bias字段输出sd地图偏置值。
        tcmsf_result_msg_->mutable_map_bias()->set_x(vehicle_state_ptr->sdmap_bias_enu.x());
        tcmsf_result_msg_->mutable_map_bias()->set_y(vehicle_state_ptr->sdmap_bias_enu.y());
        tcmsf_result_msg_->mutable_map_bias()->set_z(vehicle_state_ptr->sdmap_bias_enu.z());
    } else {
        // 如果未使能，则map_bias字段输出ld地图修正项。
        tcmsf_result_msg_->mutable_map_bias()->set_x(vehicle_state_ptr->map_bias.x());
        tcmsf_result_msg_->mutable_map_bias()->set_y(vehicle_state_ptr->map_bias.y());
        tcmsf_result_msg_->mutable_map_bias()->set_z(vehicle_state_ptr->map_bias.z());
    }

    tcmsf_result_msg_->set_mileage(vehicle_state_ptr->mileage);

    tcmsf_result_msg_->mutable_attitude_std()->set_x(vehicle_state_ptr->error_state_std[0]);
    tcmsf_result_msg_->mutable_attitude_std()->set_y(vehicle_state_ptr->error_state_std[1]);
    tcmsf_result_msg_->mutable_attitude_std()->set_z(vehicle_state_ptr->error_state_std[2]);
    tcmsf_result_msg_->mutable_velocity_std()->set_x(vehicle_state_ptr->error_state_std[3]);
    tcmsf_result_msg_->mutable_velocity_std()->set_y(vehicle_state_ptr->error_state_std[4]);
    tcmsf_result_msg_->mutable_velocity_std()->set_z(vehicle_state_ptr->error_state_std[5]);
    tcmsf_result_msg_->mutable_position_std()->set_x(vehicle_state_ptr->error_state_std[6]);
    tcmsf_result_msg_->mutable_position_std()->set_y(vehicle_state_ptr->error_state_std[7]);
    tcmsf_result_msg_->mutable_position_std()->set_z(vehicle_state_ptr->error_state_std[8]);
    tcmsf_result_msg_->mutable_gyro_bias_std()->set_x(vehicle_state_ptr->error_state_std[9]);
    tcmsf_result_msg_->mutable_gyro_bias_std()->set_y(vehicle_state_ptr->error_state_std[10]);
    tcmsf_result_msg_->mutable_gyro_bias_std()->set_z(vehicle_state_ptr->error_state_std[11]);
    tcmsf_result_msg_->mutable_acc_bias_std()->set_x(vehicle_state_ptr->error_state_std[12]);
    tcmsf_result_msg_->mutable_acc_bias_std()->set_y(vehicle_state_ptr->error_state_std[13]);
    tcmsf_result_msg_->mutable_acc_bias_std()->set_z(vehicle_state_ptr->error_state_std[14]);
    tcmsf_result_msg_->mutable_vehicle_bias_std()->set_x(vehicle_state_ptr->error_state_std[15]);
    tcmsf_result_msg_->mutable_vehicle_bias_std()->set_y(vehicle_state_ptr->error_state_std[16]);
    tcmsf_result_msg_->mutable_vehicle_bias_std()->set_z(vehicle_state_ptr->error_state_std[17]);
    tcmsf_result_msg_->mutable_map_bias_std()->set_x(vehicle_state_ptr->error_state_std[18]);
    tcmsf_result_msg_->mutable_map_bias_std()->set_y(vehicle_state_ptr->error_state_std[19]);
    tcmsf_result_msg_->mutable_map_bias_std()->set_z(vehicle_state_ptr->error_state_std[20]);

    double heading = -vehicle_state_ptr->eulr_.z() * 180.0 / M_PI;
    if (heading < 0) {
        heading = heading + 360;
    }

    tcmsf_result_msg_->set_gnss_status((byd::modules::tcmsf::Pose_GnssType)vehicle_state_ptr->rtk_status);
    tcmsf_result_msg_->set_fusion_status((byd::modules::tcmsf::Pose_FusionStatus)vehicle_state_ptr->fusion_status);

    // 对coarse_align状态进行防抖处理
    // 避免频繁进入该状态
    coarse_align_debouncer.process_state_frame(vehicle_state_ptr->align_type, vehicle_state_ptr->measurement_timestamp);
    static MSF::State::AlignType align_type_debounced = MSF::State::AlignType::COARSE_ALIGN;
    if (vehicle_state_ptr->align_type == MSF::State::AlignType::COARSE_ALIGN) {
        // 如果coarse_align稳定，则更新状态
        // 否则维持之前的状态
        if (coarse_align_debouncer.is_target_state_stable()) {
            align_type_debounced = vehicle_state_ptr->align_type;
        }
    } else {
        // 如果不是coarse_align状态，则实时更新
        align_type_debounced = vehicle_state_ptr->align_type;
    }

    tcmsf_result_msg_->set_align_status((byd::modules::tcmsf::Pose_AlignType)align_type_debounced);

    tcmsf_result_msg_->set_heading(heading);

    tcmsf_result_msg_->set_zupt_count(vehicle_state_ptr->zupt_imu_bias_estimate_ok_count);
    return 0;
}

/*
  TCMSF主线程，结构如下。以IMU消息驱动整个预测、量测更新。
  基本设计思想是以数据（消息）驱动，这种方式同时适用于在线、离线模式。
  这里IMU消息采用定时阻塞主要是为了主线程能够及时处理模块的退出信息。
  一般IMU消息具备最高的频率且融合定位的运动学主要靠IMU递推，所以以IMU驱动整个更新过程。
  如果量测消息频率高于IMU，那么在这种情况下，量测消息会被浪费掉一些。

                        ┌────────────────┐
                        │                │
                        │                │
  each try will┌────────┴─────────┐ fail │
  block 0.1 s  │ try fuse imu     ├─────►│
  wait for msg └────────┬─────────┘      │
                        │success         │
                        ▼                │
                        │                │
  go on for    ┌────────┴─────────┐      │
  measurement  │ try fuse gnss    │      │
  update only  └────────┬─────────┘      ▲
  if predition          │                │
  success               ▼                │
                        │                │
               ┌────────┴─────────┐      │
               │ try fuse vehicle │      │
               └────────┬─────────┘      │
                        │                │
                        ▼                │
                        │                │
               ┌────────┴─────────┐      │
               │ try fuse vision  │      │
               └────────┬─────────┘      │
                        │                │
                        ▼                │
                        │                │
               ┌────────┴─────────┐      │
               │ try fuse  ...    │      │
               └────────┬─────────┘      │
                        │                │
                        └────────────────┘
 */

int TCMSF_IMPL::start_fusion_daemon(std::function<void()> cb) {
    fusion_thread = std::thread([this, cb] {
        AINFO << "TCMSF daemon thread started!";
        // pthread_setname_np(pthread_self(), "TCMSF_thread");
        bool msf_ready = false;
        while (!fusion_shall_exit.load()) {

            if (!try_fuse_imu(msf_ready)) {
                continue;
            }
            try_fuse_gnss();
            try_fuse_veh();
            try_fuse_vis();
            try_fuse_map();
            try_fuse_db();

            if (msf_ready && cb) {
                cb();
                // 这里添加一条记录，表明初始化完成。
                static auto tcmsf_ready_call_once_ = [this] {
                    // 执行一次的逻辑
                    AINFO << "[startup] TCMSF Ready! imu time: " << fmt::format("{:14.4f}", cur_imu_data.measurement_timestamp);
                    return 0;
                }(); // 定义后立即执行
                (void)tcmsf_ready_call_once_;
            }
        }
        AINFO << "tcmsf thread exit";
    });

    apollo::cyber::scheduler::Instance()->SetInnerThreadAttr("tcmsf_main_t", &fusion_thread);

    return 0;
}

/*
  TCMSF单步模式，手动调用主循环，主要用于离线模式
*/
int TCMSF_IMPL::offline_mode_step(std::function<void()> cb) {
    bool msf_ready = false;

    if (!try_fuse_imu(msf_ready)) {
        return -1;
    }
    try_fuse_gnss();
    try_fuse_veh();
    try_fuse_vis();
    try_fuse_map();
    try_fuse_db();

    if (msf_ready && cb) {
        cb();
        // 对于离线模式，这里也添加一条记录，表明初始化完成。
        static auto tcmsf_ready_call_once_ = [this] {
            // 执行一次的逻辑
            AINFO << "[startup] TCMSF Ready! imu time: " << fmt::format("{:14.4f}", cur_imu_data.measurement_timestamp);
            return 0;
        }(); // 定义后立即执行
        (void)tcmsf_ready_call_once_;
    }
    return 0;
}

} // namespace tcmsf
} // namespace byd