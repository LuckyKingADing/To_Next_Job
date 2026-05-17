#include "state_type.h"
#include "rigid_transform.h"

namespace MSF {

// 使用参考位置和ENU修正量
// 得到地图坐标系下的位置
inline Eigen::Vector3d GetMapPos(const Eigen::Vector3d &lla0, // lat-lon-height
                                 const Eigen::Vector3d &dpos  // delta position
) {
    byd::geo::Trans geotrans;
    auto            lla_ = geotrans.enu2blh({lla0.x() * 180.0 / M_PI, lla0.y() * 180.0 / M_PI, lla0.z()}, {dpos.x(), dpos.y(), dpos.z()});
    return Eigen::Vector3d{lla_.b / 180.0 * M_PI, lla_.l / 180.0 * M_PI, lla_.h};
}

// 使用当前姿态和姿态修正量
// 得到在地图上的姿态
inline Eigen::Quaterniond GetMapAtt(
    const Eigen::Quaterniond att0, //
    double                   dyaw  //
) {
    Eigen::Quaterniond q_ref2map = INS::euler2quaternion({0.0, 0.0, dyaw});
    return q_ref2map * att0;
}

// 载体系映射，从IMU系映射到自车系
State State::ToVehicle() {

    // IMU壳体系定义，右-前-上
    // 车辆车体系定义，右-前-上
    // 此处需要将状态从IMU壳体系转换到车体系，需要考虑安装杆臂和安装角偏差。
    // 因为纵轴（横滚）的安装角误差在轮速融合过程中难以观测，此处只考虑俯仰角和航向角误差。
    State vehicle_state_ = *this;

    Eigen::Matrix3d C_imu2vehicle = this->C_imu2vehicle;

    vehicle_state_.att  = this->att * this->q_imu2vehicle.conjugate();
    vehicle_state_.gyro = C_imu2vehicle * this->gyro;
    vehicle_state_.acc  = C_imu2vehicle * this->acc;

    Eigen::Matrix3d C_veh2ref = this->C_b2n * C_imu2vehicle.transpose();

    if (parameters_sgt.get_output_position_center() == parameters_sgt.GNSS_ANTENNA) {
        // 调试的时候，可以把TCMSF的最终输出点，映射到GNSS天线的位置。这样方便对比TCMSF输出与GNSS观测的区别。
        vehicle_state_.vel = this->vel + C_veh2ref * (INS::Skew(vehicle_state_.web) * C_imu2vehicle * vehicle_state_.lever_imu2gnss);
        vehicle_state_.lla = this->lla + (this->MpvCnb * C_imu2vehicle * vehicle_state_.lever_imu2gnss);
    } else if (parameters_sgt.get_output_position_center() == parameters_sgt.VEHICLE_REAR_AXLE_CENTER) {
        // 输出的时候，映射到自车系。
        vehicle_state_.vel = this->vel + C_veh2ref * (INS::Skew(vehicle_state_.web) * C_imu2vehicle * vehicle_state_.lever_imu2vehicle);
        vehicle_state_.lla = this->lla + (this->MpvCnb * C_imu2vehicle * vehicle_state_.lever_imu2vehicle);
    } else {
        // 默认输出IMU位置
    }

    if (parameters_sgt.get_output_reference_frame() == parameters_sgt.ENU_NAVI_FRAME) {
        // 默认为ENU，此处无需操作
    } else if (parameters_sgt.get_output_reference_frame() == parameters_sgt.MAP_FRAME) {
        // 参考系为地图时，需要将地图误差附加到定位输出上
        vehicle_state_.lla = GetMapPos(vehicle_state_.lla.eval(), C_veh2ref * Eigen::Vector3d{vehicle_state_.map_bias.x(), vehicle_state_.map_bias.y(), 0.0});
        vehicle_state_.att = GetMapAtt(vehicle_state_.att, vehicle_state_.map_bias.z());
    } else {
        // 默认参考系为ENU
    }

    vehicle_state_.vel_ego = C_veh2ref.transpose() * vehicle_state_.vel;

    vehicle_state_.eulr_ = INS::quaternion2euler(vehicle_state_.att);

    return vehicle_state_;
}

State State::FromVehicleToImu() {

    this->q_imu2vehicle         = INS::euler2quaternion({this->vehicle_bias.x(), 0.0, this->vehicle_bias.z()});
    this->wheel_spd_scale_bias_ = this->vehicle_bias.y();

    State           imu_state_ = *this;
    Eigen::Matrix3d C_veh2ref  = this->att.toRotationMatrix();

    Eigen::Matrix3d C_imu2vehicle = this->C_imu2vehicle;

    imu_state_.vel = this->vel - C_veh2ref * (INS::Skew(this->gyro) * C_imu2vehicle * this->lever_imu2vehicle);
    imu_state_.att = this->att * this->q_imu2vehicle;
    imu_state_.lla = GetMapPos(imu_state_.lla.eval(), C_veh2ref * Eigen::Vector3d{-imu_state_.map_bias.x() - imu_state_.lever_imu2vehicle.x(), -imu_state_.map_bias.y() - imu_state_.lever_imu2vehicle.y(), -imu_state_.lever_imu2vehicle.z()});
    imu_state_.att = GetMapAtt(imu_state_.att, -imu_state_.map_bias.z());
    return imu_state_;
}

// 计算自车坐标系下自车的速度信息
Eigen::Vector3d State::GetEgoVel() {
    // 获取自车系下速度
    // 调用对象需要是IMU系下信息

    Eigen::Matrix3d C_imu2vehicle = this->C_imu2vehicle;
    Eigen::Matrix3d C_veh2ref     = this->C_b2n * C_imu2vehicle.transpose();
    Eigen::Vector3d vel_n         = this->vel + C_veh2ref * (INS::Skew(this->web) * C_imu2vehicle * this->lever_imu2vehicle);
    Eigen::Vector3d vel_ego       = C_veh2ref.transpose() * vel_n;

    return vel_ego;
}

// 获取融合投影到卫星天线的位置
Eigen::Vector3d State::GetAntennaBLH() {
    Eigen::Matrix3d C_imu2vehicle = this->C_imu2vehicle;
    return this->lla + (this->MpvCnb * C_imu2vehicle * this->lever_imu2gnss);
}

// 位置重置的时候，制图误差的消除策略
// 这个策略需要在有VF的时候才启用，否则无意义。
void State::mapBiasAblationWhenReset(const Eigen::Vector3d &new_lla_) {

    Eigen::Vector3d dpos_ = local_trans.LLAtoEgoRfu(this->lla, new_lla_, this->att * this->q_imu2vehicle.conjugate(), true);

    static constexpr double MAP_BIAS_ABLATION_BOUND = 1.6;

    double lat_virtual_mapbias = this->map_bias.x() - dpos_.x();

    AINFO << "\nmapBiasAblationWhenReset info:\n"
          << "reset delta pos RFU: " << dpos_.x() << " " << dpos_.y() << " " << dpos_.z() << std::endl
          << "map bias pos RFU: " << this->map_bias.x() << " " << this->map_bias.y() << std::endl
          << "virtual lat mapbias: " << lat_virtual_mapbias;

    if (std::fabs(lat_virtual_mapbias) < MAP_BIAS_ABLATION_BOUND) {
        //
        this->map_bias.x() = lat_virtual_mapbias;
    } else {
        if (lat_virtual_mapbias > 0.0) {
            this->map_bias.x() = MAP_BIAS_ABLATION_BOUND;
        } else {
            this->map_bias.x() = -MAP_BIAS_ABLATION_BOUND;
        }
    }
    AINFO << "lat mapbias ablation. New mapbias.x: " << this->map_bias.x();
}

// 在lla上附加sdmap偏置量，返回新状态
State State::WithSdmapBias() {
    if (!parameters_sgt.get_enable_sd_map_bias_comp()) {
        return *this;
    }
    State state_ = *this;
    state_.lla   = this->lla + this->Mpv * this->sdmap_bias_enu;
    return state_;
}

// 在lla上去掉sdmap偏置量，返回新状态（逆过程）
State State::WithoutSdmapBias() {
    if (!parameters_sgt.get_enable_sd_map_bias_comp()) {
        return *this;
    }
    State state_ = *this;
    state_.lla   = this->lla - this->Mpv * this->sdmap_bias_enu;
    return state_;
}

} // namespace MSF