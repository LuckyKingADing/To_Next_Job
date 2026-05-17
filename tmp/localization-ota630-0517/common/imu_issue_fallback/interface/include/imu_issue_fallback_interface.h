#pragma once

#include "modules/msg/drivers_msgs/imu.pb.h"
#include "modules/msg/drivers_msgs/veh_info.pb.h"

#include <memory>

// IMU异常兜底

// 这里可以考虑通过宏定义来控制是否启用IMU兜底
// #define __TCMSF_ENABLE_IMU_ISSUE_FALLBACK_

namespace MSF::IIF {

using byd::msg::drivers::Imu;
using byd::msg::drivers::VehInfo;

// 使用方法
// 1、使用create创建imu兜底实例
// 2、在定位模块接收imu消息处，调用insert_imu，使用返回值替代原消息
// 3、在定位模块接收veh消息处，调用insert_veh。
class ImuIssueFallbackInterface {

public:
    virtual ~ImuIssueFallbackInterface() = default;

public:
    // 创建IMU兜底实例
    static std::unique_ptr<ImuIssueFallbackInterface> create();

public:
    // 插入IMU消息，使用返回值作为新的IMU消息，传入到下游。
    virtual std::shared_ptr<Imu> insert_imu(const std::shared_ptr<Imu>) = 0;
    // 插入VEH消息。
    virtual int insert_veh(const std::shared_ptr<VehInfo>) = 0;
};
} // namespace MSF::IIF