#pragma once

#include "base_type.h"

// 定义了一些变换函数，用于更改传感器的输入
// 主要是为了在回灌的时候，验证定位算法性能，以及应对异常输入的表现。

// #define __TCMSF_ENABLE_TROUBLE_MAKER_

namespace TroubleMaker {

using namespace MSF;

class SensorModifyBase {

public:
    virtual ~SensorModifyBase() = default;

public:
    static std::unique_ptr<SensorModifyBase> create();

public:
    virtual bool sensor_modify(ImuData &)          = 0;
    virtual bool sensor_modify(GnssData &)         = 0;
    virtual bool sensor_modify(VehicleData &)      = 0;
    virtual bool sensor_modify(VisionFusionData &) = 0;
};
} // namespace TroubleMaker