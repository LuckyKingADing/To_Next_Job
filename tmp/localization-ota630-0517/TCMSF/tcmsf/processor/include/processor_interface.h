#pragma once

#include "base_type.h"
#include "kalman_filter.h"
#include <functional>
#include <memory>

namespace byd {
namespace tcmsf {
namespace processor {

using namespace MSF;

class Processor {
public:
    virtual ~Processor();

public:
    static std::unique_ptr<Processor> create();

public:
    virtual bool ProcessImuData(const ImuDataPtr imu_data_ptr, double dt_, StatePtr state_ptr)                                                        = 0;
    virtual bool ProcessGnssData(const GnssDataPtr gps_data_ptr, StatePtr state_ptr, const KinematicDataPtr delta_ptr, const KinematicDataPtr dr_ptr) = 0;
    virtual bool ProcessVehicleData(const VehicleDataPtr vehicle_data_ptr, StatePtr state_ptr, const KinematicDataPtr delta_ptr)                      = 0;
    virtual bool ProcessVisionData(const VisionFusionDataPtr vis_data_ptr, StatePtr state_ptr, const KinematicDataPtr delta_ptr)                      = 0;
    virtual bool ProcessMapData(const MapPosDataPtr map_data_ptr, StatePtr state_ptr, const KinematicDataPtr delta_ptr)                               = 0;
    virtual bool ProcessDbData(const DbDataPtr db_data_ptr, StatePtr state_ptr)                                                                        = 0;
    virtual void AttitudeReference(const MSF::ImuData &, const MSF::VehicleData &, double dt_, StatePtr state_ptr, const KinematicDataPtr cur_dr_ptr) = 0;

public:
    KfPtr<21, 3> kf_ptr_ = nullptr;
};
} // namespace processor
} // namespace tcmsf
} // namespace byd