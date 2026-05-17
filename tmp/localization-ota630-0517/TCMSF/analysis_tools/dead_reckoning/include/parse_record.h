#pragma once

#include "modules/msg/drivers_msgs/gps.pb.h"
#include "modules/msg/drivers_msgs/imu.pb.h"
#include "modules/msg/drivers_msgs/veh_info.pb.h"
#include "modules/msg/localization_msgs/localization_info.pb.h"

#include "modules/msg/localization_msgs/result_info.pb.h"

#include "modules/common/topic/topic_gflags.h"

#include "modules/localization/src/TCMSF/tcmsf/signal_process/signal_filter/include/signal_filter.h"

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"

#include "Coord.h"

class PARSER {

public:
    int run(const std::string &, const std::string &);

private:
    MSF::VLPF<3> veh_lpf = MSF::VLPF<3>(100.0, 1.0);
};
