#pragma once

#include "modules/msg/drivers_msgs/gps.pb.h"
#include "modules/msg/drivers_msgs/imu.pb.h"
#include "modules/msg/drivers_msgs/veh_info.pb.h"
#include "modules/msg/localization_msgs/localization_info.pb.h"
#include "modules/msg/localization_msgs/result_info.pb.h"
#include "modules/msg/localization_msgs/tcmsf.pb.h"
#include "modules/msg/localization_msgs/vf_result.pb.h"
#include "modules/msg/localization_msgs/local_map.pb.h"
#include "modules/msg/orin_msgs/routing_map.pb.h"

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
};
