#pragma once

#include "modules/common/topic/topic_gflags.h"
#include "modules/localization/src/TCMSF/tcmsf/sensor/rtcm/include/rtcm_interface.h"

#include "modules/msg/drivers_msgs/rtcm.pb.h"
// #include "tcmsf_config.h"
#include "tcmsf_interface.h"
// #include "tcmsf_interface_impl.h"
#include "tcmsf_timer.h"

#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

// #define _TCMSF_WRITE_RESULT_TO_RECORD_

class TCMSF_ {
private:
    std::unique_ptr<byd::tcmsf::TCMSF> tcmsf_ = nullptr;

public:
    TCMSF_(const std::string &);

    // 设置pvtlc模式数据源 ("pvt" 或 "gnss")
    void set_pvtlc_source(const std::string &source);

    // 获取TCMSF实例指针（用于初始化评估测试工具）
    byd::tcmsf::TCMSF* get_tcmsf() { return tcmsf_.get(); }

public:
    int run(const std::string &, const std::string &);
};
