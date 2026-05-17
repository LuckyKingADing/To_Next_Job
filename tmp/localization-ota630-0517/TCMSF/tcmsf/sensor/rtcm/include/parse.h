#pragma once

#include "rtcm_interface.h"

#include "cyber/common/log.h"
#include "modules/localization/src/TCMSF/tcmsf/sensor/rtcm/include/dump.h"
// #include "modules/localization/src/TCMSF/third_party/RTKLIB-b34k/src/rtklib.h"
#include "fmt/color.h"
#include "fmt/core.h"
#include "fmt/format.h"
#include "modules/localization/src/TCMSF/third_party/concurrentqueue-1.0.4/include/blockingconcurrentqueue.h"
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <pthread.h>
#include <thread>

namespace byd {
namespace tcmsf {
namespace rtcm {

// 100kB message queue for buffering rtcm messages
#define RTCM_MSG_QUEUE_LENGTH 100000

using namespace moodycamel;
class ResolveImpl : public Resolve {

private:
    gnss::GnssInfo gnss_info;

private:
    BlockingConcurrentQueue<uint8_t> rover_msg_queue, base_msg_queue;
    uint64_t                         rover_msg_byte_count = 0, base_msg_byte_count = 0;
    uint64_t                         rover_msg_queue_overflow_idx = 0, base_msg_queue_overflow_idx = 0;
    rtcm_t                           rover_rtcm, base_rtcm;
    std::mutex                       base_rtcm_mutex;

    std::mutex                change_cb;
    std::function<void(void)> solve_cb = nullptr;

public:
    ResolveImpl();
    ResolveImpl(double timepoint[6]);
    ~ResolveImpl();

public:
    virtual void write_base(uint8_t byte) override final;
    virtual void write_rover(uint8_t byte) override final;

public:
    virtual void register_solve_cb(std::function<void(void)> cb_) override final;

private:
    void start_daemon();

private:
    void parse_base();
    void parse_rover();

private:
    std::atomic<bool> shall_exit;
    std::thread       parse_base_t, parse_rover_t;

private:
    obs_t    obs;
    rtk_t    rtk;
    prcopt_t opt;
    bool     resolve();
};
} // namespace rtcm
} // namespace tcmsf
} // namespace byd
