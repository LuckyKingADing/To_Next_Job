#pragma once
#include <chrono>

#include "cyber/common/log.h"

namespace byd {
namespace tcmsf {
namespace time {

class Timer {
    std::chrono::steady_clock::time_point t0{};
    std::string                           state{};

public:
    Timer() { reset(); }
    Timer(const std::string state_name) {
        reset();
        state = state_name;
    }
    inline void   reset() { t0 = std::chrono::steady_clock::now(); }
    inline double dt() {
        auto t1 = std::chrono::steady_clock::now();
        auto dt = t1 - t0;
        return std::chrono::duration_cast<std::chrono::nanoseconds>(dt).count() / 1e9;
    };
    inline void info() {
        AINFO << "State " << state << " consists " << dt() << " seconds";
    }
};

} // namespace time
} // namespace tcmsf
} // namespace byd
