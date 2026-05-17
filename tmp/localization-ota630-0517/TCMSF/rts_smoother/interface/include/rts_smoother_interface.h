#pragma once

#include "rts_base_type.h"
#include <memory>

// RTS平滑

// #define __TCMSF_ENABLE_RTS_SMOOTHER_

namespace RTS {

class RtsSmoother {

public:
    virtual ~RtsSmoother() = default;

public:
    static std::unique_ptr<RtsSmoother> create();

public:
    virtual int insert(const ForwardStepInfo &) = 0;
    virtual int backward()                      = 0;
};
} // namespace RTS