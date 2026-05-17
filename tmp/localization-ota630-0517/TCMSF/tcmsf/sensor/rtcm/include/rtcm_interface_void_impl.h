#pragma once

#include "rtcm_interface.h"

namespace byd {
namespace tcmsf {
namespace rtcm {
class ResolveVoidImpl : public Resolve {

public:
    virtual void write_base(uint8_t byte) override final;
    virtual void write_rover(uint8_t byte) override final;

public:
    virtual void register_solve_cb(std::function<void(void)> cb_) override final;

    ResolveVoidImpl();
    ResolveVoidImpl(double timepoint[6]);
};
} // namespace rtcm
} // namespace tcmsf
} // namespace byd