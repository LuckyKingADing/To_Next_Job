#pragma once

#include <cstdint>
#include <functional>
#include <memory>

namespace byd {
namespace tcmsf {
namespace rtcm {
class Resolve {

public:
    virtual ~Resolve();

public:
    static std::unique_ptr<Resolve> create();
    static std::unique_ptr<Resolve> create(double timepoint[6]);

public:
    virtual void write_base(uint8_t byte)                         = 0;
    virtual void write_rover(uint8_t byte)                        = 0;
    virtual void register_solve_cb(std::function<void(void)> cb_) = 0;
};
} // namespace rtcm
} // namespace tcmsf
} // namespace byd