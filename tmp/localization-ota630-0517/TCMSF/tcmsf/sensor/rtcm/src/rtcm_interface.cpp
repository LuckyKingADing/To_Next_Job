#include "rtcm_interface.h"
#include "rtcm_interface_void_impl.h"
// #include "parse.h"

#include "cyber/common/log.h"

namespace byd {
namespace tcmsf {
namespace rtcm {

std::unique_ptr<Resolve> Resolve::create() {
    return std::make_unique<ResolveVoidImpl>();
}

std::unique_ptr<Resolve> Resolve::create(double timepoint[6]) {
    return std::make_unique<ResolveVoidImpl>(timepoint);
}

Resolve::~Resolve() {
}

} // namespace rtcm
} // namespace tcmsf
} // namespace byd