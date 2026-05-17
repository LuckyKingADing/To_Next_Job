#include "rtcm_interface_void_impl.h"

namespace byd {
namespace tcmsf {
namespace rtcm {
void ResolveVoidImpl::write_base(uint8_t byte) {}
void ResolveVoidImpl::write_rover(uint8_t byte) {}
void ResolveVoidImpl::register_solve_cb(std::function<void(void)> cb_) {}
ResolveVoidImpl::ResolveVoidImpl() {}
ResolveVoidImpl::ResolveVoidImpl(double timepoint[6]) {}
} // namespace rtcm
} // namespace tcmsf
} // namespace byd