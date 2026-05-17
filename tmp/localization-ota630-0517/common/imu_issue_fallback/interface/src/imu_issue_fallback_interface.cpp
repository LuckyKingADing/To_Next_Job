#include "imu_issue_fallback_interface.h"
#include "imu_issue_fallback_impl.h"

namespace MSF::IIF {
std::unique_ptr<ImuIssueFallbackInterface> ImuIssueFallbackInterface::create() {
    return std::make_unique<ImuIssueFallbackImpl>();
}
} // namespace MSF::IIF