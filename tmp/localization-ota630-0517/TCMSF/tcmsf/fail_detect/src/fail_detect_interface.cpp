#include "fail_detect_interface.h"
#include "fail_detect_interface_impl.h"

namespace byd {
namespace tcmsf {
namespace fail_detect {
std::unique_ptr<IMUFailDetect> IMUFailDetect::create() {
    return std::make_unique<IMUFailDetectImpl>();
}
} // namespace fail_detect
} // namespace tcmsf
} // namespace byd