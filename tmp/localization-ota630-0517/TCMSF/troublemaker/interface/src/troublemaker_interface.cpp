#include "troublemaker_impl.h"

namespace TroubleMaker {
std::unique_ptr<SensorModifyBase> SensorModifyBase::create() {
    return std::make_unique<SensorModifyImpl>();
}
} // namespace TroubleMaker