#include "rts_smoother_interface.h"
#include "rts_smoother_impl.h"

namespace RTS {
std::unique_ptr<RtsSmoother> RtsSmoother::create() {
    return std::make_unique<RtsSmootherImpl>("data/tmp/rts_result.csv", RtsSmootherImpl::FULL, RtsSmootherImpl::F_ANN);
}
} // namespace RTS