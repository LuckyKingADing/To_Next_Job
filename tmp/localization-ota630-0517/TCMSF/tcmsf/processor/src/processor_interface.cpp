#include "processor_interface.h"
#include "processor_interface_impl.h"

namespace byd {
namespace tcmsf {
namespace processor {

Processor::~Processor() {
    AINFO << "exit processor interface";
}

std::unique_ptr<Processor> Processor::create() {
    return std::make_unique<ProcessorImpl>();
}
} // namespace processor
} // namespace tcmsf
} // namespace byd