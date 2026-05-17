#include "tcmsf_interface.h"
#include "tcmsf_interface_impl.h"

namespace byd {
namespace tcmsf {

std::unique_ptr<TCMSF> TCMSF::create(const std::string &lever_file_) {
    return std::make_unique<TCMSF_IMPL>(lever_file_);
}
} // namespace tcmsf
} // namespace byd