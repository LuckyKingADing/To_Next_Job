#include "dr_interface.h"

#include "dr.h"

namespace byd {
namespace dr {

std::unique_ptr<DR_interface> DR_interface::create() {
  return std::make_unique<DEAD_RECKONING>();
}

}  // namespace dr
}  // namespace byd