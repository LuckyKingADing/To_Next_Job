#include <chrono>
#include <functional>
#include <thread>

#include "cyber/common/log.h"

namespace byd {
namespace dr {

class STATE_TIMER {
 private:
  std::chrono::steady_clock::time_point t0{};

 public:
  STATE_TIMER() { reset(); }
  inline void reset() { t0 = std::chrono::steady_clock::now(); }
  inline double dt() {
    auto t1 = std::chrono::steady_clock::now();
    auto dt = t1 - t0;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(dt).count() /
           1e9;
  };
};

}  // namespace dr
}  // namespace byd
