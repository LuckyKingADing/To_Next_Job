#include "time_trigger.h"

namespace byd {
namespace dr {

// void cron_task(std::function<void(double)> func, unsigned int interval) {
//   std::thread([func, interval]() {
//     while (true) {
//       auto t0 = std::chrono::steady_clock::now();
//       auto t1 = t0 + std::chrono::microseconds(interval);
//       func(double(interval) / 1e6);
//       auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
//                     std::chrono::steady_clock::now() - t0)
//                     .count();
//       if (dt > interval) {
//         AWARN << "task consume more time than given interval";
//         continue;
//       }
//       std::this_thread::sleep_until(t1);
//     }
//   }).detach();
// }

}  // namespace dr
}  // namespace byd
