#include "sins_interface.h"
#include "ins.h"
#include <thread>

namespace INS {
std::unique_ptr<Sins> Sins::create() {
    return std::make_unique<INS::SinsImpl>();
}
std::unique_ptr<Sins> Sins::create(const AVP &avp_, double ts_) {
    return std::make_unique<INS::SinsImpl>();
}

} // namespace INS