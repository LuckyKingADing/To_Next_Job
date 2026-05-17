#pragma once

#include "cyber/record/record_reader.h"
// #include "cyber/record/record_writer.h"
#include <functional>
#include <string>

namespace analysis {
class Parser {
private:
    std::function<void(apollo::cyber::record::RecordMessage &)> msg_cb_;

public:
    void register_msg_cb_func(std::function<void(apollo::cyber::record::RecordMessage &)> func);
    int  parse(const std::string &record_dir);
};
} // namespace analysis