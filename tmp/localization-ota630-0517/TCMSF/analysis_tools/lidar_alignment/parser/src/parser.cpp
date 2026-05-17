#include "parser.h"
#include <filesystem>

namespace analysis {

void Parser::register_msg_cb_func(std::function<void(apollo::cyber::record::RecordMessage &)> func) {
    msg_cb_ = func;
}

int Parser::parse(const std::string &record_dir) {

    namespace fs = std::filesystem;
    if (!fs::exists(record_dir)) {
        return -1;
    }
    fs::directory_entry entry(record_dir);
    if (!entry.is_directory()) {
        return -1;
    }

    std::vector<std::string> records;

    fs::directory_iterator iters(record_dir);

    for (auto &iter : iters) {
        records.push_back(iter.path());
    }

    std::sort(records.begin(), records.end());

    for (auto record_ : records) {
        std::cout << record_ << "\n";
        apollo::cyber::record::RecordReader  reader(record_);
        apollo::cyber::record::RecordMessage msg_;

        while (reader.ReadMessage(&msg_)) {
            msg_cb_(msg_);
        }
    }
    return 0;
}

} // namespace analysis