#include "fmt/format.h"
#include "traj_logger.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <type_traits>
#include <vector>

namespace byd {
namespace traj_logger {

class InfoBlock {

public:
    std::string fusion       = "";
    std::string dlat_f       = "";
    std::string dlon_f       = "";
    std::string fusion_mb    = "";
    std::string dlat_f_mb    = "";
    std::string dlon_f_mb    = "";
    std::string RTK          = "";
    std::string dlat_r       = "";
    std::string dlon_r       = "";
    std::string sd_proj      = "";
    std::string dlat_s       = "";
    std::string dlon_s       = "";
    std::string sd_proj_db   = "";
    std::string dlat_s_db    = "";
    std::string dlon_s_db    = "";
    std::string rtk_s        = "";
    std::string align        = "";

    void clear() {
        fusion       = "";
        dlat_f       = "";
        dlon_f       = "";
        fusion_mb    = "";
        dlat_f_mb    = "";
        dlon_f_mb    = "";
        RTK          = "";
        dlat_r       = "";
        dlon_r       = "";
        sd_proj      = "";
        dlat_s       = "";
        dlon_s       = "";
        sd_proj_db   = "";
        dlat_s_db    = "";
        dlon_s_db    = "";
        rtk_s        = "";
        align        = "";
    }
    bool is_ready() {
        return (fusion != "") &&
               (dlat_f != "") &&
               (dlon_f != "") &&
               (fusion_mb != "") &&
               (dlat_f_mb != "") &&
               (dlon_f_mb != "") &&
               (RTK != "") &&
               (dlat_r != "") &&
               (dlon_r != "") &&
               (sd_proj != "") &&
               (dlat_s != "") &&
               (dlon_s != "") &&
               (sd_proj_db != "") &&
               (dlat_s_db != "") &&
               (dlon_s_db != "") &&
               (rtk_s != "") &&
               (align != "");
    }
};

class BatchDeltaLocParser {

private:
    // 因为日志里面不一定有对应的字段，这里使用一些标志位来判断
    // 如果有对应字段且满足条件，则解析到csv，如果不满足条件则直接使用默认值0.0
    bool WITH_GNSS_          = false;
    bool WITH_FUSION_        = false;
    bool WITH_SDMAP_BIAS_    = false;
    bool WITH_SDMAP_PROJ_    = false;
    bool WITH_SDMAP_PROJ_DB_ = false;
    bool WITH_ADDITIONAL_    = false;

private:
    std::vector<LocPoint> batch_buffer_;
    const size_t          batch_size_;
    LocPoint              last_batch_point_;

private:
    // 使用C++17的if constexpr的现代写法
    template <typename T>
    T convertToType(const std::string &token) {
        if constexpr (std::is_same_v<T, std::string>) {
            return token;
        } else {
            std::stringstream converter(token);
            T                 value;

            if (!(converter >> value)) {
                throw std::invalid_argument("转换失败: " + token);
            }

            std::string remaining;
            if (converter >> remaining) {
                throw std::invalid_argument("有多余内容: " + token);
            }
            return value;
        }
    }
    // 主模板函数：解析逗号分隔的数据
    template <typename T>
    std::vector<T> parseCommaSeparatedData(const std::string &input, const std::string &prefix = "") {
        std::vector<T> result;

        // 如果指定了前缀，查找并截取前缀之后的内容
        std::string dataStr = input;
        if (!prefix.empty()) {
            size_t prefixPos = dataStr.find(prefix);
            if (prefixPos != std::string::npos) {
                dataStr = dataStr.substr(prefixPos + prefix.length());
            }
        }

        // 去除可能的首尾空格
        dataStr.erase(0, dataStr.find_first_not_of(" \t\n\r"));
        dataStr.erase(dataStr.find_last_not_of(" \t\n\r") + 1);

        // 使用stringstream进行分割
        std::stringstream ss(dataStr);
        std::string       token;

        while (std::getline(ss, token, ',')) {
            // 去除每个token的首尾空格
            token.erase(0, token.find_first_not_of(" \t\n\r"));
            token.erase(token.find_last_not_of(" \t\n\r") + 1);

            if (!token.empty()) {
                // 使用类型特化的转换函数
                result.push_back(convertToType<T>(token));
            }
        }

        return result;
    }

public:
    BatchDeltaLocParser(size_t batch_size = 10) :
        batch_size_(batch_size) {
        batch_buffer_.reserve(batch_size_);
    }

    std::string parseLocData(const InfoBlock &info_) {
        std::string csv_;

        auto fusion_       = parseCommaSeparatedData<double>(info_.fusion, "[Fusion]");
        auto dlat_f_       = parseCommaSeparatedData<double>(info_.dlat_f, "<dlat_f>");
        auto dlon_f_       = parseCommaSeparatedData<double>(info_.dlon_f, "<dlon_f>");
        auto fusion_mb_    = parseCommaSeparatedData<double>(info_.fusion_mb, "[Fus_mb]");
        auto dlat_f_mb_    = parseCommaSeparatedData<double>(info_.dlat_f_mb, "<dlat_f_mb>");
        auto dlon_f_mb_    = parseCommaSeparatedData<double>(info_.dlon_f_mb, "<dlon_f_mb>");
        auto RTK_          = parseCommaSeparatedData<double>(info_.RTK, "[RTK]");
        auto dlat_r_       = parseCommaSeparatedData<double>(info_.dlat_r, "<dlat_r>");
        auto dlon_r_       = parseCommaSeparatedData<double>(info_.dlon_r, "<dlon_r>");
        auto sd_proj_      = parseCommaSeparatedData<double>(info_.sd_proj, "[Sd_proj]");
        auto dlat_s_       = parseCommaSeparatedData<double>(info_.dlat_s, "<dlat_s>");
        auto dlon_s_       = parseCommaSeparatedData<double>(info_.dlon_s, "<dlon_s>");
        auto sd_proj_db_   = parseCommaSeparatedData<double>(info_.sd_proj_db, "[Sd_db_mid]");
        auto dlat_s_db_    = parseCommaSeparatedData<double>(info_.dlat_s_db, "<dlat_s_db>");
        auto dlon_s_db_    = parseCommaSeparatedData<double>(info_.dlon_s_db, "<dlon_s_db>");
        auto rtk_s_        = parseCommaSeparatedData<int>(info_.rtk_s, "[rtk_s]");
        auto align_        = parseCommaSeparatedData<int>(info_.align, "[align]");

        if (fusion_.size() == 3 &&
            dlat_f_.size() == batch_size_ - 1 &&
            dlon_f_.size() == batch_size_ - 1) {
            WITH_FUSION_ = true;
        } else {
            WITH_FUSION_ = false;
        }

        if (RTK_.size() == 2 &&
            dlat_r_.size() == batch_size_ - 1 &&
            dlon_r_.size() == batch_size_ - 1) {
            WITH_GNSS_ = true;
        } else {
            WITH_GNSS_ = false;
        }

        if (fusion_mb_.size() == 2 &&
            dlat_f_mb_.size() == batch_size_ - 1 &&
            dlon_f_mb_.size() == batch_size_ - 1) {
            WITH_SDMAP_BIAS_ = true;
        } else {
            WITH_SDMAP_BIAS_ = false;
        }

        if (sd_proj_.size() == 2 &&
            dlat_s_.size() == batch_size_ - 1 &&
            dlon_s_.size() == batch_size_ - 1) {
            WITH_SDMAP_PROJ_ = true;
        } else {
            WITH_SDMAP_PROJ_ = false;
        }

        if (sd_proj_db_.size() == 2 &&
            dlat_s_db_.size() == batch_size_ - 1 &&
            dlon_s_db_.size() == batch_size_ - 1) {
            WITH_SDMAP_PROJ_DB_ = true;
        } else {
            WITH_SDMAP_PROJ_DB_ = false;
        }

        if (rtk_s_.size() == batch_size_ &&
            align_.size() == batch_size_) {
            WITH_ADDITIONAL_ = true;
        } else {
            WITH_ADDITIONAL_ = false;
        }

        {
            LocPoint pre_p, cur_p;
            if (WITH_FUSION_) {
                pre_p.latitude_fusion  = fusion_.at(0);
                pre_p.longitude_fusion = fusion_.at(1);
                pre_p.timestamp_fusion = fusion_.at(2);
            }
            if (WITH_SDMAP_BIAS_) {
                pre_p.latitude_fusion_map_b  = fusion_mb_.at(0);
                pre_p.longitude_fusion_map_b = fusion_mb_.at(1);
            }
            if (WITH_GNSS_) {
                pre_p.latitude_rtk  = RTK_.at(0);
                pre_p.longitude_rtk = RTK_.at(1);
            }
            if (WITH_SDMAP_PROJ_) {
                pre_p.latitude_sdmap_proj  = sd_proj_.at(0);
                pre_p.longitude_sdmap_proj = sd_proj_.at(1);
            }
            if (WITH_SDMAP_PROJ_DB_) {
                pre_p.latitude_sdmap_proj_db_mid  = sd_proj_db_.at(0);
                pre_p.longitude_sdmap_proj_db_mid = sd_proj_db_.at(1);
            }
            if (WITH_ADDITIONAL_) {
                pre_p.align_type = align_.at(0);
                pre_p.rtk_status = rtk_s_.at(0);
            }
            batch_buffer_.clear();
            batch_buffer_.push_back(pre_p);
            for (size_t i = 0; i < batch_size_ - 1; i++) {
                if (WITH_FUSION_) {
                    cur_p.latitude_fusion  = pre_p.latitude_fusion + dlat_f_.at(i) / DELTA_SCALE;
                    cur_p.longitude_fusion = pre_p.longitude_fusion + dlon_f_.at(i) / DELTA_SCALE;
                    cur_p.timestamp_fusion = pre_p.timestamp_fusion + 0.1 * (i + 1);
                }
                if (WITH_SDMAP_BIAS_) {
                    cur_p.latitude_fusion_map_b  = pre_p.latitude_fusion_map_b + dlat_f_mb_.at(i) / DELTA_SCALE;
                    cur_p.longitude_fusion_map_b = pre_p.longitude_fusion_map_b + dlon_f_mb_.at(i) / DELTA_SCALE;
                }
                if (WITH_GNSS_) {
                    cur_p.latitude_rtk  = pre_p.latitude_rtk + dlat_r_.at(i) / DELTA_SCALE;
                    cur_p.longitude_rtk = pre_p.longitude_rtk + dlon_r_.at(i) / DELTA_SCALE;
                }
                if (WITH_SDMAP_PROJ_) {
                    cur_p.latitude_sdmap_proj  = pre_p.latitude_sdmap_proj + dlat_s_.at(i) / DELTA_SCALE;
                    cur_p.longitude_sdmap_proj = pre_p.longitude_sdmap_proj + dlon_s_.at(i) / DELTA_SCALE;
                }
                if (WITH_SDMAP_PROJ_DB_) {
                    cur_p.latitude_sdmap_proj_db_mid  = pre_p.latitude_sdmap_proj_db_mid + dlat_s_db_.at(i) / DELTA_SCALE;
                    cur_p.longitude_sdmap_proj_db_mid = pre_p.longitude_sdmap_proj_db_mid + dlon_s_db_.at(i) / DELTA_SCALE;
                }
                if (WITH_ADDITIONAL_) {
                    cur_p.align_type = align_.at(i + 1);
                    cur_p.rtk_status = rtk_s_.at(i + 1);
                }

                batch_buffer_.push_back(cur_p);
                pre_p = cur_p;
            }
            for (auto &iter : batch_buffer_) {
                csv_ += iter.DebugStr();
            }
        }

        return csv_;
    }
};

} // namespace traj_logger
} // namespace byd

int main(int argc, char **argv) {
    if (argc != 3) {
        fmt::print("usage: <PARSER> log_file output_file");
        return -1;
    }
    std::string log_path(argv[1]);
    std::string out_path(argv[2]);

    std::fstream log_fs(log_path, std::ios::in);
    std::fstream out_fs(out_path, std::ios::out);
    if (!log_fs.is_open() || !out_fs.is_open()) {
        fmt::print("file path errror!");
        return -1;
    }

    std::string traj_info_first_line =
        "fus_lat,"         // 1
        "fus_lon,"         // 2
        "fus_lat_mb,"      // 3
        "fus_lon_mb,"      // 4
        "gnss_lat,"        // 5
        "gnss_lon,"        // 6
        "proj_lat,"        // 7
        "proj_lon,"        // 8
        "proj_db_mid_lat," // 9
        "proj_db_mid_lon," // 10
        "RTK_sta,"         // 11
        "align_sta\n";     // 12
    out_fs << traj_info_first_line;

    byd::traj_logger::BatchDeltaLocParser parser{20};

    std::string                 cur_line;
    byd::traj_logger::InfoBlock info_block;
    while (std::getline(log_fs, cur_line)) {
        if (cur_line.find("[Fusion]") != std::string::npos) {
            info_block.clear();
            info_block.fusion = cur_line;
        }
        if (cur_line.find("<dlat_f>") != std::string::npos) {
            info_block.dlat_f = cur_line;
        }
        if (cur_line.find("<dlon_f>") != std::string::npos) {
            info_block.dlon_f = cur_line;
        }
        if (cur_line.find("[Fus_mb]") != std::string::npos) {
            info_block.fusion_mb = cur_line;
        }
        if (cur_line.find("<dlat_f_mb>") != std::string::npos) {
            info_block.dlat_f_mb = cur_line;
        }
        if (cur_line.find("<dlon_f_mb>") != std::string::npos) {
            info_block.dlon_f_mb = cur_line;
        }
        if (cur_line.find("[RTK]") != std::string::npos) {
            info_block.RTK = cur_line;
        }
        if (cur_line.find("<dlat_r>") != std::string::npos) {
            info_block.dlat_r = cur_line;
        }
        if (cur_line.find("<dlon_r>") != std::string::npos) {
            info_block.dlon_r = cur_line;
        }
        if (cur_line.find("[Sd_proj]") != std::string::npos) {
            info_block.sd_proj = cur_line;
        }
        if (cur_line.find("<dlat_s>") != std::string::npos) {
            info_block.dlat_s = cur_line;
        }
        if (cur_line.find("<dlon_s>") != std::string::npos) {
            info_block.dlon_s = cur_line;
        }
        if (cur_line.find("[Sd_db_mid]") != std::string::npos) {
            info_block.sd_proj_db = cur_line;
        }
        if (cur_line.find("<dlat_s_db>") != std::string::npos) {
            info_block.dlat_s_db = cur_line;
        }
        if (cur_line.find("<dlon_s_db>") != std::string::npos) {
            info_block.dlon_s_db = cur_line;
        }
        if (cur_line.find("[rtk_s]") != std::string::npos) {
            info_block.rtk_s = cur_line;
        }
        if (cur_line.find("[align]") != std::string::npos) {
            info_block.align = cur_line;
            out_fs << parser.parseLocData(info_block);
        }
    }
    return 0;
}