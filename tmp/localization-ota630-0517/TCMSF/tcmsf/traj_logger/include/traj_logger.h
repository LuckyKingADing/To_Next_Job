#pragma once

#include "cyber/common/log.h"
#include "fmt/format.h"
#include <cmath>
#include <string>
#include <vector>

namespace byd {
namespace traj_logger {

constexpr static double DELTA_SCALE = 1e9f;

struct LocPoint {
    double timestamp_fusion;           // 融合时间戳
    double latitude_fusion;            // 融合纬度
    double longitude_fusion;           // 融合经度
    double latitude_fusion_map_b;      // 融合纬度+mapbias
    double longitude_fusion_map_b;     // 融合经度+mapbias
    double latitude_rtk;               // 卫星纬度
    double longitude_rtk;              // 卫星经度
    double latitude_sdmap_proj;        // 投影点纬度
    double longitude_sdmap_proj;       // 投影点经度
    double latitude_sdmap_proj_db_mid; // DB中心投影点纬度
    double longitude_sdmap_proj_db_mid;// DB中心投影点经度
    int    rtk_status;                 // RTK 状态
    int    align_type;                 // 对准状态

    LocPoint(double lat_f, double lon_f, double lat_f_mb, double lon_f_mb, double lat_rtk, double lon_rtk, double lat_sd_proj, double lon_sd_proj, double lat_sd_proj_db_mid, double lon_sd_proj_db_mid, int rtk_, int align_, double timestamp_f) {
        timestamp_fusion              = timestamp_f;
        latitude_fusion               = lat_f;
        longitude_fusion              = lon_f;
        latitude_fusion_map_b         = lat_f_mb;
        longitude_fusion_map_b        = lon_f_mb;
        latitude_rtk                  = lat_rtk;
        longitude_rtk                 = lon_rtk;
        latitude_sdmap_proj           = lat_sd_proj;
        longitude_sdmap_proj          = lon_sd_proj;
        latitude_sdmap_proj_db_mid    = lat_sd_proj_db_mid;
        longitude_sdmap_proj_db_mid   = lon_sd_proj_db_mid;
        rtk_status                    = rtk_;
        align_type                    = align_;
    }
    LocPoint() {
        timestamp_fusion              = 0.0;
        latitude_fusion               = 0.0;
        longitude_fusion              = 0.0;
        latitude_fusion_map_b         = 0.0;
        longitude_fusion_map_b        = 0.0;
        latitude_rtk                  = 0.0;
        longitude_rtk                 = 0.0;
        latitude_sdmap_proj           = 0.0;
        longitude_sdmap_proj          = 0.0;
        latitude_sdmap_proj_db_mid    = 0.0;
        longitude_sdmap_proj_db_mid   = 0.0;
        rtk_status                    = 0;
        align_type                    = 0;
    }
    std::string DebugStr(bool with_time = false) {
        std::string t_str    = fmt::format("{:>14.4f}", timestamp_fusion);
        std::string data_str = fmt::format("{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f},{:>14.10f},{:d},{:d}\n", //
                                           latitude_fusion, longitude_fusion, latitude_fusion_map_b, longitude_fusion_map_b, latitude_rtk, longitude_rtk, latitude_sdmap_proj, longitude_sdmap_proj, latitude_sdmap_proj_db_mid, longitude_sdmap_proj_db_mid, rtk_status, align_type);
        if (with_time) {
            data_str = t_str + "," + data_str;
        }
        return data_str;
    }
};

// 批量处理提高压缩率
class BatchDeltaLocLogger {
private:
    std::vector<LocPoint> batch_buffer_;
    const size_t          batch_size_;
    LocPoint              last_batch_point_;

public:
    BatchDeltaLocLogger(size_t batch_size = 10) :
        batch_size_(batch_size) {
        batch_buffer_.reserve(batch_size_);
    }

    ~BatchDeltaLocLogger() {
    }

    void logLocData(const LocPoint &point);

private:
    void flushBuffer();
};
} // namespace traj_logger
} // namespace byd
