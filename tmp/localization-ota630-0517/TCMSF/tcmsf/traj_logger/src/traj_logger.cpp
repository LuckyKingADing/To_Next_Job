#include "traj_logger.h"

namespace byd {
namespace traj_logger {
void BatchDeltaLocLogger::logLocData(const LocPoint &point) {
    batch_buffer_.push_back(point);

    if (batch_buffer_.size() >= batch_size_) {
        flushBuffer();
    }
}
void BatchDeltaLocLogger::flushBuffer() {
    if (batch_buffer_.empty())
        return;

    // 记录批次起始点（绝对值）
    const auto &first_point            = batch_buffer_.front();
    std::string fusion_str             = "[Fusion] ";
    std::string fusion_str_lat         = " <dlat_f> ";
    std::string fusion_str_lon         = " <dlon_f> ";
    std::string fusion_mapbias_str     = "[Fus_mb] ";
    std::string fusion_mapbias_str_lat = " <dlat_f_mb> ";
    std::string fusion_mapbias_str_lon = " <dlon_f_mb> ";
    std::string rtk_str                = "[RTK] ";
    std::string rtk_str_lat            = " <dlat_r> ";
    std::string rtk_str_lon            = " <dlon_r> ";
    std::string sd_str                 = "[Sd_proj] ";
    std::string sd_str_lat             = " <dlat_s> ";
    std::string sd_str_lon             = " <dlon_s> ";
    std::string sd_db_mid_str          = "[Sd_db_mid] ";
    std::string sd_db_mid_str_lat      = " <dlat_s_db> ";
    std::string sd_db_mid_str_lon      = " <dlon_s_db> ";
    std::string rtk_status_str         = "[rtk_s] ";
    std::string align_type_str         = "[align] ";

    // 记录批次内的增量数据
    LocPoint last_point = first_point;

    // 记录首个点
    fusion_str += fmt::format("{:>14.10f},{:>14.10f},{:>14.4f}", first_point.latitude_fusion, first_point.longitude_fusion, first_point.timestamp_fusion);
    fusion_mapbias_str += fmt::format("{:>14.10f},{:>14.10f}", first_point.latitude_fusion_map_b, first_point.longitude_fusion_map_b);
    rtk_str += fmt::format("{:>14.10f},{:>14.10f}", first_point.latitude_rtk, first_point.longitude_rtk);
    sd_str += fmt::format("{:>14.10f},{:>14.10f}", first_point.latitude_sdmap_proj, first_point.longitude_sdmap_proj);
    sd_db_mid_str += fmt::format("{:>14.10f},{:>14.10f}", first_point.latitude_sdmap_proj_db_mid, first_point.longitude_sdmap_proj_db_mid);
    rtk_status_str += fmt::format("{:d},", first_point.rtk_status);
    align_type_str += fmt::format("{:d},", first_point.align_type);

    // 为了节约空间，使用增量记录坐标点位置
    for (size_t i = 1; i < batch_buffer_.size(); ++i) {
        const auto &current_point = batch_buffer_[i];

        // 八位有效数字取整
        // 基本能够保证毫米级精度
        // 10个数累积也能够保证厘米级别精度
        int64_t lat_delta_f    = static_cast<int64_t>((current_point.latitude_fusion - last_point.latitude_fusion) * DELTA_SCALE);
        int64_t lon_delta_f    = static_cast<int64_t>((current_point.longitude_fusion - last_point.longitude_fusion) * DELTA_SCALE);
        int64_t lat_delta_f_mb = static_cast<int64_t>((current_point.latitude_fusion_map_b - last_point.latitude_fusion_map_b) * DELTA_SCALE);
        int64_t lon_delta_f_mb = static_cast<int64_t>((current_point.longitude_fusion_map_b - last_point.longitude_fusion_map_b) * DELTA_SCALE);
        int64_t lat_delta_g    = static_cast<int64_t>((current_point.latitude_rtk - last_point.latitude_rtk) * DELTA_SCALE);
        int64_t lon_delta_g    = static_cast<int64_t>((current_point.longitude_rtk - last_point.longitude_rtk) * DELTA_SCALE);
        int64_t lat_delta_s    = static_cast<int64_t>((current_point.latitude_sdmap_proj - last_point.latitude_sdmap_proj) * DELTA_SCALE);
        int64_t lon_delta_s    = static_cast<int64_t>((current_point.longitude_sdmap_proj - last_point.longitude_sdmap_proj) * DELTA_SCALE);
        int64_t lat_delta_s_db = static_cast<int64_t>((current_point.latitude_sdmap_proj_db_mid - last_point.latitude_sdmap_proj_db_mid) * DELTA_SCALE);
        int64_t lon_delta_s_db = static_cast<int64_t>((current_point.longitude_sdmap_proj_db_mid - last_point.longitude_sdmap_proj_db_mid) * DELTA_SCALE);

        fusion_str_lat += fmt::format("{:d},", lat_delta_f);
        fusion_str_lon += fmt::format("{:d},", lon_delta_f);
        fusion_mapbias_str_lat += fmt::format("{:d},", lat_delta_f_mb);
        fusion_mapbias_str_lon += fmt::format("{:d},", lon_delta_f_mb);
        rtk_str_lat += fmt::format("{:d},", lat_delta_g);
        rtk_str_lon += fmt::format("{:d},", lon_delta_g);
        sd_str_lat += fmt::format("{:d},", lat_delta_s);
        sd_str_lon += fmt::format("{:d},", lon_delta_s);
        sd_db_mid_str_lat += fmt::format("{:d},", lat_delta_s_db);
        sd_db_mid_str_lon += fmt::format("{:d},", lon_delta_s_db);
        rtk_status_str += fmt::format("{:d},", current_point.rtk_status);
        align_type_str += fmt::format("{:d},", current_point.align_type);

        last_point = current_point;
    }
    batch_buffer_.clear();

    fusion_str_lat.pop_back();
    fusion_str_lon.pop_back();
    rtk_str_lat.pop_back();
    rtk_str_lon.pop_back();
    sd_db_mid_str_lat.pop_back();
    sd_db_mid_str_lon.pop_back();
    rtk_status_str.pop_back();
    align_type_str.pop_back();

    AINFO << '\n'
          << fusion_str << '\n'
          << fusion_str_lat << '\n'
          << fusion_str_lon << '\n'
          << fusion_mapbias_str << '\n'
          << fusion_mapbias_str_lat << '\n'
          << fusion_mapbias_str_lon << '\n'
          << rtk_str << '\n'
          << rtk_str_lat << '\n'
          << rtk_str_lon << '\n'
          << sd_str << '\n'
          << sd_str_lat << '\n'
          << sd_str_lon << '\n'
          << sd_db_mid_str << '\n'
          << sd_db_mid_str_lat << '\n'
          << sd_db_mid_str_lon << '\n'
          << rtk_status_str << '\n'
          << align_type_str;
}
} // namespace traj_logger
} // namespace byd