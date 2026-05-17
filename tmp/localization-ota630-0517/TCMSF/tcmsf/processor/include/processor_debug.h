#pragma once

#include "tocsv.h"
#include <filesystem>

// 这里做个额外的宏保护，只有在x86平台下，才可能启用DEBUG模式
// 如果是ARM平台，则不会启用
// 实车ARM平台不可以随便写文件，如果启用DEBUG模式的话，大概率会直接崩溃
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
// x86-64 或者 x86-32架构

// 开启调试，保存中间状态信息
#define __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES

#endif

namespace debug {
namespace FS = std::filesystem;
class Debug2CSV {
public:
    static Debug2CSV &getInstance(const std::string &debug_info_dir_path) {
        static Debug2CSV inst(debug_info_dir_path);
        return inst;
    }

    Debug2CSV(const Debug2CSV &)            = delete;
    Debug2CSV &operator=(const Debug2CSV &) = delete;

public:
    fileio::ToCSV cmp_state, veh_std, kcp_state, vis_std;
    fileio::ToCSV imu_state, veh_state, msf_state, gps_state, gps_std, alg_state, zup_state, slp_state, vis_state, lpc_state;
    fileio::ToCSV gps_update;
    fileio::ToCSV bias_esti_state;
    fileio::ToCSV inner_dr_state;
    fileio::ToCSV init_state;
    fileio::ToCSV cx_vali_state;
    fileio::ToCSV spp_vali_state;
    fileio::ToCSV pos_inno_range_norm;
    fileio::ToCSV sdmap_state;
    fileio::ToCSV db_state;
    fileio::ToCSV gnss_consistency_state; // 卫星信息自洽性验证状态

private:
    bool debug_files_created = false;
    Debug2CSV(const std::string &debug_info_dir_path) {
        if (!debug_files_created) {

            auto output_dir = FS::path(debug_info_dir_path);

            FS::path veh_state_fp(output_dir / "veh_debug_state.csv");
            FS::path msf_state_fp(output_dir / "msf_debug_state.csv");
            FS::path alg_state_fp(output_dir / "alg_debug_state.csv");
            FS::path zup_state_fp(output_dir / "zup_debug_state.csv");
            FS::path slp_state_fp(output_dir / "slp_debug_state.csv");
            FS::path gps_state_fp(output_dir / "gps_debug_state.csv");
            FS::path vis_state_fp(output_dir / "vis_debug_state.csv");
            FS::path lpc_state_fp(output_dir / "lpc_debug_state.csv");
            FS::path gps_std_fp(output_dir / "gps_std.csv");

            FS::path imu_state_fp(output_dir / "imu_debug_state.csv");
            FS::path cmp_state_fp(output_dir / "cmp_debug_state.csv");
            FS::path kcp_state_fp(output_dir / "kcp_debug_state.csv");
            FS::path veh_std_fp(output_dir / "veh_std.csv");
            FS::path vis_std_fp(output_dir / "vis_std.csv");

            FS::path gps_update_fp(output_dir / "gps_update.csv");

            FS::path bias_esti_state_fp(output_dir / "bias_esti_state.csv");

            FS::path inner_dr_state_fp(output_dir / "inner_dr_state.csv");

            FS::path init_state_fp(output_dir / "init_debug_state.csv");

            FS::path cx_vali_state_fp(output_dir / "cx_vali_state.csv");

            FS::path spp_vali_state_fp(output_dir / "spp_vali_state.csv");

            FS::path pos_inno_range_norm_fp(output_dir / "pos_inno_range_norm.csv");

            FS::path sdmap_state_fp(output_dir / "sdmap_state.csv");

            FS::path db_state_fp(output_dir / "db_debug_state.csv");

            FS::path gnss_consistency_state_fp(output_dir / "gnss_consistency_state.csv");

            if (veh_state.open(veh_state_fp.string()) &&
                msf_state.open(msf_state_fp.string()) &&
                alg_state.open(alg_state_fp.string()) &&
                zup_state.open(zup_state_fp.string()) &&
                slp_state.open(slp_state_fp.string()) &&
                gps_state.open(gps_state_fp.string()) &&
                vis_state.open(vis_state_fp.string()) &&
                lpc_state.open(lpc_state_fp.string()) &&
                gps_std.open(gps_std_fp.string()) &&
                imu_state.open(imu_state_fp.string()) &&
                cmp_state.open(cmp_state_fp.string()) &&
                kcp_state.open(kcp_state_fp.string()) &&
                veh_std.open(veh_std_fp.string()) &&
                vis_std.open(vis_std_fp.string()) &&
                gps_update.open(gps_update_fp.string()) &&
                bias_esti_state.open(bias_esti_state_fp.string()) &&
                inner_dr_state.open(inner_dr_state_fp.string()) &&
                init_state.open(init_state_fp.string()) &&
                cx_vali_state.open(cx_vali_state_fp.string()) &&
                spp_vali_state.open(spp_vali_state_fp.string()) &&
                pos_inno_range_norm.open(pos_inno_range_norm_fp.string()) &&
                sdmap_state.open(sdmap_state_fp.string()) &&
                db_state.open(db_state_fp.string()) &&
                gnss_consistency_state.open(gnss_consistency_state_fp.string())) {
                debug_files_created = true;
            }
        }
    }
};

#ifdef __TCMSF_DEBUG__SAVE_INFO_TO_CSV_FILES
static Debug2CSV &debug_sgt = Debug2CSV::getInstance("data/tmp/");
#endif

} // namespace debug