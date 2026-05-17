#pragma once

#include "earth.h"
#include "local_trans.h"
#include "rts_smoother_interface.h"
#include <deque>
#include <fstream>

namespace RTS {

class RtsSmootherImpl : public RtsSmoother {

public:
    // 两种模式：
    // 1. 使用全量的P矩阵信息进行RTS平滑（FULL）
    // 2. 使用P矩阵对角线元素进行RTS平滑（LITE）
    enum RTS_TYPE {
        FULL = 0,
        LITE = 1
    };

    // 输出点位置
    enum RTS_FRAME {
        F_IMU = 0,
        F_VEH = 1,
        F_ANN = 2,
    };

private:
    const RTS_TYPE  rts_type;
    const RTS_FRAME rts_frame;

private:
    INS::EARTH earth;

public:
    virtual int insert(const ForwardStepInfo &) override final;
    virtual int backward() override final;

private:
    bool large_inno_detect(const Nx1_RTS_D &pre_, const Nx1_RTS_D &cur_);

private:
    byd::geo::LocalTrans local_trans;

public:
    RtsSmootherImpl(const std::string &result_out_file_path, RTS_TYPE type_, RTS_FRAME frame_);
    ~RtsSmootherImpl();

private:
    constexpr static int64_t SAVE_TO_FILE_SKIP = 10;

    std::ofstream out_fs;

    bool create_file_if_not_exist(const std::string &filepath);
    int  to_csv(double timestamp, double sow, const Nx1_RTS_D &Xk);

private:
    std::deque<ForwardStepInfo>     forward_buffer;
    std::deque<ForwardStepInfoLite> lite_forward_buffer;
};

} // namespace RTS