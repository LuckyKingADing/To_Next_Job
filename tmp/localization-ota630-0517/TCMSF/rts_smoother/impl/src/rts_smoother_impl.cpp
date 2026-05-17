#include "rts_smoother_impl.h"
#include "cyber/common/log.h"
#include "fmt/format.h"
#include "rigid_transform.h"
#include "tcmsf_config.h"
#include <filesystem>

namespace RTS {
int RtsSmootherImpl::insert(const ForwardStepInfo &forward_step_info_) {
    switch (rts_type) {
        case FULL: {
            forward_buffer.push_back(forward_step_info_);
        } break;

        case LITE: {
            ForwardStepInfoLite lite_info_;
            lite_info_.timestamp = forward_step_info_.timestamp;
            lite_info_.Pk        = forward_step_info_.Pk.diagonal();
            lite_info_.Pkk1      = forward_step_info_.Pkk1.diagonal();
            lite_info_.Tkk1      = forward_step_info_.Tkk1;
            lite_info_.Xk        = forward_step_info_.Xk;
            lite_info_.Xkk1      = forward_step_info_.Xkk1;
            lite_info_.zupt      = forward_step_info_.zupt;
            lite_info_.m_sta     = forward_step_info_.m_sta;
            lite_info_.sow       = forward_step_info_.sow;
            lite_forward_buffer.push_back(lite_info_);
        } break;
    }
    return 0;
}

int RtsSmootherImpl::backward() {
    auto delta_state_func_ = [](Nx1_RTS_D x, Nx1_RTS_D ref) -> Nx1_RTS_D {
        Nx1_RTS_D          dx       = x - ref;
        Eigen::Quaterniond att_     = INS::euler2quaternion(x.block<3, 1>(0, 0));
        Eigen::Quaterniond att_ref_ = INS::euler2quaternion(ref.block<3, 1>(0, 0));

        Eigen::Quaterniond datt_     = att_ * att_ref_.conjugate();
        Eigen::Vector3d    eulr_tmp_ = INS::quaternion2euler(datt_.conjugate());
        dx.block<3, 1>(0, 0)         = eulr_tmp_;
        return dx;
    };
    auto state_feedback_func_ = [](Nx1_RTS_D x0, Nx1_RTS_D dx) -> Nx1_RTS_D {
        Nx1_RTS_D          x    = x0 + dx;
        Eigen::Quaterniond att_ = INS::euler2quaternion(x0.block<3, 1>(0, 0));
        Eigen::Quaterniond dq_  = INS::euler2quaternion(dx.block<3, 1>(0, 0)).conjugate();
        x.block<3, 1>(0, 0)     = INS::quaternion2euler((dq_ * att_).normalized());
        return x;
    };
    switch (rts_type) {
        case FULL: {
            Nx1_RTS_D Xk = forward_buffer.back().Xk;
            NxN_RTS_D Pk = forward_buffer.back().Pk;

            uint64_t total_size = forward_buffer.size();
            uint64_t count_     = 0;

            for (auto Skp1 = forward_buffer.rbegin(); Skp1 != forward_buffer.rend() - 1; ++Skp1) {
                auto Sk = Skp1 + 1;
                if (Sk->zupt) {
                    // 零速更新状态下，不做RTS更新
                } else {
                    bool Manu_Steady_ = (Sk->m_sta == STEADY || Sk->m_sta == DYNAMIC_LOW);

                    if (large_inno_detect(Xk, Skp1->Xkk1) || !Manu_Steady_) {
                        // 针对行车场景，横向新息大的时候，仅采用P矩阵的对角线元素，避免污染其他状态
                        NxN_RTS_D Sk_Pk_tmp_     = Sk->Pk.diagonal().asDiagonal();
                        NxN_RTS_D Skp1_Pkk1_tmp_ = Skp1->Pkk1.diagonal().asDiagonal();

                        NxN_RTS_D Kk_ = Sk_Pk_tmp_ * Skp1->Tkk1.transpose() * Skp1_Pkk1_tmp_.inverse();
                        Nx1_RTS_D Xk_ = state_feedback_func_(Sk->Xk, Kk_ * delta_state_func_(Xk, Skp1->Xkk1));
                        NxN_RTS_D Pk_ = Sk_Pk_tmp_ + Kk_ * (Pk - Skp1_Pkk1_tmp_) * Kk_.transpose();

                        Xk = Xk_;
                        Pk = Pk_;
                    } else {

                        NxN_RTS_D Sk_Pk_tmp_     = Sk->Pk;
                        NxN_RTS_D Skp1_Pkk1_tmp_ = Skp1->Pkk1;

                        NxN_RTS_D Kk_ = Sk_Pk_tmp_ * Skp1->Tkk1.transpose() * Skp1_Pkk1_tmp_.inverse();
                        Nx1_RTS_D Xk_ = state_feedback_func_(Sk->Xk, Kk_ * delta_state_func_(Xk, Skp1->Xkk1));
                        NxN_RTS_D Pk_ = Sk_Pk_tmp_ + Kk_ * (Pk - Skp1_Pkk1_tmp_) * Kk_.transpose();

                        Xk = Xk_;
                        Pk = Pk_;
                    }
                }
                if (count_ % SAVE_TO_FILE_SKIP == 0) {
                    to_csv(Sk->timestamp, Sk->sow, Xk);
                }
                count_++;
                AINFO_IF((count_ + 3) % (total_size / 10 + 1) == 0)
                    << "--------- RTS backward processing: "
                    << fmt::format("{:.1f} %", static_cast<double>(count_ + 2) / static_cast<double>(total_size) * 100.0);
            }

        } break;

        case LITE: {
            Nx1_RTS_D Xk = lite_forward_buffer.back().Xk;
            NxN_RTS_D Pk = lite_forward_buffer.back().Pk.asDiagonal();

            uint64_t total_size = lite_forward_buffer.size();
            uint64_t count_     = 0;

            for (auto Skp1 = lite_forward_buffer.rbegin(); Skp1 != lite_forward_buffer.rend() - 1; ++Skp1) {
                auto Sk = Skp1 + 1;
                if (Sk->zupt) {
                    // 零速更新状态下，不做RTS更新
                } else {
                    NxN_RTS_D Pk_f   = Sk->Pk.asDiagonal();
                    NxN_RTS_D Pkk1_f = Skp1->Pkk1.asDiagonal();
                    NxN_RTS_D Kk_    = Pk_f * Skp1->Tkk1.transpose() * Pkk1_f.inverse();
                    Nx1_RTS_D Xk_    = state_feedback_func_(Sk->Xk, Kk_ * delta_state_func_(Xk, Skp1->Xkk1));
                    NxN_RTS_D Pk_    = Pk_f + Kk_ * (Pk - Pkk1_f) * Kk_.transpose();

                    Xk = Xk_;
                    Pk = Pk_;
                }
                if (count_ % SAVE_TO_FILE_SKIP == 0) {
                    to_csv(Sk->timestamp, Sk->sow, Xk);
                }
                count_++;
                AINFO_IF((count_ + 3) % (total_size / 10 + 1) == 0)
                    << "--------- RTS backward processing: "
                    << fmt::format("{:.1f} %", static_cast<double>(count_ + 2) / static_cast<double>(total_size) * 100.0);
            }
        } break;
    }

    return 0;
}

bool RtsSmootherImpl::large_inno_detect(const Nx1_RTS_D &pre_, const Nx1_RTS_D &cur_) {

    constexpr static double EGO_LAT_DELTA_POS_BOUND  = 2.0;
    constexpr static double EGO_LON_DELTA_POS_BOUND  = 2.0;
    constexpr static double DELTA_EULR_BOUND         = 2.0 / 180.0 * M_PI;
    constexpr static double DELTA_VEL_BOUND          = 1.0;
    constexpr static double PROPORTIONAL_SPEED_BOUND = 2.0;

    // 做一下位置重置检测
    Eigen::Quaterniond att               = INS::euler2quaternion(pre_.block<3, 1>(0, 0));
    Eigen::Vector3d    delta_pos_rfu     = local_trans.LLAtoEgoRfu(pre_.block<3, 1>(6, 0), cur_.block<3, 1>(6, 0), att);
    bool               is_lat_pos_fault_ = std::fabs(delta_pos_rfu.x()) > EGO_LAT_DELTA_POS_BOUND;
    bool               is_lon_pos_fault_ = std::fabs(delta_pos_rfu.y()) > EGO_LON_DELTA_POS_BOUND;

    // 做一下姿态检测
    Eigen::Vector3d deulr_         = pre_.block<3, 1>(0, 0) - cur_.block<3, 1>(0, 0);
    bool            is_eulr_fault_ = (deulr_.array().cwiseAbs() > DELTA_EULR_BOUND).any();

    // 做一下速度检测
    Eigen::Vector3d dvel_         = pre_.block<3, 1>(3, 0) - cur_.block<3, 1>(3, 0);
    bool            is_vel_fault_ = (dvel_.array().cwiseAbs() > DELTA_VEL_BOUND).any();
    bool            is_spd_fault  = dvel_.norm() > cur_.block<3, 1>(3, 0).norm() * PROPORTIONAL_SPEED_BOUND;

    return is_lat_pos_fault_ || is_lon_pos_fault_ || is_eulr_fault_ || is_vel_fault_ || is_spd_fault;
}

RtsSmootherImpl::RtsSmootherImpl(const std::string &result_out_file_path, RTS_TYPE type_, RTS_FRAME frame_) :
    rts_type(type_), rts_frame(frame_) {
    if (create_file_if_not_exist(result_out_file_path)) {
        out_fs.open(result_out_file_path, std::ios::out);
        AINFO_IF(out_fs.is_open()) << "open rts result file success: " << result_out_file_path;
    } else {
        AWARN << "rts result file open failed: " << result_out_file_path;
    }
}

bool RtsSmootherImpl::create_file_if_not_exist(const std::string &filepath) {
    namespace FS = std::filesystem;
    FS::path cfg_path_{filepath};
    if (FS::exists(cfg_path_) && FS::status(cfg_path_).type() == FS::file_type::regular) {
        return true;
    } else if (!FS::exists(cfg_path_.parent_path())) {
        try {
            if (FS::create_directories(cfg_path_.parent_path())) {
                AINFO << "file parent path not exist, create: " << cfg_path_.parent_path();
                std::fstream cfg_fs;
                cfg_fs.open(filepath, std::ios::out);
                if (cfg_fs.is_open()) {
                    AINFO << "create file: " << filepath;
                    cfg_fs.close();
                    return true;
                }
            }
        } catch (...) {
            return false;
        }
    } else if (FS::exists(cfg_path_.parent_path())) {
        try {
            std::fstream cfg_fs;
            cfg_fs.open(filepath, std::ios::out);
            if (cfg_fs.is_open()) {
                AINFO << "create file: " << filepath;
                cfg_fs.close();
                return true;
            }
        } catch (...) {
            return false;
        }
    }
    return false;
}

int RtsSmootherImpl::to_csv(double timestamp, double sow, const Nx1_RTS_D &Xk) {

    using namespace byd::tcmsf::config;

    Parameters &parameters_sgt = Parameters::getInstance(TCMSF_CONFIG_FILE_DIR_);

    Eigen::Matrix3d MpvAtt = Eigen::Matrix3d::Identity();
    Eigen::Vector3d lever  = Eigen::Vector3d::Zero();

    earth.update(Xk.block<3, 1>(6, 0), Xk.block<3, 1>(3, 0));

    Eigen::Quaterniond att_f_imu2nav = INS::euler2quaternion(Xk.block<3, 1>(0, 0)).conjugate();
    Eigen::Matrix3d    Mpv           = Eigen::Matrix3d::Zero();
    Mpv << 0.0, 1.0 / earth.RMh, 0.0, 1.0 / earth.clRNh, 0.0, 0.0, 0.0, 0.0, 1.0;

    MpvAtt = Mpv * att_f_imu2nav.toRotationMatrix();

    switch (rts_frame) {
        case F_IMU:
            break;

        case F_ANN: {
            lever = parameters_sgt.get_lever_imu2gnss();
        } break;

        case F_VEH: {
            lever = parameters_sgt.get_lever_imu2vehicle();
        } break;

        default:
            break;
    }
    Eigen::Vector3d eulr_       = INS::quaternion2euler(att_f_imu2nav) * 180.0 / M_PI;
    Eigen::Vector3d lever_comp_ = MpvAtt * lever;
    std::string     str =
        fmt::format("{:14.4f},"                               // 1  时间戳
                    "{:7.4f},"                                // 2  俯仰
                    "{:7.4f},"                                // 3  横滚
                    "{:7.4f},"                                // 4  偏航
                    "{:7.4f},"                                // 5  东向速度
                    "{:7.4f},"                                // 6  北向速度
                    "{:7.4f},"                                // 7  天向速度
                    "{:14.10f},"                              // 8  经度
                    "{:14.10f},"                              // 9  纬度
                    "{:7.4f},"                                // 10 高度
                    "{:7.4f}\n",                              // 11 周内秒
                    timestamp,                                // 1
                    eulr_[0],                                 // 2
                    eulr_[1],                                 // 3
                    eulr_[2],                                 // 4
                    Xk[3],                                    // 5
                    Xk[4],                                    // 6
                    Xk[5],                                    // 7
                    (Xk[6] + lever_comp_.x()) * 180.0 / M_PI, // 8
                    (Xk[7] + lever_comp_.y()) * 180.0 / M_PI, // 9
                    Xk[8] + lever_comp_.z(),                  // 10
                    sow);                                     // 11

    if (out_fs.is_open()) {
        out_fs << str;
        return 0;
    } else {
        return -1;
    }
}

RtsSmootherImpl::~RtsSmootherImpl() {
    backward();
    if (out_fs.is_open()) {
        out_fs.close();
    }
}

} // namespace RTS