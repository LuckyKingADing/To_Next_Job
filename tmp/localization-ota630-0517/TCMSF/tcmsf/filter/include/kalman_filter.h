#pragma once

#include "Eigen/Eigen"
#include "fmt/format.h"
#include <memory>

namespace MSF {

template <int M, int N>
class KF {
public:
    KF() {
        es  = Mx1::Zero();
        P   = MxM::Identity();
        cov = MxM::Identity();
        dx  = Mx1::Zero();
        K   = MxN::Zero();
    }

public:
    using NxM = Eigen::Matrix<double, N, M>;
    using MxN = Eigen::Matrix<double, M, N>;
    using Mx1 = Eigen::Matrix<double, M, 1>;
    using Nx1 = Eigen::Matrix<double, N, 1>;
    using MxM = Eigen::Matrix<double, M, M>;
    using NxN = Eigen::Matrix<double, N, N>;

public:
    Mx1 es; // error states
    MxM P;
    MxM cov;
    Mx1 dx;
    MxN K;

public:
    void kf_update(const NxM &H, const NxN &R, const Nx1 &inno) {
        K        = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        MxM I_KH = MxM::Identity() - K * H;
        cov      = I_KH * P * I_KH.transpose() + K * R * K.transpose();
        dx       = es + K * inno;
    }
    void kf_update(const MxM &T, const MxM &Q) {
        cov = T * P * T.transpose() + Q;
        dx  = T * es;
    }
    // Mx1 get_error_states() {
    //     return es;
    // }
    // MxM get_P() {
    //     return P;
    // }
    // MxM get_cov() {
    //     return cov;
    // }
    // Mx1 get_delta_x() {
    //     return dx;
    // }
    // void set_error_states(const Mx1 &es_) {
    //     es = es_;
    // }
    // void set_P(const MxM &P_) {
    //     P = P_;
    // }
    // void set_cov(const MxM &cov_) {
    //     cov = cov_;
    // }
    // void set_delta_x(const Mx1 &dx_) {
    //     dx = dx_;
    // }
    void measurement_update_debug_info() {
        std::string P0_str = "";
        std::string P1_str = "";
        std::string dP_str = "";
        for (size_t i = 0; i < P.rows(); i++) {
            for (size_t j = 0; j < P.cols(); j++) {
                P0_str += fmt::format("{: >9.1}", P(i, j));
                P1_str += fmt::format("{: >9.1}", cov(i, j));
                dP_str += fmt::format("{: >9.1}", P(i, j) - cov(i, j));
            }
            P0_str += "\n";
            P1_str += "\n";
            dP_str += "\n";
        }
        fmt::print("delta P:\n{}\n", dP_str);
    }

public:
    std::string dump_();
    std::string dump_diag_();
};

template <int M, int N>
std::string KF<M, N>::dump_() {
    std::string p_str   = "";
    std::string cov_str = "";
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < M; j++) {
            p_str += fmt::format("{: >8.3} ", P(i, j));
            cov_str += fmt::format("{: >8.3} ", cov(i, j));
        }
        p_str += "\n";
        cov_str += "\n";
    }
    return "P\n" + p_str + "cov\n" + cov_str;
};

template <int M, int N>
std::string KF<M, N>::dump_diag_() {
    std::string p_str   = "";
    std::string cov_str = "";
    for (int i = 0; i < M; i++) {
        p_str += fmt::format("{: >8.3} ", P(i, i));
        cov_str += fmt::format("{: >8.3} ", cov(i, i));
    }
    p_str += "\n";
    cov_str += "\n";
    return "P\n" + p_str + "cov\n" + cov_str;
};

template <int M, int N>
using KfPtr = std::shared_ptr<MSF::KF<M, N>>;
} // namespace MSF
