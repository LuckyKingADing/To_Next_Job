#pragma once

#include <array>
#include <memory>

#include "Iir.h"

namespace MSF {

using LPF = class LOW_PASS_FILTER {
public:
    const static int ORDER = 1;
    const double     samplingrate;
    const double     cutoff_frequency;

private:
    std::shared_ptr<Iir::Butterworth::LowPass<ORDER>> lowpass_ptr = nullptr;

public:
    LOW_PASS_FILTER() : samplingrate(100), cutoff_frequency(5) {
        lowpass_ptr = std::make_shared<Iir::Butterworth::LowPass<ORDER>>();
        lowpass_ptr->setup(samplingrate, cutoff_frequency);
    }
    LOW_PASS_FILTER(double samplingrate_, double cutoff_frequency_)
        : samplingrate(samplingrate_), cutoff_frequency(cutoff_frequency_) {
        lowpass_ptr = std::make_shared<Iir::Butterworth::LowPass<ORDER>>();
        lowpass_ptr->setup(samplingrate, cutoff_frequency);
    }

public:
    double operator()(double x_) { return lowpass_ptr->filter(x_); }
};

template <int N>
class VLPF {
public:
    std::array<std::shared_ptr<LPF>, N> lpfs{nullptr};

    //  public:
    VLPF() {
        for (auto &lpf : lpfs) {
            lpf = std::make_shared<LPF>();
        }
    }
    VLPF(double samplingrate_, double cutoff_frequency_) {
        for (auto &lpf : lpfs) {
            lpf = std::make_shared<LPF>(samplingrate_, cutoff_frequency_);
        }
    }

public:
    std::array<double, N> operator()(const std::array<double, N> &sigsin_) {
        std::array<double, N> sigsout{0};
        for (size_t i = 0; i < N; i++) {
            sigsout[i] = (*(lpfs[i]))(sigsin_[i]);
        }
        return std::move(sigsout);
    }
};
} // namespace MSF