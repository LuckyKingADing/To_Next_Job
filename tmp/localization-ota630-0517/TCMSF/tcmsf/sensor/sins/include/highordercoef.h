#pragma once

#include <Eigen/Dense>
#include <vector>

namespace INS {
namespace highordercoef {

struct COEF {
    size_t                       N;
    Eigen::Matrix<size_t, -1, 2> ij;
    Eigen::Matrix<double, -1, 1> K2;
    Eigen::Matrix<size_t, -1, 3> ijk;
    Eigen::Matrix<double, -1, 1> K3;
    Eigen::Matrix<size_t, -1, 4> ijkl;
    Eigen::Matrix<double, -1, 1> K4;
};

std::vector<COEF> hocoef();

} // namespace highordercoef
} // namespace INS