#pragma once

#include "pcd.h"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/points/point_cloud.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "small_gicp/util/normal_estimation_omp.hpp"

namespace registration {

std::vector<Eigen::Vector3d> pc_filter(const std::vector<PCD::PCDParser::Point> &cloud, std::function<bool(const PCD::PCDParser::Point &)> filter) {
    std::vector<Eigen::Vector3d> v3cloud;
    v3cloud.reserve(cloud.size());
    for (auto &p : cloud) {
        if (filter(p)) {
            v3cloud.push_back({(double)(p.x), (double)(p.y), (double)(p.z)});
        }
    }
    v3cloud.shrink_to_fit();
    return v3cloud;
}

Eigen::Isometry3d regis(const std::vector<Eigen::Vector3d> &target_points, const std::vector<Eigen::Vector3d> &source_points) {
    using namespace small_gicp;
    int    num_threads                 = 4;
    double downsampling_resolution     = 1.0;
    int    num_neighbors               = 5;
    double max_correspondence_distance = 0.4;

    // Convert to small_gicp::PointCloud
    auto target = std::make_shared<PointCloud>(target_points);
    auto source = std::make_shared<PointCloud>(source_points);

    // Downsampling
    target = voxelgrid_sampling_omp(*target, downsampling_resolution, num_threads);
    source = voxelgrid_sampling_omp(*source, downsampling_resolution, num_threads);

    // Create KdTree
    auto target_tree = std::make_shared<KdTree<PointCloud>>(target, KdTreeBuilderOMP(num_threads));
    auto source_tree = std::make_shared<KdTree<PointCloud>>(source, KdTreeBuilderOMP(num_threads));

    // Estimate point covariances
    estimate_covariances_omp(*target, *target_tree, num_neighbors, num_threads);
    estimate_covariances_omp(*source, *source_tree, num_neighbors, num_threads);

    // GICP + OMP-based parallel reduction
    Registration<GICPFactor, ParallelReductionOMP> registration;
    registration.reduction.num_threads = num_threads;
    registration.rejector.max_dist_sq  = max_correspondence_distance * max_correspondence_distance;

    // Align point clouds
    Eigen::Isometry3d init_T_target_source = Eigen::Isometry3d::Identity();
    auto              result               = registration.align(*target, *source, *target_tree, init_T_target_source);

    Eigen::Isometry3d T = result.T_target_source; // Estimated transformation
    // size_t                      num_inliers = result.num_inliers;     // Number of inlier source points
    // Eigen::Matrix<double, 6, 6> H           = result.H;               // Final Hessian matrix (6x6)
    return T;
}

} // namespace registration
