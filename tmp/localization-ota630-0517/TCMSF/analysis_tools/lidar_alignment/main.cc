#include "kinematic_compensation.h"
#include "modules/msg/localization_msgs/localization_info.pb.h"
#include "parser.h"
#include "pcd.h"
#include "registration.h"
#include <iomanip>

#include <Eigen/Geometry>

#define DR_TPC "/localization/dr"

using namespace analysis;
using namespace registration;
using byd::msg::localization::LocalizationEstimate;

int main(int argc, char **argv) {
    if (argc != 4) {
        std::cout << "usage: PARSER record_dir pcd_file_1 pcd_file_2\n";
        return -1;
    }
    KinematicCompensation                 kc;
    Parser                                parser;
    std::shared_ptr<LocalizationEstimate> dr_msg_ = std::make_shared<LocalizationEstimate>();

    auto cb_ = [&kc, &dr_msg_](apollo::cyber::record::RecordMessage &msg_) {
        if (msg_.channel_name == DR_TPC) {
            dr_msg_->ParseFromString(msg_.content);
            KinematicData kin_data;
            // 输入为DR坐标系，即FLU
            if (dr_msg_->has_header() &&
                dr_msg_->header().has_measurement_timestamp() &&
                dr_msg_->header().has_publish_timestamp() &&
                dr_msg_->header().has_sequence_num() &&
                dr_msg_->has_pose() &&
                // dr_msg_->pose().has_linear_velocity() &&
                // dr_msg_->pose().linear_velocity().has_x() &&
                // dr_msg_->pose().linear_velocity().has_y() &&
                // dr_msg_->pose().linear_velocity().has_z() &&
                dr_msg_->pose().has_orientation() &&
                dr_msg_->pose().orientation().has_qw() &&
                dr_msg_->pose().orientation().has_qx() &&
                dr_msg_->pose().orientation().has_qy() &&
                dr_msg_->pose().orientation().has_qz() &&
                dr_msg_->pose().has_position() &&
                dr_msg_->pose().position().has_x() &&
                dr_msg_->pose().position().has_y() &&
                dr_msg_->pose().position().has_z()) {
                kin_data.measurement_timestamp = dr_msg_->header().measurement_timestamp();
                kin_data.att.w()               = dr_msg_->pose().orientation().qw();
                kin_data.att.x()               = dr_msg_->pose().orientation().qx();
                kin_data.att.y()               = dr_msg_->pose().orientation().qy();
                kin_data.att.z()               = dr_msg_->pose().orientation().qz();
                kin_data.pos.x()               = dr_msg_->pose().position().x();
                kin_data.pos.y()               = dr_msg_->pose().position().y();
                kin_data.pos.z()               = dr_msg_->pose().position().z();
                // kin_data.vel.x()               = dr_msg_->pose().linear_velocity().x();
                // kin_data.vel.y()               = dr_msg_->pose().linear_velocity().y();
                // kin_data.vel.z()               = dr_msg_->pose().linear_velocity().z();

                kc.insert(std::move(kin_data));
            }
        }
    };

    parser.register_msg_cb_func(cb_);
    parser.parse(argv[1]);

    PCD::PCDParser pcd1, pcd2;
    pcd1.parse(argv[2]);
    pcd2.parse(argv[3]);
    double min_timestamp_1 = 1e15, max_timestamp_1 = 0.0;
    for (auto &p : pcd1.cloud) {
        if (p.timestamp > max_timestamp_1) {
            max_timestamp_1 = p.timestamp;
        }
        if (p.timestamp < min_timestamp_1) {
            min_timestamp_1 = p.timestamp;
        }
    }
    double min_timestamp_2 = 1e15, max_timestamp_2 = 0.0;
    for (auto &p : pcd2.cloud) {
        if (p.timestamp > max_timestamp_2) {
            max_timestamp_2 = p.timestamp;
        }
        if (p.timestamp < min_timestamp_2) {
            min_timestamp_2 = p.timestamp;
        }
    }
    double min_t = std::min(min_timestamp_1, min_timestamp_2);
    double max_t = std::max(max_timestamp_1, max_timestamp_2);

    std::cout << std::setprecision(14) << "min t: " << min_t << " max_t: " << max_t << "\n";

    kc.crop_by_timestamp(min_t, max_t);

    std::vector<PCD::PCDParser::Point> PCD_tmp_1, PCD_tmp_2, PCD_tmp_1_regis;
    PCD_tmp_1.reserve(pcd1.header.point_size);
    PCD_tmp_2.reserve(pcd2.header.point_size);
    PCD_tmp_1_regis.reserve(pcd1.header.point_size);

    Eigen::Matrix3d FLU2RFU;
    FLU2RFU << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix3d RFU2FLU = FLU2RFU.transpose();

    Eigen::Matrix4d T;
    T << -0.0048688610274749125,
        0.9998912334215097,
        -0.013921683422571657,
        -1.696198363742224,
        -0.999985963167825,
        -0.004897370106752336,
        -0.0020144823561598457,
        -0.010974677673149117,
        -0.002082442847698026,
        0.013911679909650688,
        0.9999010531899115,
        -1.1233389549042665,
        0.0,
        0.0,
        0.0,
        1.0;

    Eigen::Matrix3d R = T.block<3, 3>(0, 0).transpose();
    // Eigen::Vector3d eulr_ = R.eulerAngles(2, 1, 0);
    Eigen::AngleAxisd axis_angle(R);
    R = Eigen::AngleAxisd(axis_angle.angle(), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Vector3d M = -T.block<3, 1>(0, 3);

    auto ptrans = [&R, &M, &RFU2FLU](const PCD::PCDParser::Point &p) -> Eigen::Vector3d {
        Eigen::Vector3d p_{p.x, p.y, p.z};
        return RFU2FLU * R * p_ + M;
    };

    for (auto &p : pcd1.cloud) {
        PCD::PCDParser::Point p_    = p;
        auto                  dp    = kc.delta(min_t, p.timestamp, true).second;
        Eigen::Matrix3d       dmat  = dp.att.toRotationMatrix();
        Eigen::Vector3d       dpos  = dp.pos;
        Eigen::Vector3d       p_tmp = dmat * ptrans(p) + dpos;
        p_.x                        = p_tmp.x();
        p_.y                        = p_tmp.y();
        p_.z                        = p_tmp.z();
        PCD_tmp_1.push_back(p_);
    }

    for (auto &p : pcd2.cloud) {
        PCD::PCDParser::Point p_    = p;
        auto                  dp    = kc.delta(min_t, p.timestamp, true).second;
        Eigen::Matrix3d       dmat  = dp.att.toRotationMatrix();
        Eigen::Vector3d       dpos  = dp.pos;
        Eigen::Vector3d       p_tmp = dmat * ptrans(p) + dpos;
        p_.x                        = p_tmp.x();
        p_.y                        = p_tmp.y();
        p_.z                        = p_tmp.z();
        PCD_tmp_2.push_back(p_);
    }

    auto filter = [](const PCD::PCDParser::Point &p_) -> bool {
        Eigen::Vector3d p{(double)(p_.x), (double)(p_.y), (double)(p_.z)};
        if (p.norm() < 100.0 && p.z() > 0.5 && p.z() < 1.5) {
            return true;
        } else {
            return false;
        }
    };
    auto result = regis(pc_filter(PCD_tmp_2, filter), pc_filter(PCD_tmp_1, filter));

    Eigen::Vector3d eulr = result.rotation().eulerAngles(0, 1, 2);

    AINFO << "point cloud registration:\n"
          << "translation: " << result.translation().x() << ", " << result.translation().y() << ", " << result.translation().z()
          << "\n   rotation: " << eulr.x() * 180 / M_PI << ", " << eulr.y() * 180 / M_PI << ", " << eulr.z() * 180 / M_PI;

    for (auto &p : PCD_tmp_1) {
        PCD::PCDParser::Point p_;
        Eigen::Matrix3d       dmat  = result.rotation();
        Eigen::Vector3d       dpos  = result.translation();
        Eigen::Vector3d       p_tmp = dmat * Eigen::Vector3d{p.x, p.y, p.z} + dpos;
        p_.x                        = p_tmp.x();
        p_.y                        = p_tmp.y();
        p_.z                        = p_tmp.z();
        PCD_tmp_1_regis.push_back(p_);
    }

    pcd1.toPcd("modules/localization/src/TCMSF/analysis_tools/lidar_alignment/data/result/pcd_1_regis.pcd", PCD_tmp_1_regis);
    pcd1.toPcd("modules/localization/src/TCMSF/analysis_tools/lidar_alignment/data/result/pcd_1.pcd", PCD_tmp_1);
    pcd2.toPcd("modules/localization/src/TCMSF/analysis_tools/lidar_alignment/data/result/pcd_2.pcd", PCD_tmp_2);

    return 0;
}