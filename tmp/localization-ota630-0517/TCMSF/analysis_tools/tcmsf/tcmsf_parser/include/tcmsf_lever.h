#include "Eigen/Dense"
#include "nlohmann/json.hpp"
#include <fstream>
#include <string>

namespace VehicleInfo {

class Lever {
public:
    Eigen::Vector3d antenna = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu     = Eigen::Vector3d::Zero();

    Lever() = default;
    Lever(Eigen::Vector3d antenna_, Eigen::Vector3d imu_) {
        antenna = antenna_;
        imu     = imu_;
    }

    // 从JSON配置文件解析杆臂
    // 配置文件格式：{"ante2RearAxle": [x,y,z], "imu2RearAxle": [x,y,z]}
    // ante2RearAxle: 天线到后轴中心的杆臂（FLU系）
    // imu2RearAxle: IMU到后轴中心的杆臂（FLU系）
    static Lever fromJson(const std::string &json_path) {
        Lever lever;
        std::ifstream ifs(json_path);
        if (!ifs.is_open()) {
            std::cerr << "Failed to open lever config: " << json_path << std::endl;
            return lever;
        }

        nlohmann::json j;
        try {
            ifs >> j;
        } catch (const nlohmann::json::parse_error &e) {
            std::cerr << "Failed to parse lever config: " << json_path << ": " << e.what() << std::endl;
            return lever;
        }

        if (j.contains("ante2RearAxle") && j["ante2RearAxle"].is_array() && j["ante2RearAxle"].size() == 3) {
            lever.antenna << j["ante2RearAxle"][0], j["ante2RearAxle"][1], j["ante2RearAxle"][2];
        }
        if (j.contains("imu2RearAxle") && j["imu2RearAxle"].is_array() && j["imu2RearAxle"].size() == 3) {
            lever.imu << j["imu2RearAxle"][0], j["imu2RearAxle"][1], j["imu2RearAxle"][2];
        }

        return lever;
    }
};

} // namespace VehicleInfo