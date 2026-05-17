#include "csv_player.h"

namespace utils {

CSV_IO::CSV_IO() { csv_ptr = std::make_shared<std::ifstream>(); }

CSV_IO::~CSV_IO() {
    if (csv_ptr->is_open()) {
        csv_ptr->close();
    }
}

bool CSV_IO::open(const std::string filename) {
    csv_ptr->open(filename, std::ios::in);
    if (csv_ptr->is_open()) {
        return true;
    } else {
        return false;
    }
}

bool CSV_IO::splite(std::vector<std::string> &str_v) {
    if (csv_ptr->is_open()) {
        std::string str_;
        std::getline(*csv_ptr, str_);
        std::istringstream iss;
        iss.str(str_);
        std::string key;
        while (std::getline(iss, key, ',')) {
            str_v.push_back(key);
        }
        return true;
    } else {
        return false;
    }
}

bool CSV_IO::skip() {
    if (csv_ptr->is_open()) {
        std::string str_;
        std::getline(*csv_ptr, str_);
        return true;
    } else {
        return false;
    }
}

bool IMU_PARSER::parse(std::shared_ptr<MSF::ImuData> imu_ptr) {
    std::vector<std::string> str_v;
    if (!splite(str_v)) {
        return false;
    }
    try {
        imu_ptr->measurement_timestamp = std::stod(str_v[0]);
        imu_ptr->acc << std::stod(str_v[1]), std::stod(str_v[2]), std::stod(str_v[3]);
        imu_ptr->gyro << std::stod(str_v[4]), std::stod(str_v[5]), std::stod(str_v[6]);
    } catch (...) {
        return false;
    }
    return true;
}

bool GNSS_PARSER::parse(std::shared_ptr<MSF::GnssData> gnss_ptr) {
    std::vector<std::string> str_v;
    if (!splite(str_v)) {
        return false;
    }
    try {
        gnss_ptr->measurement_timestamp = std::stod(str_v[0]);
        gnss_ptr->lla << std::stod(str_v[1]), std::stod(str_v[2]), std::stod(str_v[3]);
        gnss_ptr->vel << std::stod(str_v[4]), std::stod(str_v[5]), std::stod(str_v[6]);
        gnss_ptr->hdg    = std::stod(str_v[7]);
        gnss_ptr->status = std::stoul(str_v[8]);
    } catch (...) {
        return false;
    }
    return true;
}

bool VEHICLE_PARSER::parse(std::shared_ptr<MSF::VehicleData> vehicle_ptr) {
    std::vector<std::string> str_v;
    if (!splite(str_v)) {
        return false;
    }
    try {
        vehicle_ptr->measurement_timestamp = std::stod(str_v[0]);
        vehicle_ptr->speed_rl              = std::stod(str_v[3]);
        vehicle_ptr->speed_rr              = std::stod(str_v[4]);
        vehicle_ptr->yaw_rate              = std::stod(str_v[6]);
    } catch (...) {
        return false;
    }
    return true;
}

PLAYER::PLAYER() {
    imu_parser_ptr     = std::make_shared<IMU_PARSER>();
    gnss_parser_ptr    = std::make_shared<GNSS_PARSER>();
    vehicle_parser_ptr = std::make_shared<VEHICLE_PARSER>();
    imu_data_ptr       = std::make_shared<MSF::ImuData>();
    gnss_data_ptr      = std::make_shared<MSF::GnssData>();
    vehicle_data_ptr   = std::make_shared<MSF::VehicleData>();
}

bool PLAYER::init(const std::string imu_csv, const std::string gnss_csv, const std::string vehicle_csv) {
    if (!imu_parser_ptr->open(imu_csv) || !gnss_parser_ptr->open(gnss_csv) || !vehicle_parser_ptr->open(vehicle_csv)) {
        return false;
    }
    if (!imu_parser_ptr->skip() || !gnss_parser_ptr->skip() || !vehicle_parser_ptr->skip()) {
        return false;
    }
    pos_fix_std    = parameters_sgt.get_gnss_position_bias_std(MSF::Parameters::GNSS_STATUS::FIX);
    pos_float_std  = parameters_sgt.get_gnss_position_bias_std(MSF::Parameters::GNSS_STATUS::FLOAT);
    pos_single_std = parameters_sgt.get_gnss_position_bias_std(MSF::Parameters::GNSS_STATUS::SINGLE);
    vel_fix_std    = parameters_sgt.get_gnss_velocity_bias_std(MSF::Parameters::GNSS_STATUS::FIX);
    vel_float_std  = parameters_sgt.get_gnss_velocity_bias_std(MSF::Parameters::GNSS_STATUS::FLOAT);
    vel_single_std = parameters_sgt.get_gnss_velocity_bias_std(MSF::Parameters::GNSS_STATUS::SINGLE);

    return true;
}

bool PLAYER::play() {
    bool imu_parse_result     = true;
    bool gnss_parse_result    = true;
    bool vehicle_parse_result = true;
    // Eigen::Vector3d gravity              = MSF::UpdateGravity(0);
    imu_parse_result = imu_parser_ptr->parse(imu_data_ptr);

    // imu_data_ptr->acc = imu_data_ptr->acc * std::abs(gravity.z()) * DT_IMU;
    imu_data_ptr->acc  = imu_data_ptr->acc * earth.g.norm() * DT_IMU;
    imu_data_ptr->gyro = imu_data_ptr->gyro / 180.0 * M_PI * DT_IMU;

    while (imu_data_ptr->measurement_timestamp > gnss_data_ptr->measurement_timestamp) {
        gnss_parse_result                    = gnss_parser_ptr->parse(gnss_data_ptr);
        gnss_data_ptr->lla.block<2, 1>(0, 0) = gnss_data_ptr->lla.block<2, 1>(0, 0) / 180.0 * M_PI;
        gnss_data_ptr->hdg                   = gnss_data_ptr->hdg / 180.0 * M_PI - M_PI;
        if (gnss_data_ptr->status == 6) {
            Eigen::Vector3d std(pos_fix_std, pos_fix_std, pos_fix_std * 3.0);
            gnss_data_ptr->lla_cov = std.array().pow(2).matrix();

            std << vel_fix_std, vel_fix_std, vel_fix_std * 3.0;
            gnss_data_ptr->vel_cov = std.array().pow(2).matrix();
        } else if (gnss_data_ptr->status == 5) {
            Eigen::Vector3d std(pos_float_std, pos_float_std, pos_float_std * 3.0);
            gnss_data_ptr->lla_cov = std.array().pow(2).matrix();

            std << vel_float_std, vel_float_std, vel_float_std * 3.0;
            gnss_data_ptr->vel_cov = std.array().pow(2).matrix();
        } else {
            Eigen::Vector3d std(pos_single_std, pos_single_std, pos_single_std * 3.0);
            gnss_data_ptr->lla_cov = std.array().pow(2).matrix();

            std << vel_single_std, vel_single_std, vel_single_std * 3.0;
            gnss_data_ptr->vel_cov = std.array().pow(2).matrix();
        }
        // gravity = MSF::UpdateGravity(gnss_data_ptr->lla.x());
        // earth.update(gnss_data_ptr->lla, {0.0, 0.0, 0.0});
        if (!gnss_parse_result) {
            break;
        }
    }
    while (imu_data_ptr->measurement_timestamp > vehicle_data_ptr->measurement_timestamp) {
        vehicle_parse_result = vehicle_parser_ptr->parse(vehicle_data_ptr);
        if (!vehicle_parse_result) {
            break;
        }
    }
    if (imu_parse_result && gnss_parse_result && vehicle_parse_result) {
        return true;
    } else {
        return false;
    }
}

} // namespace utils