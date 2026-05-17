#pragma once

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <vector>

#include "base_type.h"

#include "earth.h"
#include "tcmsf_config.h"

// #include "utils.h"

namespace utils {

class CSV_IO {
private:
    std::shared_ptr<std::ifstream> csv_ptr;

public:
    bool splite(std::vector<std::string> &);

public:
    CSV_IO();
    ~CSV_IO();
    bool open(const std::string);
    bool skip();
};

class IMU_PARSER : public CSV_IO {
public:
    virtual bool parse(std::shared_ptr<MSF::ImuData>);
};

class GNSS_PARSER : public CSV_IO {
public:
    virtual bool parse(std::shared_ptr<MSF::GnssData>);
};

class VEHICLE_PARSER : public CSV_IO {
public:
    virtual bool parse(std::shared_ptr<MSF::VehicleData>);
};

class PLAYER {
private:
    byd::tcmsf::config::Parameters &parameters_sgt = byd::tcmsf::config::Parameters::getInstance(byd::tcmsf::config::TCMSF_CONFIG_FILE_DIR_);

private:
    const double DT_IMU     = 0.01;
    const double DT_GNSS    = 0.1;
    const double DT_VEHICLE = 0.01;
    INS::EARTH   earth;

private:
    double pos_fix_std;
    double pos_float_std;
    double pos_single_std;
    double vel_fix_std;
    double vel_float_std;
    double vel_single_std;

private:
    std::shared_ptr<IMU_PARSER>     imu_parser_ptr     = nullptr;
    std::shared_ptr<GNSS_PARSER>    gnss_parser_ptr    = nullptr;
    std::shared_ptr<VEHICLE_PARSER> vehicle_parser_ptr = nullptr;

public:
    MSF::ImuDataPtr     imu_data_ptr     = nullptr;
    MSF::GnssDataPtr    gnss_data_ptr    = nullptr;
    MSF::VehicleDataPtr vehicle_data_ptr = nullptr;

public:
    PLAYER();
    bool init(const std::string imu_csv, const std::string gnss_csv, const std::string vehicle_csv);
    bool play();
};

} // namespace utils