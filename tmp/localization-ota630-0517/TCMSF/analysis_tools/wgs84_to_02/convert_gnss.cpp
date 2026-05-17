#include "Coord.h"
#include "fmt/format.h"
#include "tocsv.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// wgtochina_lb(0, lla_antenna_.y() * 180.0 / M_PI, lla_antenna_.x() * 180.0 / M_PI, lla_antenna_.z(), 0, 0, &gnss_antenna_lon_mars, &gnss_antenna_lat_mars);

using fileio::FromCSV;
using fileio::ToCSV;

class Convert : public FromCSV {

    ToCSV tocsv;

public:
    bool convert(const std::string infile, const std::string outfile) {

        fmt::print("infile : {}\n", infile);
        fmt::print("outfile: {}\n", outfile);
        if (!open(infile)) {
            return false;
        }
        if (!tocsv.open(outfile)) {
            return false;
        }
        std::string line;
        uint64_t    line_count = 0;
        while (get_line(line)) {
            line_count++;
            if (line_count<2)
                continue;

            if (line.empty() || line.length() < 200)
                break;
            std::vector<std::string> data;
            std::stringstream        ss(line);
            std::string              tmp;

            while (getline(ss, tmp, ',')) {
                if (!tmp.empty()) { // 跳过空字符串
                    data.push_back(tmp);
                }
            }
            // fmt::print("outfile: {}\n", data[16]);
            // if (std::stod(data[7])*std::stod(data[7])+std::stod(data[6])*std::stod(data[6]) < 1.0 || )
            if (data[16] == "6") continue;

            double wrs2[2];
            wgtochina_lb(0, std::stod(data[3]), std::stod(data[4]), std::stod(data[5]), 0, 0, &wrs2[0], &wrs2[1]);
            std::string outline =
                fmt::format("{},{},{},{:>15.10f},{:>15.10f},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
                            data[0], data[1],data[2], wrs2[1], wrs2[0],data[5], //alt
                            data[6], data[7],data[8], data[9], data[10],data[11],data[12], data[13],
                            data[14], data[15], data[16], data[17],data[18],data[19], data[20]);
            // fmt::print("outfile: {}\n", outline);
            tocsv.line(outline);
        }
        fmt::print("end parse\n");
        return true;
    }
};

int main(int argc, char **argv) {
    if (argc != 3) {
        fmt::print("usage: format wgs84_file_dir mars_file_dir\n");
        return -1;
    }
    Convert convert;
    convert.convert(argv[1], argv[2]);
    return 0;
}