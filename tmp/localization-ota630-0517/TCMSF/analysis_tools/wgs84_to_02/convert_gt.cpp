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

            double wrs2[2];
            double dt0 = 1767580031.0 - 95249.000;//0
            double sow = std::stod(data[2]) - dt0;
            wgtochina_lb(0, std::stod(data[0]), std::stod(data[1]), 0, 0, 0, &wrs2[0], &wrs2[1]);
            std::string outline =
                fmt::format("{:>9.2f},{},{:>15.10f},{:>15.10f},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",sow,"0",
                            wrs2[1], wrs2[0],"0","0","0","0","0","0","0","0","0","0","0","0","0","0");
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