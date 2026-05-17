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
        while (get_line(line)) {
            if (line.empty())
                break;
            std::vector<double> data;
            std::stringstream   ss(line);
            std::string         tmp;
            while (getline(ss, tmp, ' ')) {
                data.push_back(strtod(tmp.c_str(), NULL));
            }
            std::vector<double> data_out = data;
            wgtochina_lb(0, data[2], data[1], data[3], 0, 0, &data_out[2], &data_out[1]);
            std::string outline =
                fmt::format("{:>14.5f},{:>14.9f},{:>14.9f},{:>14.9f},{:>14.9f},{:>14.9f},{:>14.9f},{:>14.9f},{:>14.9f},{:>14.9f},{:>14.9f},{:>14.9f},{:>14.9f}\n",
                            data_out[0],  // 1
                            data_out[1],  // 2
                            data_out[2],  // 3
                            data_out[3],  // 4
                            data_out[4],  // 5
                            data_out[5],  // 6
                            data_out[6],  // 7
                            data_out[7],  // 8
                            data_out[8],  // 9
                            data_out[9],  // 10
                            data_out[10], // 11
                            data_out[11], // 12
                            data_out[12]  // 13
                );
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