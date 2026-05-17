#pragma once

#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace PCD {

class PCDParser {
public:
// 点结构（支持扩展字段）
#pragma pack(push, 1)
    struct Point {
        float    x, y, z;
        uint32_t ring;
        double   timestamp;
        uint32_t intensity;
    };
#pragma pack(pop)

    // 头部元数据
    struct PCDHeader {
        std::map<std::string, std::string> metadata;
        size_t                             point_size;   // 单点总字节数
        size_t                             points_count; // 点数
        bool                               is_ascii;     // 是否为ASCII格式
    };

private:
    std::string HeaderRaw = "";

public:
    PCDHeader header;

private:
    PCDHeader parseHeader(std::ifstream &file) {
        PCDHeader   header;
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty())
                continue;

            HeaderRaw.append(line + "\n");

            // 提取键值对
            size_t pos = line.find(' ');
            if (pos == std::string::npos)
                continue;

            std::string key      = line.substr(0, pos);
            std::string value    = line.substr(pos + 1);
            header.metadata[key] = value;

            // 关键字段处理
            if (key == "POINTS")
                header.points_count = std::stoul(value);
            else if (key == "DATA")
                header.is_ascii = (value == "ascii");

            // 计算单点字节数
            if (key == "SIZE") {
                size_t             total_size = 0;
                std::istringstream ss(value);
                for (int size; ss >> size; total_size += size) {
                }

                header.point_size = total_size;
            }
            if (line.find("DATA") != std::string::npos)
                break; // 结束头部
        }
        return header;
    }

    std::vector<Point> parseASCII(std::ifstream &file, const PCDHeader &header) {
        std::vector<Point> cloud;
        std::string        line;
        for (size_t i = 0; i < header.points_count; ++i) {
            std::getline(file, line);
            std::istringstream iss(line);
            Point              p;
            iss >> p.x >> p.y >> p.z >> p.ring >> p.timestamp >> p.intensity;
            cloud.push_back(p);
        }
        return cloud;
    }

    std::vector<Point> parseBinary(std::ifstream &file, const PCDHeader &header) {
        std::vector<Point> cloud(header.points_count);
        file.read(reinterpret_cast<char *>(cloud.data()), header.points_count * header.point_size);
        return cloud;
    }

public:
    bool parse(const std::string &pcd_file) {
        std::ifstream file(pcd_file, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Error: Failed to open file!" << std::endl;
            return false;
        }

        // 解析流程
        header = parseHeader(file);

        if (header.is_ascii)
            cloud = parseASCII(file, header);
        else
            cloud = parseBinary(file, header);

        // 示例：打印前5个点
        std::cout << "x,y,z,ring,timestamp,intensity\n";
        for (int i = 0; i < 5; ++i)
            std::cout << cloud[i].x << ", "
                      << cloud[i].y << ", "
                      << cloud[i].z << ", "
                      << cloud[i].ring << ", "
                      << cloud[i].timestamp << ", "
                      << cloud[i].intensity << "\n";

        return true;
    }

    void toPcd(const std::string &out_dir) {
        std::ofstream out(out_dir, std::ios::binary);
        out << HeaderRaw;
        out.write(reinterpret_cast<const char *>(cloud.data()), cloud.size() * header.point_size);
        std::cout << "data size: " << cloud.size() << " point size: " << header.point_size << std::endl;
    }
    void toPcd(const std::string &out_dir, const std::vector<Point> &cld_) {
        std::ofstream out(out_dir, std::ios::binary);
        out << HeaderRaw;
        out.write(reinterpret_cast<const char *>(cld_.data()), cld_.size() * header.point_size);
        std::cout << "data size: " << cld_.size() << " point size: " << header.point_size << std::endl;
    }

public:
    std::vector<Point> cloud;
};
} // namespace PCD