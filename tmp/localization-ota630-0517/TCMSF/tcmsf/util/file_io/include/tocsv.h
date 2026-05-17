#pragma once

#include "fmt/format.h"
#include <filesystem>
#include <fstream>
#include <string>

namespace fileio {
class ToCSV {
private:
    std::fstream fs;

public:
    bool open(const std::string &filepath) {
        // 输入文件路径

        // 如果路径不存在，则创建
        namespace fsys = std::filesystem;
        if (!fsys::exists(filepath)) {

            auto        idx  = filepath.find_last_of('/');
            std::string path = filepath.substr(0, idx);
            if (idx >= filepath.length() - 1) {
                return false;
            }
            if (!fsys::exists(path)) {
                if (!fsys::create_directories(path)) {
                    return false;
                }
            }
        } else {
            // 如果路径不是普通文件，则退出
            fsys::directory_entry entry(filepath);
            if (!entry.is_regular_file()) {
                return false;
            }
        }

        fs.open(filepath, std::ios::out);
        if (fs.is_open()) {
            return true;
        } else {
            return false;
        }
    }
    void line(const std::string &line) {
        fs << line;
    }
    ~ToCSV() {
        if (fs.is_open()) {
            fs.close();
        }
    }
};

class FromCSV {
private:
    std::fstream fs;

public:
    bool open(const std::string &filepath) {
        // 输入文件路径

        // 如果路径不存在，则创建
        namespace fsys = std::filesystem;
        if (!fsys::exists(filepath)) {
            return false;
        } else {
            // 如果路径不是普通文件，则退出
            fsys::directory_entry entry(filepath);
            if (!entry.is_regular_file()) {
                return false;
            }
        }

        fs.open(filepath, std::ios::in);
        if (fs.is_open()) {
            return true;
        } else {
            return false;
        }
    }
    bool get_line(std::string &str_) {
        if (fs.is_open()) {
            if (getline(fs, str_)) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }
    ~FromCSV() {
        if (fs.is_open()) {
            fs.close();
        }
    }
};

} // namespace fileio