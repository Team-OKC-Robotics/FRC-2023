
#pragma once

#include <iomanip>
#include <iostream>
#include <string.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sys/stat.h>
#include <filesystem>
namespace fs = std::filesystem;

#include <frc/Filesystem.h>

class OKCLog {

    public: 
        OKCLog() {}
        ~OKCLog() {
            if (log_out_.is_open()) {
                log_out_.close();
            }
        }

        std::string TimeToString(std::time_t t) {
            std::stringstream ss;
            ss << std::put_time(std::localtime(&t), "%Y-%m-%d_%X");
            return ss.str();
        }

        void Init(std::string file_prefix)
        {
            // Set start time.
            start_time_ = std::chrono::system_clock::now();
            std::time_t start = std::chrono::system_clock::to_time_t(start_time_);

            // Timestamp the file
            std::string timestamp = TimeToString(start);
            std::replace(timestamp.begin(), timestamp.end(), ':', '_');
            std::string filename = file_prefix + "_" + timestamp + ".csv";

            // Put the log in the operating directory
            std::string op_dir = frc::filesystem::GetOperatingDirectory();
            full_file_ = op_dir;
            full_file_ /= filename;

            // Determine if the file already exists. If it does, then don't rewrite the headers.
            bool write_headers = !FileExists(full_file_);

            // Open the file
            std::cout << full_file_ << std::endl;
            log_out_.open(full_file_,  std::ofstream::out | std::ios::app);

            // Apply headers if the file is new
            if (write_headers)
            {
                log_out_ << "Time,Label,Data\n" << std::flush;
            }
        }

        template<typename T>
        void Log(std::string label, T data) {
            auto cur_t = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = cur_t - start_time_;

            // Build the output for this log event and save it to the file.
            std::string row_output = std::to_string(elapsed_seconds.count()) + "," + label + "," + std::to_string(data) + "\n";
            log_out_ << row_output << std::flush;
        }

        bool FileExists(const fs::path& filename)
        {
            struct stat buf;
            std::string path_str = filename.string();
            if (stat(path_str.c_str(), &buf) != -1)
            {
                return true;
            }
            return false;
        }

    private:
        std::ofstream log_out_;
        fs::path full_file_;
        std::chrono::system_clock::time_point start_time_;
};
