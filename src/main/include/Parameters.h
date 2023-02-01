
#pragma once

#include <fstream>
#include <string.h>

#include <frc/Filesystem.h>

#include "Utils.h"
#include "third_party/toml.hpp"

namespace RobotParams {
    extern toml::table parameters;
    extern std::string param_file;

    bool LoadParameters(const std::string &path);
    bool SaveParameters(const std::string &path);

    template <typename T>
    T GetParam(const std::string_view &path, T default_val) {
        return parameters.at_path(path).value_or(default_val);
    }

    template <typename T> bool SetParam(const std::string_view &path, T value) {
        parameters.insert_or_assign(path, value);
        return true;
    }
} // namespace RobotParams
