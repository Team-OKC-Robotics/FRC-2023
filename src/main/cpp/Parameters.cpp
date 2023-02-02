#include "Parameters.h"

namespace RobotParams {
    toml::table parameters;

    // Save parameter file location.
    std::string param_file =
        frc::filesystem::GetDeployDirectory() + "/parameters.toml";

    bool LoadParameters(const std::string &path) {
        // Load the parameters from a file.
        try {
            parameters = toml::parse_file(path.c_str());
        } catch (const toml::parse_error &err) {
            std::cerr << "Parsing failed:\n" << err << "\n";
            return false;
        }

        return true;
    }

    bool SaveParameters(const std::string &path) {
        // Output the parameters table to the file.
        std::ofstream file;
        file.open(path, std::ofstream::out | std::ofstream::trunc);
        file << parameters << std::flush;
        file.close();

        return true;
    }

} // namespace RobotParams
