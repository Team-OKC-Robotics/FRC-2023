
#include <frc/Filesystem.h>
#include <gtest/gtest.h>
#include <iostream>

#include <filesystem>
namespace fs = std::filesystem;

#include "Parameters.h"

class ParametersTest : public testing::Test {
public:
    virtual void SetUp() {
        // Get the FRC deploy folder path
        std::string deploy_path = frc::filesystem::GetDeployDirectory();
        std::string param_file = deploy_path + "/parameters.toml";

        // Load parameters
        ASSERT_TRUE(RobotParams::LoadParameters(param_file));
    }

protected:
};

TEST_F(ParametersTest, LoadParametersTest) {
    std::string title = RobotParams::GetParam("title", "");
    EXPECT_EQ(title, "OKC Robotics 2023 Parameters");
}

TEST_F(ParametersTest, GetSetParams) {
    ASSERT_TRUE(RobotParams::SetParam("title", "Test Title"));
    std::string title = RobotParams::GetParam("title", "");
    EXPECT_EQ(title, "Test Title");
}