#include <frc/Filesystem.h>
#include <gtest/gtest.h>
#include <iostream>

#include <filesystem>

#include "frc/geometry/Pose2d.h"

#include "subsystems/Arm.h"
#include "io/ArmIO.h"
#include "Parameters.h"

class ArmTest : public testing::Test {
public:
    virtual void SetUp() {
        arm_ = std::make_shared<Arm>(&arm_interface_);
        arm_->Init();
        // std::string deploy_path = frc::filesystem::GetDeployDirectory();
        // std::string param_file = deploy_path + "/parameters.toml";
        // RobotParams::LoadParameters(param_file);
    }

protected:
    std::shared_ptr<Arm> arm_;
    
    ArmSoftwareInterface arm_interface_;
};

TEST_F(ArmTest, Init) {
    ASSERT_TRUE(arm_->Init());

    //TODO check configured values
}
    