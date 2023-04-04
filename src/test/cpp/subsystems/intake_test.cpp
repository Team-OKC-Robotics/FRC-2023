#include <frc/Filesystem.h>
#include <gtest/gtest.h>
#include <iostream>

#include <filesystem>

#include "frc/geometry/Pose2d.h"

#include "subsystems/Intake.h"
#include "io/IntakeIO.h"
#include "Parameters.h"

class IntakeTest : public testing::Test {
public:
    virtual void SetUp() {
        intake_ = std::make_shared<Arm>(&intake_interface_);
        intake_->Init();
        // std::string deploy_path = frc::filesystem::GetDeployDirectory();
        // std::string param_file = deploy_path + "/parameters.toml";
        // RobotParams::LoadParameters(param_file);
    }

protected:
    std::shared_ptr<Intake> intake_;
    
    IntakeSoftwareInterface intake_interface_;
};

TEST_F(IntakeTest, InitIntake) {
    ASSERT_TRUE(intake_->Init());

}

TEST_F(IntakeTest, SetIntake) {
    ASSERT_TRUE(intake_->SetControlMode());
    ASSERT_TRUE(intake_->SetIntakePower());
    ASSERT_TRUE(intake_->SetTurn());
    //TODO  get value from mode
    //EXPECT_EQ(mode_, Manual);

}

TEST_F(IntakeTest, ControlIntake) {
    ASSERT_TRUE(intake_->ManualControl());
    ASSERT_TRUE(intake_->AutoControl());

}