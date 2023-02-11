#include <frc/Filesystem.h>
#include <gtest/gtest.h>
#include <iostream>

#include <filesystem>

#include "frc/geometry/Pose2d.h"

#include "subsystems/Arm.h"
#include "io/ArmIO.h"

class ArmTest : public testing::Test {
public:
    virtual void SetUp() {
        arm_ = std::make_shared<Arm>(&arm_interface_);
        arm_->Init();
    }

protected:
    std::shared_ptr<Arm> arm_;
    
    ArmSoftwareInterface arm_interface_;
};

TEST_F(ArmTest, Init) {
    ASSERT_TRUE(arm_->Init());

    //TODO check configured values
}

TEST_F(ArmTest, SetArm) {
    ASSERT_TRUE(arm_->SetDegrees(1));
    ASSERT_TRUE(arm_->SetControlMode(Auto));

    arm_->Periodic();

    EXPECT_NE(arm_interface_.arm_lift_power, 0);
}
TEST_F(ArmTest, SetExtend) {
    ASSERT_TRUE(arm_->SetExtend(1));
    ASSERT_TRUE(arm_->SetControlMode(Auto));
    
    arm_->Periodic();

    EXPECT_NE(arm_interface_.arm_extend_power, 0);
}   
TEST_F(ArmTest, SetPreset) {
    ASSERT_TRUE(arm_->SetPreset(1));
    ASSERT_TRUE(arm_->SetControlMode(Auto));

    arm_->Periodic();

    EXPECT_NE(arm_interface_.arm_lift_power, 0);
}    
TEST_F(ArmTest, SetManual) {
    ASSERT_TRUE(arm_->SetManualLiftPower(1));
    ASSERT_TRUE(arm_->SetManualUpPower(1));
    ASSERT_TRUE(arm_->SetManualExtendPower(1));

    ASSERT_TRUE(arm_->SetControlMode(Manual));

    EXPECT_NE(arm_interface_.arm_extend_power, 0);
    EXPECT_NE(arm_interface_.arm_lift_power, 0);
} 

TEST_F(ArmTest, SetControl) {
    ASSERT_TRUE(arm_->SetControlMode(Manual));
    ASSERT_TRUE(arm_->ManualControl());
}

    