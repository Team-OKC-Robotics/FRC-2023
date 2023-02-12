
#include <frc/Filesystem.h>
#include <gtest/gtest.h>
#include <iostream>

#include <filesystem>

#include "frc/geometry/Pose2d.h"

#include "SwerveModule.h"

class SwerveModuleTest : public testing::Test {
public:
    virtual void SetUp() {
        module_ = std::make_shared<SwerveModule>();
        module_->Init(Location(LEFT_FRONT));
    }

protected:
    std::shared_ptr<SwerveModule> module_;
};

TEST_F(SwerveModuleTest, InitModule) {
    ASSERT_TRUE(module_->Init(Location(LEFT_FRONT)));

    //TODO check configured values
}

TEST_F(SwerveModuleTest, ResetModule) {
    ASSERT_TRUE(module_->Reset());
}

TEST_F(SwerveModuleTest, AngleSetpointTest) {
    // test setting the steer PID setpoint
    const double angle = 30;
    ASSERT_TRUE(module_->SetAngle(angle));
    
    // test retreiving that setpoint
    double angle_tmp;
    EXPECT_EQ(module_->GetAngle(&angle_tmp), true);
    EXPECT_EQ(angle_tmp, angle);

    // test the PID setpoint logic (in this case, make sure we don't get a false-positive)
    EXPECT_EQ(module_->Update(0, 20, 0, 10), true);
    bool at_tmp = true;
    EXPECT_EQ(module_->AtSteerSetpoint(&at_tmp), true);
    EXPECT_EQ(at_tmp, false);

    // test for a false-negative
    EXPECT_EQ(module_->Update(0, 30, 0, 0), true);
    at_tmp = false;
    EXPECT_EQ(module_->AtSteerSetpoint(&at_tmp), true);
    EXPECT_EQ(at_tmp, true);

    //TODO test:
    //  - bool GetSteerError(double *error);
}

/**
    bool GetSteerEncoderReading(double *reading);
    bool GetSteerOutput(double *output); // PID, optimize angle

    bool SetDistance(double distance);
    bool GetDriveError(double *error);
    bool GetDriveOutput(double *output); // PID

    bool SetDrivePID(double kP, double kI, double kD);
    bool SetSteerPID(double kP, double kI, double kD);

    bool Update(double drive_enc, double steer_enc, double drive_vel, double steer_vel);
    bool Reset();

*/