
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

TEST_F(SwerveModuleTest, SteerEncoderReadingTest) {
    // steer encoder reading should be initialized to 0
    double reading = 0.0;
    EXPECT_TRUE(module_->GetSteerEncoderReading(&reading));
    EXPECT_EQ(reading, 0);

    //TODO fix this so it reads the offset from RobotParams
    const double offset = 330;

    // now test for Update()
    const double angle = 45;
    double sensor_value = (angle + offset) / 360;

    // wrap the sensor value
    if (sensor_value > 1) {
        sensor_value -= 1;
    } else if (sensor_value < 0) {
        sensor_value += 1;
    }
    
    reading = 0.0;
    // reading should update based off of calling Update()
    EXPECT_TRUE(module_->Update(0, sensor_value, 0, 0));
    EXPECT_TRUE(module_->GetSteerEncoderReading(&reading));
    EXPECT_EQ(reading, angle);
}

TEST_F(SwerveModuleTest, DrivePIDTest) {
    // test setting the P, I, and D gains for the drive PID
    ASSERT_TRUE(module_->SetDrivePID(0, 0, 0));
    ASSERT_TRUE(module_->SetDrivePID(1, 0, 0));
    ASSERT_TRUE(module_->SetDrivePID(1, 0.01, 0.11));
    ASSERT_TRUE(module_->SetDrivePID(0, 0.01, 100));
}

TEST_F(SwerveModuleTest, SteerPIDTest) {
    // test setting the P, I, and D gains for the steer PID
    ASSERT_TRUE(module_->SetSteerPID(0, 0, 0));
    ASSERT_TRUE(module_->SetSteerPID(1, 0, 0));
    ASSERT_TRUE(module_->SetSteerPID(1, 0.01, 0.11));
    ASSERT_TRUE(module_->SetSteerPID(0, 0.01, 100));
}


/**    
    bool GetSteerOutput(double *output); // PID, optimize angle

    bool SetDistance(double distance);
    bool GetDriveError(double *error);
    bool GetDriveOutput(double *output); // PID


    bool Update(double drive_enc, double steer_enc, double drive_vel, double steer_vel);
*/