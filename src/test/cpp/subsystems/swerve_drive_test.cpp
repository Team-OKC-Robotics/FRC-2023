
#include <frc/Filesystem.h>
#include <gtest/gtest.h>
#include <iostream>

#include <filesystem>

#include "frc/geometry/Pose2d.h"

#include "subsystems/SwerveDrive.h"
#include "io/SwerveDriveIO.h"

class SwerveDriveTest : public testing::Test {
public:
    virtual void SetUp() {
        swerve_ = std::make_shared<SwerveDrive>(&sw_interface_);
        swerve_->Init();
    }

protected:
    std::shared_ptr<SwerveDrive> swerve_;
    
    SwerveDriveSoftwareInterface sw_interface_;
};

TEST_F(SwerveDriveTest, InitSwerve) {
    ASSERT_TRUE(swerve_->Init());

    //TODO check configured values
}

TEST_F(SwerveDriveTest, ResetSwerve) {
    ASSERT_TRUE(swerve_->ResetPIDs());
    ASSERT_TRUE(swerve_->ResetDriveEncoders());
    ASSERT_TRUE(swerve_->ResetGyro());

    // update flags should've 
    ASSERT_EQ(sw_interface_.reset_gyro, true);
    ASSERT_EQ(sw_interface_.reset_drive_encoders, true);
}

TEST_F(SwerveDriveTest, VectorTeleOpDrive) {
    // give all steer encoders initial 0 positions

    // tele-op drive with no power given
    ASSERT_TRUE(swerve_->VectorTeleOpDrive(0, 0, 0));

    // so everything should be 0 (except steer motors, because they're trying to PID to 0)
    ASSERT_EQ(sw_interface_.left_front_drive_motor_output, 0);
    ASSERT_EQ(sw_interface_.left_back_drive_motor_output, 0);
    ASSERT_EQ(sw_interface_.right_front_drive_motor_output, 0);
    ASSERT_EQ(sw_interface_.right_back_drive_motor_output, 0);
    //TODO steer motors


    // tele-op drive with some higher numbers
    ASSERT_TRUE(swerve_->VectorTeleOpDrive(0.5, 2, 4));

    //TODO check motor outputs I suppose? make sure none of them are 0?
    // and also make sure none are greater than 1
}

TEST_F(SwerveDriveTest, WPITeleOpDrive) {
    ASSERT_TRUE(swerve_->TeleOpDrive(0, 0, 0));
    swerve_->Periodic();

    // so everything should be 0 (except steer motors, because they're trying to PID to 0)
    ASSERT_EQ(sw_interface_.left_front_drive_motor_output, 0);
    ASSERT_EQ(sw_interface_.left_back_drive_motor_output, 0);
    ASSERT_EQ(sw_interface_.right_front_drive_motor_output, 0);
    ASSERT_EQ(sw_interface_.right_back_drive_motor_output, 0);
    //TODO steer motors

    //TODO larger numbers
}

TEST_F(SwerveDriveTest, AutonomousDrive) {
    frc::Pose2d pos = frc::Pose2d();
    //TODO better pos/heading
    ASSERT_TRUE(swerve_->InitAuto(pos, true)); // lock heading

    ASSERT_TRUE(swerve_->RunAuto());

    //do some janky periodic() stuff maybe? just get it to go through all the states
    //TODO
}

TEST_F(SwerveDriveTest, GyroTest) {
    sw_interface_.imu_yaw = 0;
    ASSERT_TRUE(swerve_->ResetGyro());
    ASSERT_EQ(sw_interface_.reset_gyro, true);

    // Eli's number - thank you Eli
    const double yaw = 64;
    sw_interface_.imu_yaw = yaw;
    ASSERT_EQ(sw_interface_.imu_yaw, yaw);
}

TEST_F(SwerveDriveTest, DriveEncodersTest) {
    const double left_front = -100;
    const double left_back = -50;
    const double right_front = 100;
    const double right_back = 200;

    ASSERT_TRUE(swerve_->ResetDriveEncoders());
    
    sw_interface_.left_front_drive_motor_enc = left_front;
    sw_interface_.left_back_drive_motor_enc = left_back;
    sw_interface_.right_front_drive_motor_enc = right_front;
    sw_interface_.right_back_drive_motor_enc = right_back;
    
    double *avg;
    ASSERT_TRUE(swerve_->GetLeftDriveEncoderAverage(avg));
    ASSERT_EQ(*avg, (left_front+left_back) / 2);
    
    ASSERT_TRUE(swerve_->GetRightDriveEncoderAverage(avg));
    ASSERT_EQ(*avg, (right_front+right_back) / 2);

    ASSERT_TRUE(swerve_->GetDriveEncoderAverage(avg));
    ASSERT_EQ(*avg, (left_front+left_back+right_front+right_back) / 4);

    //TODO velocity encoders
    //FIXME this test fails
}

TEST_F(SwerveDriveTest, SteerEncodersTest) {
    //TODO
}

TEST_F(SwerveDriveTest, PeriodicTest) {
    // wait how does this test pass?
    swerve_->Periodic();
    //TODO
}

TEST_F(SwerveDriveTest, SwerveModuleTest) {
    //TODO
}
