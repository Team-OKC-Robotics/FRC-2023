
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

    // update flags should've been set to true
    EXPECT_EQ(sw_interface_.reset_gyro, true);
    EXPECT_EQ(sw_interface_.reset_drive_encoders, true);
}

TEST_F(SwerveDriveTest, VectorTeleOpDrive) {
    // give all steer encoders initial 0 positions
    //TODO

    // tele-op drive with no power given
    ASSERT_TRUE(swerve_->VectorTeleOpDrive(0, 0, 0));

    // so everything should be 0 (except steer motors, because they're trying to PID to 0)
    EXPECT_EQ(sw_interface_.left_front_drive_motor_output, 0);
    EXPECT_EQ(sw_interface_.left_back_drive_motor_output, 0);
    EXPECT_EQ(sw_interface_.right_front_drive_motor_output, 0);
    EXPECT_EQ(sw_interface_.right_back_drive_motor_output, 0);
    //TODO steer motors


    // tele-op drive with some higher numbers
    ASSERT_TRUE(swerve_->VectorTeleOpDrive(0.5, 2, 4));

    EXPECT_NE(sw_interface_.left_front_drive_motor_output, 0);
    EXPECT_NE(sw_interface_.left_back_drive_motor_output, 0);
    EXPECT_NE(sw_interface_.right_front_drive_motor_output, 0);
    EXPECT_NE(sw_interface_.right_back_drive_motor_output, 0);

    // and also make sure none are greater than 1
}

//TEST_F(SwerveDriveTest, LockAutonomousDrive) {
//    //TODO better pos/heading
//    TeamOKC::Pose pos = TeamOKC::Pose(10, -5, 30);
//
//    ASSERT_TRUE(swerve_->InitAuto(pos, true)); // lock heading
//
//    swerve_->Periodic();
//
//    EXPECT_NE(sw_interface_.left_front_drive_motor_output, 0);


    //do some janky periodic() stuff maybe? just get it to go through all the states
    //TODO
//}

TEST_F(SwerveDriveTest, UnlockedAutoDrive) {
     //TODO better pos/heading
    TeamOKC::Pose pos = TeamOKC::Pose(-5, 20, 90);

    ASSERT_TRUE(swerve_->InitAuto(pos, false)); // turn to heading first before driving

    swerve_->Periodic();
    swerve_->Periodic();

    EXPECT_NE(sw_interface_.left_front_drive_motor_output, 0);

    //do some janky periodic() stuff maybe? just get it to go through all the states
    //TODO
}

TEST_F(SwerveDriveTest, GyroTest) {
    // initialize yaw in interface
    sw_interface_.imu_yaw = 0;

    // check to make sure the gyro can be reset
    ASSERT_TRUE(swerve_->ResetGyro());
    EXPECT_EQ(sw_interface_.reset_gyro, true);

    // check to make sure the software interface updates
    // Eli's number - thank you Eli
    const double yaw = 64;
    sw_interface_.imu_yaw = yaw;
    EXPECT_EQ(sw_interface_.imu_yaw, yaw);

    // check to make sure software interface updates make it to the subsystem
    double swerve_yaw = 0.0;
    ASSERT_TRUE(swerve_->GetHeading(&swerve_yaw));
    EXPECT_EQ(yaw, swerve_yaw);
}

TEST_F(SwerveDriveTest, DriveEncodersTest) {
    const double left_front = -100;
    const double left_back = -50;
    const double right_front = 100;
    const double right_back = 200;

    ASSERT_TRUE(swerve_->ResetDriveEncoders());
    EXPECT_EQ(sw_interface_.reset_drive_encoders, true);
    
    sw_interface_.left_front_drive_motor_enc = left_front;
    sw_interface_.left_back_drive_motor_enc = left_back;
    sw_interface_.right_front_drive_motor_enc = right_front;
    sw_interface_.right_back_drive_motor_enc = right_back;
    
    double avg = 0.0;
    ASSERT_TRUE(swerve_->GetLeftDriveEncoderAverage(&avg));
    EXPECT_EQ(avg, (left_front+left_back) / 2);
    
    ASSERT_TRUE(swerve_->GetRightDriveEncoderAverage(&avg));
    EXPECT_EQ(avg, (right_front+right_back) / 2);

    ASSERT_TRUE(swerve_->GetDriveEncoderAverage(&avg));
    EXPECT_EQ(avg, (left_front+left_back+right_front+right_back) / 4);

    //TODO velocity encoders
}

TEST_F(SwerveDriveTest, SteerEncodersTest) {
    //check to see if steerencoders can be reset
    ASSERT_TRUE(swerve_->ResetSteerEncoders());
    EXPECT_EQ(sw_interface_.reset_steer_encoders, true);
    
    sw_interface_.left_front_drive_motor_enc = left_front;
    sw_interface.left_back_drive_motor_enc = left_back;
    sw_interface_.right_front_drive_motor_enc = right_front;
    sw_interface_.right_back_drive_motor_enc = right_back;

    double avg = 0.0;
    ASSERT_TRUE(swerve_->GetLeftSteerEncoderAverge(&avg));
    EXPECT_EQ(avg, (left_front+left_back) / 2 );

    ASSERT_TRUE(swerve_->GetRightSteerEncoderAverage(&avg));
    EXPECT_EQ(avg, (right_front+right_back) / 2);

}

TEST_F(SwerveDriveTest, PeriodicTest) {
    swerve_->Periodic();
    //TODO change some things and verify they get updated by Periodic?
}

TEST_F(SwerveDriveTest, DistanceTest) {
    ASSERT_TRUE(swerve_->GetMotorOutput);
    EXPECT_NE(sw_interface_.left_front_drive_motor_output = 0);
    EXPECT_NE(sw_interface_.left_back_drive_motor_output = 0);
    EXPECT_NE(sw_interface_.right_front_drive_motor_output = 0);
    EXPECT_NE(sw_interface_.right_back_drive_motor_output = 0);
}

TEST_F(SwerveDriveTest, DriveAutoTest) {
    ASSERT_TRUE(swerve_->DriveAuto);
    EXPECT_NE(sw_interface_.left_front_drive_motor_output = 0);
    EXPECT_NE(sw_interface_.left_back_drive_motor_output = 0);
    EXPECT_NE(sw_interface_.right_front_drive_motor_output = 0);
    EXPECT_NE(sw_interface_.right_back_drive_motor_output = 0);
    EXPECT_EQ(max_speed, 0.5);
}

TEST_F(SwerveDriveTest, AutoBalanceTest) {
    ASSERT_TRUE(swerve_->AutoBalance);
    EXPECT_EQ(sw_interface_.left_front_drive_motor_output = 0.4);
    EXPECT_EQ(sw_interface_.left_back_drive_motor_output = 0.4);
    EXPECT_EQ(sw_interface_.right_front_drive_motor_output = 0.4);
    EXPECT_EQ(sw_interface_.right_back_drive_motor_output = 0.4);

    ASSERT_TRUE(swerve_->tilted);
    EXPECT_EQ(sw_interface_.left_front_drive_motor_output = 0.1);
    EXPECT_EQ(sw_interface_.left_back_drive_motor_output = 0.1);
    EXPECT_EQ(sw_interface_.right_front_drive_motor_output = 0.1);
    EXPECT_EQ(sw_interface_.right_back_drive_motor_output = 0.1);

    ASSERT_TRUE(swerve_->balanced);
    EXPECT_EQ(sw_interface_.left_front_drive_motor_output = 0);
    EXPECT_EQ(sw_interface_.left_back_drive_motor_output = 0);
    EXPECT_EQ(sw_interface_.right_front_drive_motor_output = 0);
    EXPECT_EQ(sw_interface_.right_back_drive_motor_output = 0.);




}