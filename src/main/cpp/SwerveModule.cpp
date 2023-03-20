
#include "SwerveModule.h"
#include <iostream>

bool SwerveModule::Init(Location loc) {
    // PID Controller stuff
    // drive PID gains
    double drive_kP = RobotParams::GetParam("swerve.drive_pid.kP", 0.0);
    double drive_kI = RobotParams::GetParam("swerve.drive_pid.kI", 0.0);
    double drive_kD = RobotParams::GetParam("swerve.drive_pid.kD", 0.0);

    // steer PID gains
    double steer_kP = RobotParams::GetParam("swerve.steer_pid.kP", 0.0);
    double steer_kI = RobotParams::GetParam("swerve.steer_pid.kI", 0.0);
    double steer_kD = RobotParams::GetParam("swerve.steer_pid.kD", 0.0);

    // create our PIDs
    drive_pid_ = std::make_shared<frc::PIDController>(drive_kP, drive_kI, drive_kD);
    steer_pid_ = std::make_shared<frc::PIDController>(steer_kP, steer_kI, steer_kD);

    OKC_CHECK(this->steer_pid_ != nullptr);
    steer_pid_->EnableContinuousInput(-180, 180);
    steer_pid_->SetTolerance(2, 2); // tolerate 2 degrees of deviation, which shouldn't be a lot I don't think

    // units and conversions and numbers and stuff
    L2_GEAR_RATIO_ = RobotParams::GetParam("swerve.l2_gear_ratio", 6.75);
    WHEEL_DIAMETER_ = RobotParams::GetParam("swerve.wheel_diameter", 4);
    INCHES_TO_METERS_ = RobotParams::GetParam("swerve.inches_to_meters", 0.0254);

    steer_max_output_ = RobotParams::GetParam("swerve.steer_max_output", 1);

    this->location_ = loc;

    // physical location on the robot
    double x_disp = RobotParams::GetParam("swerve.x_disp", 0.3); // in meters
    double y_disp = RobotParams::GetParam("swerve.y_disp", 0.3); // in meters

    // based on where it needs to be
    switch(loc) {
        case Location::LEFT_FRONT:
            // we only have this to pass it into the kinematics object, because WPILib needs it
            // positive x is to the front of the robot, positive y is to the left of the robot
            // this should match the code in the docs pretty well, I think
            offset_ = RobotParams::GetParam("swerve.offset.left_front_offset", 0);
            break;
        case Location::LEFT_BACK:
            offset_ = RobotParams::GetParam("swerve.offset.left_back_offset", 0);
            break;
        case Location::RIGHT_FRONT:
            offset_ = RobotParams::GetParam("swerve.offset.right_front_offset", 0);
            break;
        case Location::RIGHT_BACK:
            offset_ = RobotParams::GetParam("swerve.offset.right_back_offset", 0);
            break;
        default:
            // we shouldn't have reached here, so throw an error
            offset_ = 0;
            return false;
    }

    // reset subsystem to initial state
    this->Reset();

    // Init passed succesffully, return true
    return true;
}

bool SwerveModule::Reset() {
    OKC_CHECK(this->drive_pid_ != nullptr);
    OKC_CHECK(this->steer_pid_ != nullptr);

    // reset PIDs
    this->drive_pid_->Reset();
    this->steer_pid_->Reset();

    //TODO reset sensor readings
    return true;
}

bool SwerveModule::SetAngle(double angle) {
    OKC_CHECK(this->steer_pid_ != nullptr);
    this->steer_pid_->SetSetpoint(angle);

    return true;
}

bool SwerveModule::SetDistance(double distance) {
    OKC_CHECK(this->drive_pid_ != nullptr);
    this->drive_pid_->SetSetpoint(distance);

    return true;
}

bool SwerveModule::GetDriveOutput(double *output) {
    OKC_CHECK(this->drive_pid_ != nullptr);

    *output = this->drive_pid_->Calculate(this->drive_enc_);

    return true;
}

bool SwerveModule::GetDriveError(double *error) {
    OKC_CHECK(this->drive_pid_ != nullptr);

    *error = this->drive_pid_->GetPositionError();

    return true;
}

bool SwerveModule::GetSteerOutput(double *output) {
    OKC_CHECK(this->steer_pid_ != nullptr);

    *output = this->steer_pid_->Calculate(this->steer_enc_);
    OKC_CALL(TeamOKC::Clamp(-steer_max_output_, steer_max_output_, output));

    return true;
}

bool SwerveModule::SetDrivePID(double kP, double kI, double kD) {
    OKC_CHECK(this->drive_pid_ != nullptr);

    this->drive_pid_->SetPID(kP, kI, kD);

    return true;
}

bool SwerveModule::SetSteerPID(double kP, double kI, double kD) {
    OKC_CHECK(this->steer_pid_ != nullptr);

    this->steer_pid_->SetPID(kP, kI, kD);

    return true;
}

bool SwerveModule::AtSteerSetpoint(bool *at) {
    OKC_CHECK(at != nullptr);
    OKC_CHECK(this->steer_pid_ != nullptr);
    *at = this->steer_pid_->AtSetpoint();

    return true;
}


bool SwerveModule::Update(double drive_, double steer_, double drive_vel, double steer_vel) {
    // 6.75:1 L2 gear ratio
    // wheel is 4 inch diameter wheel
    // .0254 to convert to meters
    this->drive_enc_ = drive_ / L2_GEAR_RATIO_ * M_PI * WHEEL_DIAMETER_ * INCHES_TO_METERS_;


    // steering gear ratio of 12.8:1
    // steer_enc = steer_enc / 12.8;
    // this->steer_enc = (double) abs(steer_ / 12.8 * 360.0);
    // this converts to degrees with 0 (and 360) being the front of the robot
    this->steer_enc_ = (steer_ * 360) - offset_;

    // keep the angle in bounds
    OKC_CALL(TeamOKC::WrapAngle(&this->steer_enc_));

    // velocity readings
    this->steer_enc_vel_ = steer_vel;
    this->drive_enc_vel_ = drive_vel / L2_GEAR_RATIO_ * M_PI * WHEEL_DIAMETER_ * INCHES_TO_METERS_;

    return true;
}

bool SwerveModule::GetAngle(double *angle) {
    OKC_CHECK(angle != nullptr);
    OKC_CHECK(this->steer_pid_ != nullptr);

    *angle = this->steer_pid_->GetSetpoint();

    return true;
}

bool SwerveModule::GetSteerEncoderReading(double *reading) {
    OKC_CHECK(reading != nullptr);

    *reading = this->steer_enc_;

    return true;
}

bool SwerveModule::GetSteerError(double *error) {
    *error = this->steer_pid_->GetPositionError();
    
    return true;
}

bool SwerveModule::GetDistance(double *dist) {
    *dist = this->drive_enc_;

    return true;
}