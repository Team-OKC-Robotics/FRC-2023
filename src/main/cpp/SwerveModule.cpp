
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
    steer_pid_->EnableContinuousInput(0, 360);

    // units and conversions and numbers and stuff
    L2_GEAR_RATIO_ = RobotParams::GetParam("swerve.l2_gear_ratio", 0.0);
    WHEEL_DIAMETER_ = RobotParams::GetParam("swerve.wheel_diameter", 0.0);
    INCHES_TO_METERS_ = RobotParams::GetParam("swerve.inches_to_meters", 0.0254);

    // create a default swerve module state_ with no speed or angle
    state_ = frc::SwerveModuleState(units::meters_per_second_t(0.0), frc::Rotation2d());

    // create a default swerve module position with no distance traveled or angle    
    pos_ = frc::SwerveModulePosition(units::meter_t(0.0), frc::Rotation2d());

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
            trans_ = frc::Translation2d(units::meter_t(x_disp), units::meter_t(y_disp));
            offset_ = RobotParams::GetParam("swerve.offset.left_front_offset", 0);
            break;
        case Location::LEFT_BACK:
            trans_ = frc::Translation2d(units::meter_t(-x_disp), units::meter_t(y_disp));
            offset_ = RobotParams::GetParam("swerve.offset.left_back_offset", 0);
            break;
        case Location::RIGHT_FRONT:
            trans_ = frc::Translation2d(frc::Translation2d(units::meter_t(x_disp), units::meter_t(-y_disp)));
            offset_ = RobotParams::GetParam("swerve.offset.right_front_offset", 0);
            break;
        case Location::RIGHT_BACK:
            trans_ = frc::Translation2d(frc::Translation2d(units::meter_t(-x_disp), units::meter_t(-y_disp)));
            offset_ = RobotParams::GetParam("swerve.offset.left_front_offset", 0);
            break;
        default:
            // we shouldn't have reached here, so throw an error
            offset_ = 0;
            return false;
    }

    // reset subsystem to initial state_
    this->Reset();

    // Init passed succesffully, return true
    return true;
}

bool SwerveModule::Reset() {
    VOKC_CHECK(this->drive_pid_ != nullptr);
    VOKC_CHECK(this->steer_pid_ != nullptr);

    // reset PIDs
    this->drive_pid_->Reset();
    this->steer_pid_->Reset();

    //TODO reset sensor readings
    //TODO reset WPILib kinematics stuff
    return true;
}

bool SwerveModule::GetLocationOnRobot(frc::Translation2d *trans) {
    OKC_CHECK(trans != nullptr);

    *trans = this->trans_;

    return true;
}

bool SwerveModule::GetSwerveModulePosition(frc::SwerveModulePosition *pos) {
    OKC_CHECK(pos != nullptr);

    *pos = this->pos_;

    return true;
}

bool SwerveModule::GetSwerveModuleState(frc::SwerveModuleState *state) {
    OKC_CHECK(state != nullptr);

    *state = this->state_;
    
    return true;
}

bool SwerveModule::SetDesiredState(frc::SwerveModuleState state) {
    this->state_ = state;

    return true;
}

bool SwerveModule::SetAngle(double angle) {
    this->steer_pid_->SetSetpoint(angle);

    return true;
}

bool SwerveModule::SetDistance(double distance) {
    this->drive_pid_->SetSetpoint(distance);

    return true;
}

bool SwerveModule::GetDriveOutput(double *output) {
    OKC_CHECK(this->drive_pid_ != nullptr);

    // set setpoint
    // this->drive_pid_->SetSetpoint(this->state_.speed.value());

    // calculate output
    // *output = this->drive_pid_->Calculate(this->drive_enc_vel + *output);

    *output = this->drive_pid_->Calculate(this->drive_enc_);

    return true;
}

bool SwerveModule::GetDriveError(double *error) {
    *error = this->drive_pid_->GetPositionError();

    return true;
}

bool SwerveModule::GetSteerOutput(double *output) {
    OKC_CHECK(this->steer_pid_ != nullptr);

    // optimize angle
    // this->state_ = frc::SwerveModuleState::Optimize(this->state_, frc::Rotation2d(units::degree_t(this->pos_.angle.Degrees())));

    // set setpoint
    // this->steer_pid_->SetSetpoint(this->state_.angle.Degrees().value());

    // if (location == RIGHT_FRONT || location == RIGHT_BACK) {
        // *output = -this->steer_pid_->Calculate(this->steer_enc);
    // } else {
    *output = this->steer_pid_->Calculate(this->steer_enc_);
    OKC_CALL(TeamOKC::Clamp(-0.4, 0.4, output));
    // }

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
    // update the SwerveModulePosition with the given sensor readings
    
    // 6.75:1 L2 gear ratio
    // wheel is 4 inch diameter wheel
    // .0254 to convert to meters
    this->drive_enc_ = drive_ / L2_GEAR_RATIO_ * M_PI * WHEEL_DIAMETER_ * INCHES_TO_METERS_;


    // steering gear ratio of 12.8:1
    // steer_enc = steer_enc / 12.8;
    // this->steer_enc = (double) abs(steer_ / 12.8 * 360.0);
    // this converts to degrees with 0 (and 360) being the front of the robot
    this->steer_enc_ = (steer_ * 360) - offset_;

    if (this->steer_enc_ > 360) {
        this->steer_enc_ -=360;
    } else if (this->steer_enc_ < 0) {
        this->steer_enc_ += 360;
    }

    // if (this->location == RIGHT_FRONT) {
    //     std::cout << this->steer_enc << std::endl;
    //     std::cout << this->steer_pid_->GetSetpoint() << std::endl;
    // }

    this->pos_ = frc::SwerveModulePosition(units::meter_t(drive_enc_), frc::Rotation2d(units::degree_t(steer_enc_)));

    // velocity readings
    this->steer_enc_vel_ = steer_vel;
    this->drive_enc_vel_ = drive_vel / L2_GEAR_RATIO_ * M_PI * WHEEL_DIAMETER_ * INCHES_TO_METERS_;

    return true;
}

bool SwerveModule::GetAngle(double *angle) {
    *angle = this->steer_pid_->GetSetpoint();

    return true;
}

bool SwerveModule::GetSteerEncoderReading(double *reading) {
    *reading = this->steer_enc_;

    return true;
}