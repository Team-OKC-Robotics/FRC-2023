
#include "subsystems/SwerveDrive.h"
#include "SwerveModule.h"
#include <iostream>


bool SwerveDrive::Init() {
    // Initialize Shuffleboard from parameters.
    OKC_CALL(InitShuffleboard());

    left_front_setpoint_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/swerve/setpoint");
    left_front_output_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/swerve/output");
    left_front_steer_enc_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/swerve/steer_enc");

    left_front_motor_output_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/swerve/drive_output");
    
    drive_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/joystick/drive");
    strafe_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/joystick/strafe");
    turn_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/joystick/turn");

    double drive_max_output = RobotParams::GetParam("swerve.drive_max_output", 1);
    double drive_open_loop = RobotParams::GetParam("swerve.drive_open_loop", 1);
    double steer_max_output = RobotParams::GetParam("swerve.steer_max_output", 1);
    double steer_open_loop = RobotParams::GetParam("swerve.steer_open_loop", 1);

    // update swerve drive config
    interface_->drive_config = SwerveDriveConfig {
        drive_max_output,
        drive_open_loop,
        steer_max_output,
        steer_open_loop,
        COAST // don't start in brake mode
    };
    interface_->update_config = true;

    // so the WPILib stuff expects the X axis to run from the back to the front of the robot, so positive x is forwards
    // hence tracklength is x disp
    tracklength_ = RobotParams::GetParam("swerve.x_disp", 0.3); // in meters
    trackwidth_ = RobotParams::GetParam("swerve.y_disp", 0.3); // in meters

    position_ = TeamOKC::Pose(0, 0, 0);
    
    // !! IMPORTANT ASSUMPTION/PRACTICE/WHATEVER !!
    // the order of swerve stuff should always be in:
    // left front, left back, right front, right back
    // for the sake of consistency and also if you want the swerve drive to actually work
    // because that should be how the argumetns are passed in, and however they're passed in,
    // that's how they gonna get passed out

    // initialize swerve modules
    left_front_module_ = std::make_shared<SwerveModule>();
    left_back_module_ = std::make_shared<SwerveModule>();
    right_front_module_ = std::make_shared<SwerveModule>();
    right_back_module_ = std::make_shared<SwerveModule>();

    OKC_CALL(left_front_module_->Init(LEFT_FRONT));
    OKC_CALL(left_back_module_->Init(LEFT_BACK));
    OKC_CALL(right_front_module_->Init(RIGHT_FRONT));
    OKC_CALL(right_back_module_->Init(RIGHT_BACK));

    OKC_CHECK(left_front_module_ != nullptr);
    OKC_CHECK(left_back_module_ != nullptr);
    OKC_CHECK(right_front_module_ != nullptr);
    OKC_CHECK(right_back_module_ != nullptr);

    // PID controllers
    double headingP = RobotParams::GetParam("swerve.heading_pid.kP", 0.01);
    double headingI = RobotParams::GetParam("swerve.heading_pid.kI", 0.0);
    double headingD = RobotParams::GetParam("swerve.heading_pid.kD", 0.0);

    heading_pid_ = std::make_shared<frc::PIDController>(headingP, headingI, headingD);

    double distP = RobotParams::GetParam("swerve.dist_pid.kP", 0.01);
    double distI = RobotParams::GetParam("swerve.dist_pid.kI", 0.0);
    double distD = RobotParams::GetParam("swerve.dist_pid.kD", 0.0);

    dist_pid_ = std::make_shared<frc::PIDController>(distP, distI, distD);

    double limit = RobotParams::GetParam("swerve.slew_limit", 0.0);

    left_front_limiter_ = std::make_shared<SlewRateLimiter>(limit);
    left_back_limiter_ = std::make_shared<SlewRateLimiter>(limit);
    right_front_limiter_ = std::make_shared<SlewRateLimiter>(limit);
    right_back_limiter_ = std::make_shared<SlewRateLimiter>(limit);

    // setpoint
    at_setpoint_ = false;

    // Reset everything
    OKC_CALL(ResetDriveEncoders());
    OKC_CALL(ResetGyro());
    return true;
}

void SwerveDrive::Periodic() {
    VOKC_CHECK(interface_ != nullptr);
    
    // update modules with sensor readings
    VOKC_CALL(left_front_module_->Update(this->interface_->left_front_drive_motor_enc, this->interface_->left_front_steer_motor_enc, this->interface_->left_front_drive_enc_vel, this->interface_->left_front_steer_enc_vel));
    VOKC_CALL(left_back_module_->Update(this->interface_->left_back_drive_motor_enc, this->interface_->left_back_steer_motor_enc, this->interface_->left_back_drive_enc_vel, this->interface_->left_back_steer_enc_vel));
    VOKC_CALL(right_front_module_->Update(this->interface_->right_front_drive_motor_enc, this->interface_->right_front_steer_motor_enc, this->interface_->right_front_drive_enc_vel, this->interface_->right_front_steer_enc_vel));
    VOKC_CALL(right_back_module_->Update(this->interface_->right_back_drive_motor_enc, this->interface_->right_back_steer_motor_enc, this->interface_->right_back_drive_enc_vel, this->interface_->right_back_steer_enc_vel));

    // Update shuffleboard
    VOKC_CALL(UpdateShuffleboard());

    // get the heading
    double heading = 0;
    VOKC_CALL(this->GetHeading(&heading));
}

void SwerveDrive::SimulationPeriodic() {
    // SimulationPeriodic
}

bool SwerveDrive::SetOpenLoopRampDrive(const double &open_loop_ramp) {
    OKC_CHECK(interface_ != nullptr);

    // update interface config with new open loop ramp rate
    interface_->drive_config.open_loop_ramp_rate_drive = open_loop_ramp;
    interface_->update_config = true;

    return true;
}

bool SwerveDrive::SetOpenLoopRampSteer(const double &open_loop_ramp) {
    OKC_CHECK(interface_ != nullptr);

    interface_->drive_config.open_loop_ramp_rate_steer = open_loop_ramp;
    return true;
}

bool SwerveDrive::SetIdleMode(rev::CANSparkMax::IdleMode mode) {
    OKC_CHECK(interface_ != nullptr);

    interface_->drive_config.idle_mode = mode;
    interface_->update_config = true;

    return true;
}

bool SwerveDrive::VectorTeleOpDrive(const double &drive, const double &strafe, const double &turn) {
    double final_drive = drive * control_decay + last_drive * (1 - control_decay);
    double final_strafe = strafe * control_decay + last_strafe * (1 - control_decay);
    double final_turn = turn * control_decay  + last_turn * (1 - control_decay);

    // if turn is very small
    if (abs(final_turn) < 0.3) {
        final_turn = 0.0; // then zero it
    // } else if (final_turn)
    }

    // if strafe is very small
    if (abs(final_strafe) < 0.15) {
        final_strafe = 0.0; // then zero it
    }

    // copied from ChiefDelphi thread
    //TODO post link here
    double A = final_strafe - final_turn * tracklength_/2;
    double B = final_strafe + final_turn * tracklength_/2;
    double C = final_drive - final_turn * trackwidth_/2;
    double D = final_drive + final_turn * trackwidth_/2;

    // speed
    double left_front_speed = sqrt(pow(B, 2) + pow(D, 2));
    double left_back_speed = sqrt(pow(A, 2) + pow(D, 2));
    double right_front_speed = sqrt(pow(B, 2) + pow(C, 2));
    double right_back_speed = sqrt(pow(A, 2) + pow(C, 2));

    double right_front_turn = atan2(B, D)  *  180.0/M_PI;
    double right_back_turn = atan2(A, D)  *  180.0/M_PI;
    double left_front_turn = atan2(B, C)  *  180.0/M_PI;
    double left_back_turn = atan2(A, C)  *  180.0/M_PI;

    // keep the setpoints within [-180, 180]
    OKC_CALL(TeamOKC::WrapAngle(&left_front_turn));
    OKC_CALL(TeamOKC::WrapAngle(&left_back_turn));
    OKC_CALL(TeamOKC::WrapAngle(&right_front_turn));
    OKC_CALL(TeamOKC::WrapAngle(&right_back_turn));

    // get current angle of all the modules
    double left_front_angle = 0.0;
    double left_back_angle = 0.0;
    double right_front_angle = 0.0;
    double right_back_angle = 0.0;

    left_front_module_->GetAngle(&left_front_angle);
    left_back_module_->GetAngle(&left_back_angle);
    right_front_module_->GetAngle(&right_front_angle);
    right_back_module_->GetAngle(&right_back_angle);

    /**
     * The following if statements exist to make tele-op better by turning the modules less
     * the basic premise of the code is that you should never have to steer the module more than 90 degrees.
     * this is because to reverse the direction of travel for a module, you _could_ turn it 180 degrees,
     * but it would be better to simply invert the direction that the drive motor/wheel is spinning, like normal
     * differential drivetrains do. So you should never need to steer more than 90 degrees.
    */
    if (abs(left_front_angle - left_front_turn) > 100) {
        left_front_turn -= 180;
        left_front_speed *= -1;
    }

    if (abs(left_back_angle - left_back_turn) > 100) {
        left_back_turn -= 180;
        left_back_speed *= -1;
    }

    if (abs(right_front_angle - right_front_turn) > 100) {
        right_front_turn -= 180;
        right_front_speed *= -1;
    }

    if (abs(right_back_angle - right_back_turn) > 100) {
        right_back_turn -= 180;
        right_back_speed *= -1;
    }

    // keep the setpoints within [-180, 180]
    OKC_CALL(TeamOKC::WrapAngle(&left_front_turn));
    OKC_CALL(TeamOKC::WrapAngle(&left_back_turn));
    OKC_CALL(TeamOKC::WrapAngle(&right_front_turn));
    OKC_CALL(TeamOKC::WrapAngle(&right_back_turn));

    OKC_CHECK(this->left_front_module_ != nullptr);
    OKC_CHECK(this->left_back_module_ != nullptr);
    OKC_CHECK(this->right_front_module_ != nullptr);
    OKC_CHECK(this->right_back_module_ != nullptr);

    // left_front_turn = left_front_limiter_->Calculate(left_front_turn);
    // left_back_turn = left_back_limiter_->Calculate(left_back_turn);
    // right_front_turn = right_front_limiter_->Calculate(right_front_turn);
    // right_back_turn = right_back_limiter_->Calculate(right_back_turn);

    // really nice convoluted deadband
    // this is to stop the swerve modules from immediately trying to center themselves instead of
    // coasting until receiving another instruction so we don't tip
    if (abs(final_drive) > 0.05 || abs(final_strafe) > 0.05 || abs(final_turn) > 0.3) {
        OKC_CALL(this->left_front_module_->SetAngle(left_front_turn));
        OKC_CALL(this->left_back_module_->SetAngle(left_back_turn));
        OKC_CALL(this->right_front_module_->SetAngle(right_front_turn));
        OKC_CALL(this->right_back_module_->SetAngle(right_back_turn));
    }

    if (abs(final_drive) < 0.05 && abs(final_strafe) < 0.05 && abs(final_turn) < 0.3) {
        this->interface_->left_front_drive_motor_output = 0.0;
        this->interface_->left_back_drive_motor_output = 0.0;
        this->interface_->right_front_drive_motor_output = 0.0;
        this->interface_->right_back_drive_motor_output = 0.0;
    } else {
        double left_front_steer_error = 0.0;
        double left_back_steer_error = 0.0;
        double right_front_steer_error = 0.0;
        double right_back_steer_error = 0.0;

        this->interface_->left_front_drive_motor_output = cos(TeamOKC::Radians(left_front_module_->GetSteerError(&left_front_steer_error))) * left_front_speed;
        this->interface_->left_back_drive_motor_output = cos(TeamOKC::Radians(left_back_module_->GetSteerError(&left_back_steer_error))) * left_back_speed;
        this->interface_->right_front_drive_motor_output = cos(TeamOKC::Radians(right_front_module_->GetSteerError(&right_front_steer_error))) * right_front_speed;
        this->interface_->right_back_drive_motor_output = cos(TeamOKC::Radians(right_back_module_->GetSteerError(&right_back_steer_error))) * right_back_speed;
    }

    OKC_CALL(this->left_front_module_->GetSteerOutput(&this->interface_->left_front_steer_motor_output));
    OKC_CALL(this->left_back_module_->GetSteerOutput(&this->interface_->left_back_steer_motor_output));
    OKC_CALL(this->right_front_module_->GetSteerOutput(&this->interface_->right_front_steer_motor_output));
    OKC_CALL(this->right_back_module_->GetSteerOutput(&this->interface_->right_back_steer_motor_output));

    OKC_CHECK(this->interface_ != nullptr);

    drive_log_.Append(drive);
    strafe_log_.Append(strafe);
    turn_log_.Append(turn);
   
    // for control decay
    last_drive = drive;
    last_strafe = strafe;
    last_turn = turn;

    return true;
}

bool SwerveDrive::SetDistance(double dist) {
    this->dist_pid_->SetSetpoint(dist);

    return true;
}

bool SwerveDrive::AtDistSetpoint(bool *at) {
    *at = this->dist_pid_->AtSetpoint();

    return true;
}

bool SwerveDrive::DriveAuto(double max_speed) {
    OKC_CHECK(this->interface_ != nullptr);

    double dist = 0;
    OKC_CALL(this->left_front_module_->GetDistance(&dist));

    double drive_power = this->dist_pid_->Calculate(dist);

    this->interface_->left_front_drive_motor_output = drive_power;
    this->interface_->left_back_drive_motor_output = drive_power;
    this->interface_->right_front_drive_motor_output = drive_power;
    this->interface_->right_back_drive_motor_output = drive_power;

    // clamp the speed
    TeamOKC::Clamp(-max_speed, max_speed, &this->interface_->left_front_drive_motor_output);
    TeamOKC::Clamp(-max_speed, max_speed, &this->interface_->left_back_drive_motor_output);
    TeamOKC::Clamp(-max_speed, max_speed, &this->interface_->right_front_drive_motor_output);
    TeamOKC::Clamp(-max_speed, max_speed, &this->interface_->right_back_drive_motor_output);


    // just drive straight
    OKC_CALL(this->left_front_module_->SetAngle(0));
    OKC_CALL(this->left_back_module_->SetAngle(0));
    OKC_CALL(this->right_front_module_->SetAngle(0));
    OKC_CALL(this->right_back_module_->SetAngle(0));

    OKC_CALL(this->left_front_module_->GetSteerOutput(&this->interface_->left_front_steer_motor_output));
    OKC_CALL(this->left_back_module_->GetSteerOutput(&this->interface_->left_back_steer_motor_output));
    OKC_CALL(this->right_front_module_->GetSteerOutput(&this->interface_->right_front_steer_motor_output));
    OKC_CALL(this->right_back_module_->GetSteerOutput(&this->interface_->right_back_steer_motor_output));


    return true;
}

bool SwerveDrive::AutoBalance() {
    // if we've started to go back down
    if (abs(this->interface_->imu_pitch) > -70) {
        tilted_ = true;
    }

    if (last_yaw_ < this->interface_->imu_pitch && !balanced_) {
        // then STOP ALL THE MOTORS RIGHT NOW and call it done
        this->interface_->left_front_drive_motor_output = 0.0;
        this->interface_->left_back_drive_motor_output = 0.0;
        this->interface_->right_front_drive_motor_output = 0.0;
        this->interface_->right_back_drive_motor_output = 0.0;

        OKC_CALL(this->left_front_module_->SetAngle(45));
        OKC_CALL(this->left_back_module_->SetAngle(45));
        OKC_CALL(this->right_front_module_->SetAngle(45));
        OKC_CALL(this->right_back_module_->SetAngle(45));
    
        OKC_CALL(this->left_front_module_->GetSteerOutput(&this->interface_->left_front_steer_motor_output));
        OKC_CALL(this->left_back_module_->GetSteerOutput(&this->interface_->left_back_steer_motor_output));
        OKC_CALL(this->right_front_module_->GetSteerOutput(&this->interface_->right_front_steer_motor_output));
        OKC_CALL(this->right_back_module_->GetSteerOutput(&this->interface_->right_back_steer_motor_output));

        balanced_ = true;
    } else if (tilted_) {
        // drive slowly backwards
        this->interface_->left_front_drive_motor_output = 0.1;
        this->interface_->left_back_drive_motor_output = 0.1;
        this->interface_->right_front_drive_motor_output = 0.1;
        this->interface_->right_back_drive_motor_output = 0.1;

        OKC_CALL(this->left_front_module_->SetAngle(0.0));
        OKC_CALL(this->left_back_module_->SetAngle(0.0));
        OKC_CALL(this->right_front_module_->SetAngle(0.0));
        OKC_CALL(this->right_back_module_->SetAngle(0.0));
    
        OKC_CALL(this->left_front_module_->GetSteerOutput(&this->interface_->left_front_steer_motor_output));
        OKC_CALL(this->left_back_module_->GetSteerOutput(&this->interface_->left_back_steer_motor_output));
        OKC_CALL(this->right_front_module_->GetSteerOutput(&this->interface_->right_front_steer_motor_output));
        OKC_CALL(this->right_back_module_->GetSteerOutput(&this->interface_->right_back_steer_motor_output));
    } else {
        // otherwise keep going
        this->interface_->left_front_drive_motor_output = 0.4;
        this->interface_->left_back_drive_motor_output = 0.4;
        this->interface_->right_front_drive_motor_output = 0.4;
        this->interface_->right_back_drive_motor_output = 0.4;

        OKC_CALL(this->left_front_module_->SetAngle(0.0));
        OKC_CALL(this->left_back_module_->SetAngle(0.0));
        OKC_CALL(this->right_front_module_->SetAngle(0.0));
        OKC_CALL(this->right_back_module_->SetAngle(0.0));
    
        OKC_CALL(this->left_front_module_->GetSteerOutput(&this->interface_->left_front_steer_motor_output));
        OKC_CALL(this->left_back_module_->GetSteerOutput(&this->interface_->left_back_steer_motor_output));
        OKC_CALL(this->right_front_module_->GetSteerOutput(&this->interface_->right_front_steer_motor_output));
        OKC_CALL(this->right_back_module_->GetSteerOutput(&this->interface_->right_back_steer_motor_output));
    }

    last_yaw_ = this->interface_->imu_pitch;
    
    return true;
}

bool SwerveDrive::InitAuto(TeamOKC::Pose pos, bool keep_heading) {
    this->auto_lock_heading_ = keep_heading;

    //  1. figure out angle between here and there
    this->heading_to_goal_ = atan((position_.x - pos.x) / (position_.y - pos.y));

    //  2. figure out distance
    this->distance_to_goal_ = sqrt(pow(position_.x + pos.x, 2) + pow(position_.y + pos.x, 2));

    // set heading PID setpoint
    this->heading_pid_->SetSetpoint(this->heading_to_goal_);

    // initialization has passed, go ahead and start the autonomous
    in_auto = true;

    return true;
}

bool SwerveDrive::GetHeading(double *heading) {
    OKC_CHECK(heading != nullptr);
    OKC_CHECK(interface_ != nullptr);

    *heading = interface_->imu_yaw;

    return true;
}

bool SwerveDrive::GetLeftDriveEncoderAverage(double *avg) {
    OKC_CHECK(interface_ != nullptr);

    double tmp = 0;

    tmp += interface_->left_front_drive_motor_enc;
    tmp += interface_->left_back_drive_motor_enc;

    *avg = (tmp / 2);

    return true;
}

bool SwerveDrive::GetRightDriveEncoderAverage(double *avg) {
    OKC_CHECK(interface_ != nullptr);

    double tmp = 0;
    
    tmp += interface_->right_front_drive_motor_enc;
    tmp += interface_->right_back_drive_motor_enc;

    *avg = (tmp / 2);

    return true;
}

bool SwerveDrive::GetDriveEncoderAverage(double *avg) {
    OKC_CHECK(interface_ != nullptr);

    double left = 0;
    double right = 0;
    
    OKC_CALL(this->GetLeftDriveEncoderAverage(&left));
    OKC_CALL(this->GetRightDriveEncoderAverage(&right));

    *avg = (left + right) / 2;

    return true;
}

bool SwerveDrive::AtSetpoint(bool *at) {
    OKC_CHECK(interface_ != nullptr);

    *at = false;

    return true;
}

bool SwerveDrive::ResetPIDs() {
    OKC_CHECK(left_front_module_ != nullptr);
    OKC_CHECK(left_back_module_ != nullptr);
    OKC_CHECK(right_front_module_ != nullptr);
    OKC_CHECK(right_back_module_ != nullptr);

    OKC_CALL(left_front_module_->Reset());
    OKC_CALL(left_back_module_->Reset());
    OKC_CALL(right_front_module_->Reset());
    OKC_CALL(right_back_module_->Reset());

    this->heading_pid_->Reset();
    this->dist_pid_->Reset();

    return true;
}


bool SwerveDrive::ResetDriveEncoders() {
    OKC_CHECK(interface_ != nullptr);

    interface_->reset_drive_encoders = true;
    return true;
}


bool SwerveDrive::ResetGyro() {
    OKC_CHECK(interface_ != nullptr);

    interface_->reset_gyro = true;
    return true;
}


bool SwerveDrive::InitShuffleboard() {
    OKC_CHECK(SwerveDriveUI::nt_left_front_front_steer);
    OKC_CHECK(SwerveDriveUI::nt_left_back_front_steer);
    OKC_CHECK(SwerveDriveUI::nt_right_front_front_steer);
    OKC_CHECK(SwerveDriveUI::nt_right_back_front_steer);

    // Update dashboard.
    OKC_CALL(SwerveDriveUI::nt_left_front_front_steer->SetDouble(0.0));
    OKC_CALL(SwerveDriveUI::nt_left_back_front_steer->SetDouble(0.0));
    OKC_CALL(SwerveDriveUI::nt_right_front_front_steer->SetDouble(0.0));
    OKC_CALL(SwerveDriveUI::nt_right_back_front_steer->SetDouble(0.0));

    return true;
}

bool SwerveDrive::UpdateShuffleboard() {
    // null pointer checks
    OKC_CHECK(SwerveDriveUI::nt_left_avg != nullptr);
    OKC_CHECK(SwerveDriveUI::nt_right_avg != nullptr);
    OKC_CHECK(SwerveDriveUI::nt_avg_dist != nullptr);

    OKC_CHECK(SwerveDriveUI::nt_left_front_front_steer != nullptr);
    OKC_CHECK(SwerveDriveUI::nt_left_back_front_steer != nullptr);
    OKC_CHECK(SwerveDriveUI::nt_right_front_front_steer != nullptr);
    OKC_CHECK(SwerveDriveUI::nt_right_back_front_steer != nullptr);
    
    OKC_CHECK(SwerveDriveUI::nt_left_front_steer_setpoint != nullptr);
    OKC_CHECK(SwerveDriveUI::nt_left_back_steer_setpoint != nullptr);
    OKC_CHECK(SwerveDriveUI::nt_right_front_steer_setpoint != nullptr);
    OKC_CHECK(SwerveDriveUI::nt_right_back_steer_setpoint != nullptr);
    
    OKC_CHECK(SwerveDriveUI::nt_left_front_steer_output != nullptr);

    OKC_CHECK(this->left_front_module_ != nullptr);

    // Update encoder UI
    double encoder_tmp = 0.0;
    OKC_CALL(this->GetLeftDriveEncoderAverage(&encoder_tmp));
    OKC_CALL(SwerveDriveUI::nt_left_avg->SetDouble(encoder_tmp));
    OKC_CALL(this->GetRightDriveEncoderAverage(&encoder_tmp));
    OKC_CALL(SwerveDriveUI::nt_right_avg->SetDouble(encoder_tmp));
    OKC_CALL(this->GetDriveEncoderAverage(&encoder_tmp));
    OKC_CALL(SwerveDriveUI::nt_avg_dist->SetDouble(encoder_tmp));

    OKC_CALL(SwerveDriveUI::nt_left_front_front_steer->SetDouble(this->interface_->left_front_steer_motor_enc * 360));
    OKC_CALL(SwerveDriveUI::nt_left_back_front_steer->SetDouble(this->interface_->left_back_steer_motor_enc * 360));
    OKC_CALL(SwerveDriveUI::nt_right_front_front_steer->SetDouble(this->interface_->right_front_steer_motor_enc * 360));
    OKC_CALL(SwerveDriveUI::nt_right_back_front_steer->SetDouble(this->interface_->right_back_steer_motor_enc * 360));

    double angle_tmp = 0.0;
    OKC_CALL(this->left_front_module_->GetAngle(&angle_tmp));
    OKC_CALL(SwerveDriveUI::nt_left_front_steer_setpoint->SetDouble(angle_tmp));

    OKC_CALL(this->left_back_module_->GetAngle(&angle_tmp));
    OKC_CALL(SwerveDriveUI::nt_left_back_steer_setpoint->SetDouble(angle_tmp));

    OKC_CALL(this->right_front_module_->GetAngle(&angle_tmp));
    OKC_CALL(SwerveDriveUI::nt_right_front_steer_setpoint->SetDouble(angle_tmp));

    OKC_CALL(this->right_back_module_->GetAngle(&angle_tmp));
    OKC_CALL(SwerveDriveUI::nt_right_back_steer_setpoint->SetDouble(angle_tmp));

    OKC_CALL(SwerveDriveUI::nt_left_front_steer_output->SetDouble(this->interface_->left_front_steer_motor_output));

    // Heading UI
    double heading_tmp = 0.0;
    OKC_CALL(this->GetHeading(&heading_tmp));
    OKC_CALL(SwerveDriveUI::nt_heading->SetDouble(heading_tmp));

    OKC_CALL(SwerveDriveUI::nt_pitch->SetDouble(this->interface_->imu_pitch));

    // === LOGGING ===
    OKC_CALL(right_front_module_->GetAngle(&encoder_tmp));
    left_front_setpoint_log_.Append(encoder_tmp);
    left_front_output_log_.Append(interface_->right_front_steer_motor_output);
    OKC_CALL(right_front_module_->GetSteerEncoderReading(&encoder_tmp));
    left_front_steer_enc_log_.Append(encoder_tmp);
    left_front_motor_output_log_.Append(this->interface_->right_front_drive_motor_output);

    // Resetting the Gyro needs to always be available.
    OKC_CHECK(SwerveDriveUI::nt_reset_gyro != nullptr);
    if (SwerveDriveUI::nt_reset_gyro->GetBoolean(false)) {
        interface_->reset_gyro = true;
        OKC_CALL(SwerveDriveUI::nt_reset_gyro->SetBoolean(false));
    }

    return true;
}