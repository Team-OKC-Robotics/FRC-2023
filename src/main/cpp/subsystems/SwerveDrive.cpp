
#include "subsystems/SwerveDrive.h"
#include "SwerveModule.h"
#include <iostream>


bool SwerveDrive::Init() {
    // Initialize Shuffleboard from parameters.
    OKC_CALL(InitShuffleboard());

    // update swerve drive config
    interface_->drive_config = SwerveDriveConfig {
        0.5, // drive max output
        0.5, // drive open loop ramp rate
        0.5, // steer max
        0.5  // steer ramp rate
    };
    interface_->update_config = true;

    // so the WPILib stuff expects the X axis to run from the back to the front of the robot, so positive x is forwards
    // hence tracklength is x disp
    tracklength = RobotParams::GetParam("swerve.x_disp", 0.3); // in meters
    trackwidth = RobotParams::GetParam("swerve.y_disp", 0.3); // in meters

    
    // !! IMPORTANT ASSUMPTION/PRACTICE/WHATEVER !!
    // the order of swerve stuff should always be in:
    // left front, left back, right front, right back
    // for the sake of consistency and also if you want the swerve drive to actually work
    // because that should be how the argumetns are passed in, and however they're passed in,
    // that's how they gonna get passed out

    // initialize swerve modules
    left_front_module = std::make_shared<SwerveModule>();
    left_back_module = std::make_shared<SwerveModule>();
    right_front_module = std::make_shared<SwerveModule>();
    right_back_module = std::make_shared<SwerveModule>();

    OKC_CALL(left_front_module->Init(LEFT_FRONT));
    OKC_CALL(left_back_module->Init(LEFT_BACK));
    OKC_CALL(right_front_module->Init(RIGHT_FRONT));
    OKC_CALL(right_back_module->Init(RIGHT_BACK));

    OKC_CHECK(left_front_module != nullptr);
    OKC_CHECK(left_back_module != nullptr);
    OKC_CHECK(right_front_module != nullptr);
    OKC_CHECK(right_back_module != nullptr);

    // create location objects
    left_front_loc = std::make_shared<frc::Translation2d>();
    left_back_loc = std::make_shared<frc::Translation2d>();
    right_front_loc = std::make_shared<frc::Translation2d>();
    right_back_loc = std::make_shared<frc::Translation2d>();

    OKC_CALL(left_front_module->GetLocationOnRobot(left_front_loc.get()));
    OKC_CALL(left_back_module->GetLocationOnRobot(left_back_loc.get()));
    OKC_CALL(right_front_module->GetLocationOnRobot(right_front_loc.get()));
    OKC_CALL(right_back_module->GetLocationOnRobot(right_back_loc.get()));
    
    OKC_CHECK(left_front_loc != nullptr);
    OKC_CHECK(left_back_loc != nullptr);
    OKC_CHECK(right_front_loc != nullptr);
    OKC_CHECK(right_back_loc != nullptr);

    // define SwerveKinematics object
    swerve_kinematics = std::make_shared<frc::SwerveDriveKinematics<4>>(*left_front_loc, *left_back_loc, *right_front_loc, *right_back_loc);
    OKC_CHECK(swerve_kinematics != nullptr);

    // create position objects
    left_front_pos = std::make_shared<frc::SwerveModulePosition>();
    left_back_pos = std::make_shared<frc::SwerveModulePosition>();
    right_front_pos = std::make_shared<frc::SwerveModulePosition>();
    right_back_pos = std::make_shared<frc::SwerveModulePosition>();

    OKC_CALL(left_front_module->GetSwerveModulePosition(left_front_pos.get()));
    OKC_CALL(left_back_module->GetSwerveModulePosition(left_back_pos.get()));
    OKC_CALL(right_front_module->GetSwerveModulePosition(right_front_pos.get()));
    OKC_CALL(right_back_module->GetSwerveModulePosition(right_back_pos.get()));

    OKC_CHECK(left_front_pos != nullptr);
    OKC_CHECK(left_back_pos != nullptr);
    OKC_CHECK(right_front_pos != nullptr);
    OKC_CHECK(right_back_pos != nullptr);

    
    // create list of position objects
    positions = std::make_shared<wpi::array<frc::SwerveModulePosition, 4>>(*left_front_pos, *left_back_pos, *right_front_pos, *right_back_pos);
    OKC_CHECK(positions != nullptr);

    // define SwerveOdometry object with default 0 starting parameters
    swerve_odometry = std::make_shared<frc::SwerveDriveOdometry<4>>(*swerve_kinematics, frc::Rotation2d(), *positions, frc::Pose2d());
    OKC_CHECK(swerve_odometry != nullptr);

    // our internal position object
    position = std::make_shared<frc::Pose2d>();
    OKC_CHECK(position != nullptr);

    // setpoint
    at_setpoint = false;
    

    // Reset everything
    OKC_CALL(ResetDriveEncoders());
    OKC_CALL(ResetSteerEncoders());
    OKC_CALL(ResetGyro());
    return true;
}

void SwerveDrive::Periodic() {
    VOKC_CHECK(interface_ != nullptr);
    
    // Update shuffleboard
    VOKC_CALL(UpdateShuffleboard());

    // get the heading
    double heading = 0;
    VOKC_CALL(this->GetHeading(&heading));

    VOKC_CHECK(left_front_module != nullptr);
    VOKC_CHECK(left_back_module != nullptr);
    VOKC_CHECK(right_front_module != nullptr);
    VOKC_CHECK(right_back_module != nullptr);


    // update modules
    VOKC_CALL(left_front_module->Update(this->interface_->left_front_drive_motor_enc, this->interface_->left_front_steer_motor_enc, this->interface_->left_front_drive_enc_vel, this->interface_->left_front_steer_enc_vel));
    VOKC_CALL(left_back_module->Update(this->interface_->left_back_drive_motor_enc, this->interface_->left_back_steer_motor_enc, this->interface_->left_back_drive_enc_vel, this->interface_->left_back_steer_enc_vel));
    VOKC_CALL(right_front_module->Update(this->interface_->right_front_drive_motor_enc, this->interface_->right_front_steer_motor_enc, this->interface_->right_front_drive_enc_vel, this->interface_->right_front_steer_enc_vel));
    VOKC_CALL(right_back_module->Update(this->interface_->right_back_drive_motor_enc, this->interface_->right_back_steer_motor_enc, this->interface_->right_back_drive_enc_vel, this->interface_->right_back_steer_enc_vel));

    // update module positions
    VOKC_CALL(left_front_module->GetSwerveModulePosition(left_front_pos.get()));
    VOKC_CALL(left_back_module->GetSwerveModulePosition(left_back_pos.get()));
    VOKC_CALL(right_front_module->GetSwerveModulePosition(right_front_pos.get()));
    VOKC_CALL(right_back_module->GetSwerveModulePosition(right_back_pos.get()));

    // update odometry
    swerve_odometry->Update(frc::Rotation2d(units::degree_t(-heading)), *positions); // negate heading because odometry expects counterclockwise to be positive, but the NavX is not

    // update our position object
    *position = swerve_odometry->GetPose();
}

void SwerveDrive::SimulationPeriodic() {
    // SimulationPeriodic
}












// okay, these are probably for shuffleboard, but they're not being used right now
bool SwerveDrive::SetSpeedModifierDrive(const double &speed_mod) {
    speed_modifier_drive = speed_mod;

    return true;
}

bool SwerveDrive::SetSpeedModifierSteer(const double &speed_mod) {
    speed_modifier_steer = speed_mod;
    
    return true;
}

bool SwerveDrive::SetOpenLoopRampDrive(const double &open_loop_ramp) {
    OKC_CHECK(interface_ != nullptr);

    interface_->drive_config.open_loop_ramp_rate_drive = open_loop_ramp;
    return true;
}

bool SwerveDrive::SetOpenLoopRampSteer(const double &open_loop_ramp) {
    OKC_CHECK(interface_ != nullptr);

    interface_->drive_config.open_loop_ramp_rate_steer = open_loop_ramp;
    return true;
}

bool SwerveDrive::SetMaxOutputDrive(const double &max_output) {
    OKC_CHECK(interface_ != nullptr);

    interface_->drive_config.max_output_drive = max_output;
    return true;
}

bool SwerveDrive::SetMaxOutputSteer(const double &max_output) {
    OKC_CHECK(interface_ != nullptr);

    interface_->drive_config.max_output_steer = max_output;
    return true;
}

















bool SwerveDrive::TeleOpDrive(const double &drive, const double &strafe, const double &turn) {
    // get outputs from the kinematics object based on joystick inputs
    auto outputs = swerve_kinematics->ToSwerveModuleStates(frc::ChassisSpeeds(units::meters_per_second_t(drive * 4), units::meters_per_second_t(strafe * 4), units::radians_per_second_t(turn * 1.5)));
    
    // set desired state in all the modules (set setpoints for PIDs)
    OKC_CALL(left_front_module->SetDesiredState(outputs[0]));
    OKC_CALL(left_back_module->SetDesiredState(outputs[1]));
    OKC_CALL(right_front_module->SetDesiredState(outputs[2]));
    OKC_CALL(right_back_module->SetDesiredState(outputs[3]));

    // set all the outputs in the interface
    // drive outputs
    OKC_CALL(left_front_module->GetDriveOutput(&this->interface_->left_front_drive_motor_output));
    OKC_CALL(left_back_module->GetDriveOutput(&this->interface_->left_back_drive_motor_output));

    OKC_CALL(right_front_module->GetDriveOutput(&this->interface_->right_front_drive_motor_output));
    OKC_CALL(right_back_module->GetDriveOutput(&this->interface_->right_back_drive_motor_output));
    
    // steer outputs
    OKC_CALL(left_front_module->GetSteerOutput(&this->interface_->left_front_steer_motor_output));
    OKC_CALL(left_back_module->GetSteerOutput(&this->interface_->left_back_steer_motor_output));
    
    OKC_CALL(right_front_module->GetSteerOutput(&this->interface_->right_front_steer_motor_output));
    OKC_CALL(right_back_module->GetSteerOutput(&this->interface_->right_back_steer_motor_output));

    // if we've made it here, we haven't errored, so return true
    return true;
}

/**
 * In case the WPILib kinematics stuff doesn't work out
 */
bool SwerveDrive::DumbTeleOpDrive(const double &drive, const double &strafe, const double &turn) {
    // given: drive power, strafe power, turn power
    //  1. figure out what angle the drive and strafe thing is at
    //  2. figure out how much turn power there is
    //  3. figure out magnitude of drive/strafe
    //  4. figure out magnitude of turn
    //  5. add turn stuff together
    //  6. set steer encoders to turn stuff
    //  7. set drive power to drive magnitude
    
    // so that's:
    //  1. atan(strafe / drive)
    //  2. turn/45 // degrees. when turn is 1, the value should be 45 degrees. when turn is 0, the value should be 0 degrees. so that's 1:45, so just div by 45?
    //  3. sqrt(drive**2 + strafe**2)
    //  4. I think this is just straight [turn]
    //  5. steer = turn+angle_from_step_1
    //  6. PID
    //  7. set_output->(result_of_step_3)
    
    double drive_strafe_angle = atan(strafe / drive); // TODO figure out how this is gonna work with  negative/positive/turning/stuff
    double drive_strafe_magnitude = sqrt(pow(drive, 2) + pow(strafe, 2)); // TODO don't forget to copy the signs because just squaring gets rid of negatives
    drive_strafe_magnitude = copysign(drive_strafe_magnitude, drive);
    double turn_magnitude = turn;
    double magnitude = (drive_strafe_magnitude + turn_magnitude) / 2;

    double left_front_turn = 0;
    double left_back_turn = 0;
    double right_front_turn = 0;
    double right_back_turn = 0;

    // if turn is negative (i.e. left)
    if (turn < -0.1) {
        // set the turning appropriately
        left_front_turn = -turn * 135;
        left_back_turn = -turn * 45;
        right_front_turn = -turn * 45;
        right_back_turn = -turn * 135;
    } else if (turn > 0.1) {
        // if turning is positive (i.e. right), do likewise
        left_front_turn = turn * 45;
        left_back_turn = turn * 135;
        right_front_turn = turn * 135;
        right_back_turn = turn * 45;
    } else {
        // otherwise, we don't want to turn
        left_front_turn = 0;
        left_back_turn = 0;
        right_front_turn = 0;
        right_back_turn = 0;
    }


    left_front_turn += drive_strafe_angle;
    left_back_turn += drive_strafe_angle;
    right_front_turn += drive_strafe_angle;
    right_back_turn += drive_strafe_angle;

    // turn angle
    OKC_CALL(this->left_front_module->SetAngle(left_front_turn));
    OKC_CALL(this->left_back_module->SetAngle(left_back_turn));
    OKC_CALL(this->right_front_module->SetAngle(right_front_turn));
    OKC_CALL(this->right_back_module->SetAngle(right_back_turn));

    OKC_CALL(this->left_front_module->GetSteerOutput(&this->interface_->left_front_steer_motor_output));
    OKC_CALL(this->left_back_module->GetSteerOutput(&this->interface_->left_back_steer_motor_output));
    OKC_CALL(this->right_front_module->GetSteerOutput(&this->interface_->right_front_steer_motor_output));
    OKC_CALL(this->right_back_module->GetSteerOutput(&this->interface_->right_back_steer_motor_output));

    if (abs(drive_strafe_magnitude) < 0.1) {
        this->interface_->left_front_drive_motor_output = 0;
        this->interface_->left_back_drive_motor_output = 0;
        this->interface_->right_front_drive_motor_output = 0;
        this->interface_->right_back_drive_motor_output = 0;
        return true;
    }

    // drive power
    this->interface_->left_front_drive_motor_output = magnitude;
    this->interface_->left_back_drive_motor_output = magnitude;
    this->interface_->right_front_drive_motor_output = magnitude;
    this->interface_->right_back_drive_motor_output = magnitude;
    

    return true;
}

bool SwerveDrive::VectorTeleOpDrive(const double &drive, const double &strafe, const double &turn) {
    //TODO convert `turn` to rad/sec
    //TODO convert drive and strafe to m/s I think
    // because tracklength/width are in meters

    // copied from ChiefDelphi thread
    //TODO post link here
    double A = strafe - turn * tracklength/2;
    double B = strafe + turn * tracklength/2;
    double C = drive - turn * trackwidth/2;
    double D = drive + turn * trackwidth/2;

    // speed
    double left_front_speed = sqrt(pow(B, 2) + pow(D, 2));
    double left_back_speed = sqrt(pow(A, 2) + pow(D, 2));
    double right_front_speed = sqrt(pow(B, 2) + pow(C, 2));
    double right_back_speed = sqrt(pow(A, 2) + pow(C, 2));


    // turn
    double left_front_turn = atan2(B, D)  *  180.0/pi;
    double left_back_turn = atan2(A, D)  *  180.0/pi;
    double right_front_turn = atan2(B, C)  *  180.0/pi;
    double right_back_turn = atan2(A, C)  *  180.0/pi;

    // std::cout << left_front_turn << std::endl;

    if (left_front_turn < 0) {
        left_front_turn += 360;
    }

    if (left_back_turn < 0) {
        left_back_turn += 360;
    }

    if (right_front_turn < 0) {
        right_front_turn += 360;
    }
    
    if (right_back_turn < 0) {
        right_back_turn += 360;
    }
    
    OKC_CALL(this->left_front_module->SetAngle(left_front_turn));
    OKC_CALL(this->left_back_module->SetAngle(left_back_turn));
    OKC_CALL(this->right_front_module->SetAngle(right_front_turn));
    OKC_CALL(this->right_back_module->SetAngle(right_back_turn));

    OKC_CALL(this->left_front_module->GetSteerOutput(&this->interface_->left_front_steer_motor_output));
    OKC_CALL(this->left_back_module->GetSteerOutput(&this->interface_->left_back_steer_motor_output));
    OKC_CALL(this->right_front_module->GetSteerOutput(&this->interface_->right_front_steer_motor_output));
    OKC_CALL(this->right_back_module->GetSteerOutput(&this->interface_->right_back_steer_motor_output));

    this->interface_->left_front_drive_motor_output = left_front_speed;
    this->interface_->left_back_drive_motor_output = left_back_speed;
    this->interface_->right_front_drive_motor_output = right_front_speed;
    this->interface_->right_back_drive_motor_output = right_back_speed;


    return true;
}



bool SwerveDrive::InitAuto(frc::Pose2d pos, bool keep_heading) {
    this->auto_lock_heading = keep_heading;

    //  1. figure out angle between here and there
    this->heading_to_goal = atan((position->X().value() - pos.X().value()) / (position->Y().value() - pos.Y().value()));

    //  2. figure out distance
    this->distance_to_goal = sqrt(pow(position->X().value() + pos.X().value(), 2) + pow(position->Y().value() + pos.Y().value(), 2));

    // initialization has passed, go ahead and start the autonomous
    auto_state = INIT;

    return true;
}

bool SwerveDrive::RunAuto() {
    if (auto_state == INIT) {
        // if keep_heading skip
        if (this->auto_lock_heading) {
            auto_state = ROTATE;
        // else:
        } else {
            //  rotate wheels to the 45/135 position to rotate the robot
            //  heading_pid to the NavX
            //TODO

            // done, go to ROTATE
            // if (this->heading_pid->AtSetpoint()) {
                // auto_state = ROTATE;
            // }
        }
    } else if (auto_state == ROTATE) {
        this->left_front_module->SetAngle(heading_to_goal);
        this->left_back_module->SetAngle(heading_to_goal);
        this->right_front_module->SetAngle(heading_to_goal);
        this->right_back_module->SetAngle(heading_to_goal);
        // rotate wheels to face target
        // if not keep_heading, then they should all face forwards
    } else if (auto_state == TRANSLATE) {
        // if dist = 0 (or close enough) then skip to ROTATE_FINAL
        // drive_pid the distance
        
    } else {
        // we shouldn't have gotten here, throw an error
        return false;
    }

    //TODO do I need to call Update() on the modules?

    // set motor outputs
    OKC_CALL(this->left_front_module->GetDriveOutput(&this->interface_->left_front_drive_motor_output));
    OKC_CALL(this->left_back_module->GetDriveOutput(&this->interface_->left_back_drive_motor_output));
    OKC_CALL(this->right_front_module->GetDriveOutput(&this->interface_->right_front_drive_motor_output));
    OKC_CALL(this->right_back_module->GetDriveOutput(&this->interface_->right_back_drive_motor_output));

    OKC_CALL(this->left_front_module->GetSteerOutput(&this->interface_->left_front_steer_motor_output));
    OKC_CALL(this->left_back_module->GetSteerOutput(&this->interface_->left_back_steer_motor_output));
    OKC_CALL(this->right_front_module->GetSteerOutput(&this->interface_->right_front_steer_motor_output));
    OKC_CALL(this->right_back_module->GetSteerOutput(&this->interface_->right_back_steer_motor_output));

    return true;
}

// bool SwerveDrive::TranslateAuto(frc::Pose2d pos) {
    //TODO
    // given: initial pos, final pos
    //  1. figure out angle between here and there
    //  2. figure out distance
    //  3. turn to angle
    //  4. drive the distance
    //  5. figure out difference between current and final angle
    //  6. turn to final angle
    //  7. set AtSetpoint to true
    /**
     * 1. atan((position.x - pos.x) / (position.y - pos.y))
     * 2. sqrt((poxition.x - pos.x) ** 2  +  (position.y - pos.y) ** 2)
     * 3. PID
     * 4. #2
     * 5. pos.rot - getHeading()
     * 6. PID
     * 7. at_setpoint = true;
     */

//     double heading_to_goal = atan((position->X().value() - pos.X().value()) / (position->Y().value() - pos.Y().value()));
//     double distance_to_goal = sqrt(pow(position->X().value() + pos.X().value(), 2) + pow(position->Y().value() + pos.Y().value(), 2));

//     //TODO
//     //TODO

//     double heading_to_goal_heading = pos.Rotation().Degrees().value() - position->Rotation().Degrees().value();

//     //TODO

//     at_setpoint = true;

//     return true;
// }

bool SwerveDrive::GetHeading(double *heading) {
    OKC_CHECK(heading != nullptr);
    OKC_CHECK(interface_ != nullptr);

    heading = &interface_->imu_yaw;

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

bool SwerveDrive::GetLeftSteerEncoderAverage(double *avg) {
    OKC_CHECK(interface_ != nullptr);

    double tmp = 0;
    
    tmp += interface_->right_front_steer_motor_enc;
    tmp += interface_->right_back_steer_motor_enc;

    *avg = (tmp / 2);

    return true;
}

bool SwerveDrive::GetRightSteerEncoderAverage(double *avg) {
    OKC_CHECK(interface_ != nullptr);

    double tmp = 0;
    
    tmp += interface_->right_front_steer_motor_enc;
    tmp += interface_->right_back_steer_motor_enc;

    *avg = (tmp / 2);

    return true;
}

bool SwerveDrive::AtSetpoint(bool *at) {
    OKC_CHECK(interface_ != nullptr);

    //TODO

    return true;
}

bool SwerveDrive::ResetPIDs() {
    OKC_CALL(left_front_module->Reset());
    OKC_CALL(left_back_module->Reset());
    OKC_CALL(right_front_module->Reset());
    OKC_CALL(right_back_module->Reset());

    return true;
}


bool SwerveDrive::ResetDriveEncoders() {
    OKC_CHECK(interface_ != nullptr);

    interface_->reset_drive_encoders = true;
    return true;
}

bool SwerveDrive::ResetSteerEncoders() {
    OKC_CHECK(interface_ != nullptr);

    interface_->reset_steer_encoders = true;
    return true;
}


bool SwerveDrive::ResetGyro() {
    OKC_CHECK(interface_ != nullptr);

    interface_->reset_gyro = true;
    return true;
}


bool SwerveDrive::InitShuffleboard() {
    // Get parameters
    double dist_p = RobotParams::GetParam("swerve.drive_pid.kP", 0.0);
    double dist_i = RobotParams::GetParam("swerve.drive_pid.kI", 0.0);
    double dist_d = RobotParams::GetParam("swerve.drive_pid.kD", 0.0);

    double steer_p = RobotParams::GetParam("swerve.steer_pid.kP", 0.0);
    double steer_i = RobotParams::GetParam("swerve.steer_pid.kI", 0.0);
    double steer_d = RobotParams::GetParam("swerve.steer_pid.kD", 0.0);

    // Update dashboard.
    OKC_CALL(SwerveDriveUI::nt_dist_kp->SetDouble(dist_p));
    OKC_CALL(SwerveDriveUI::nt_dist_ki->SetDouble(dist_i));
    OKC_CALL(SwerveDriveUI::nt_dist_kd->SetDouble(dist_d));

    OKC_CALL(SwerveDriveUI::nt_steer_kp->SetDouble(steer_p));
    OKC_CALL(SwerveDriveUI::nt_steer_ki->SetDouble(steer_i));
    OKC_CALL(SwerveDriveUI::nt_steer_kd->SetDouble(steer_d));

    OKC_CALL(SwerveDriveUI::nt_left_front_front_steer->SetDouble(0.0));
    OKC_CALL(SwerveDriveUI::nt_left_back_front_steer->SetDouble(0.0));
    OKC_CALL(SwerveDriveUI::nt_right_front_front_steer->SetDouble(0.0));
    OKC_CALL(SwerveDriveUI::nt_right_back_front_steer->SetDouble(0.0));

    return true;
}

bool SwerveDrive::UpdateShuffleboard() {
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

    OKC_CALL(SwerveDriveUI::nt_left_front_steer_setpoint->SetDouble(this->left_front_module->GetAngle()));
    OKC_CALL(SwerveDriveUI::nt_left_back_steer_setpoint->SetDouble(this->left_back_module->GetAngle()));
    OKC_CALL(SwerveDriveUI::nt_right_front_steer_setpoint->SetDouble(this->right_front_module->GetAngle()));
    OKC_CALL(SwerveDriveUI::nt_right_back_steer_setpoint->SetDouble(this->right_back_module->GetAngle()));

    // Heading UI
    double heading_tmp = 0.0;
    OKC_CALL(this->GetHeading(&heading_tmp));
    OKC_CALL(SwerveDriveUI::nt_heading->SetDouble(heading_tmp));

    // If competition mode isn't set to true, then allow the PID gains to be
    // tuned.
    bool is_competition = RobotParams::GetParam("competition", false);
    if (!is_competition) {
        // Update the PID Gains if write mode is true.
        if (SwerveDriveUI::nt_write_mode->GetBoolean(false)) {
            // Get the current PID parameter values
            double dist_p = RobotParams::GetParam("swerve.distance_pid.Kp", 0.0);
            double dist_i = RobotParams::GetParam("swerve.distance_pid.Ki", 0.0);
            double dist_d = RobotParams::GetParam("swerve.distance_pid.Kp", 0.0);

            double steer_p = RobotParams::GetParam("swerve.steer_pid.Kp", 0.0);
            double steer_i = RobotParams::GetParam("swerve.steer_pid.Ki", 0.0);
            double steer_d = RobotParams::GetParam("swerve.steer_pid.Kp", 0.0);

            // Get the values from shuffleboard.
            dist_p = SwerveDriveUI::nt_dist_kp->GetDouble(dist_p);
            dist_i = SwerveDriveUI::nt_dist_ki->GetDouble(dist_i);
            dist_d = SwerveDriveUI::nt_dist_kd->GetDouble(dist_d);

            steer_p = SwerveDriveUI::nt_steer_kp->GetDouble(steer_p);
            steer_i = SwerveDriveUI::nt_steer_ki->GetDouble(steer_i);
            steer_d = SwerveDriveUI::nt_steer_kd->GetDouble(steer_d);

            // Update PIDs with values
            OKC_CALL(left_front_module->SetDrivePID(dist_p, dist_i, dist_d));
            OKC_CALL(left_back_module->SetDrivePID(dist_p, dist_i, dist_d));
            OKC_CALL(right_front_module->SetDrivePID(dist_p, dist_i, dist_d));
            OKC_CALL(right_back_module->SetDrivePID(dist_p, dist_i, dist_d));

            OKC_CALL(left_front_module->SetSteerPID(steer_p, steer_i, steer_d));
            OKC_CALL(left_back_module->SetSteerPID(steer_p, steer_i, steer_d));
            OKC_CALL(right_front_module->SetSteerPID(steer_p, steer_i, steer_d));
            OKC_CALL(right_back_module->SetSteerPID(steer_p, steer_i, steer_d));
        }

        // Allow saving parameters in non-competition modes
        if (SwerveDriveUI::nt_save->GetBoolean(true)) {
            // Save the parameters.
            OKC_CALL(RobotParams::SaveParameters(RobotParams::param_file));
            OKC_CALL(SwerveDriveUI::nt_save->SetBoolean(false));
        }
    }

    // Resetting the Gyro needs to always be available.
    if (SwerveDriveUI::nt_reset_gyro->GetBoolean(false)) {
        interface_->reset_gyro = true;
        OKC_CALL(SwerveDriveUI::nt_reset_gyro->SetBoolean(false));
    }

    return true;
}