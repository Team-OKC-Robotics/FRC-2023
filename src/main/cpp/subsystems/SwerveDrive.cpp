
#include "subsystems/SwerveDrive.h"
#include "SwerveModule.h"
#include <iostream>


bool SwerveDrive::Init() {
    // Initialize Shuffleboard from parameters.
    OKC_CALL(InitShuffleboard());

    left_front_setpoint_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/swerve/setpoint");
    left_front_output_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/swerve/output");
    left_front_steer_enc_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/swerve/steer_enc");

    double drive_max_output = RobotParams::GetParam("swerve.drive_max_output", 1);
    double drive_open_loop = RobotParams::GetParam("swerve.drive_open_loop", 1);
    double steer_max_output = RobotParams::GetParam("swerve.steer_max_output", 1);
    double steer_open_loop = RobotParams::GetParam("swerve.steer_open_loop", 1);

    // update swerve drive config
    interface_->drive_config = SwerveDriveConfig {
        drive_max_output,
        drive_open_loop,
        steer_max_output,
        steer_open_loop
    };
    interface_->update_config = true;

    // so the WPILib stuff expects the X axis to run from the back to the front of the robot, so positive x is forwards
    // hence tracklength is x disp
    tracklength_ = RobotParams::GetParam("swerve.x_disp", 0.3); // in meters
    trackwidth_ = RobotParams::GetParam("swerve.y_disp", 0.3); // in meters


    
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

    // create location objects
    left_front_loc_ = std::make_shared<frc::Translation2d>();
    left_back_loc_ = std::make_shared<frc::Translation2d>();
    right_front_loc_ = std::make_shared<frc::Translation2d>();
    right_back_loc_ = std::make_shared<frc::Translation2d>();

    OKC_CALL(left_front_module_->GetLocationOnRobot(left_front_loc_.get()));
    OKC_CALL(left_back_module_->GetLocationOnRobot(left_back_loc_.get()));
    OKC_CALL(right_front_module_->GetLocationOnRobot(right_front_loc_.get()));
    OKC_CALL(right_back_module_->GetLocationOnRobot(right_back_loc_.get()));
    
    OKC_CHECK(left_front_loc_ != nullptr);
    OKC_CHECK(left_back_loc_ != nullptr);
    OKC_CHECK(right_front_loc_ != nullptr);
    OKC_CHECK(right_back_loc_ != nullptr);

    // define SwerveKinematics object
    swerve_kinematics_ = std::make_shared<frc::SwerveDriveKinematics<4>>(*left_front_loc_, *left_back_loc_, *right_front_loc_, *right_back_loc_);
    OKC_CHECK(swerve_kinematics_ != nullptr);

    // create position objects
    left_front_pos_ = std::make_shared<frc::SwerveModulePosition>();
    left_back_pos_ = std::make_shared<frc::SwerveModulePosition>();
    right_front_pos_ = std::make_shared<frc::SwerveModulePosition>();
    right_back_pos_ = std::make_shared<frc::SwerveModulePosition>();

    OKC_CALL(left_front_module_->GetSwerveModulePosition(left_front_pos_.get()));
    OKC_CALL(left_back_module_->GetSwerveModulePosition(left_back_pos_.get()));
    OKC_CALL(right_front_module_->GetSwerveModulePosition(right_front_pos_.get()));
    OKC_CALL(right_back_module_->GetSwerveModulePosition(right_back_pos_.get()));

    OKC_CHECK(left_front_pos_ != nullptr);
    OKC_CHECK(left_back_pos_ != nullptr);
    OKC_CHECK(right_front_pos_ != nullptr);
    OKC_CHECK(right_back_pos_ != nullptr);

    
    // create list of position objects
    positions_ = std::make_shared<wpi::array<frc::SwerveModulePosition, 4>>(*left_front_pos_, *left_back_pos_, *right_front_pos_, *right_back_pos_);

    // define SwerveOdometry object with default 0 starting parameters
    OKC_CHECK(positions_ != nullptr);
    swerve_odometry_ = std::make_shared<frc::SwerveDriveOdometry<4>>(*swerve_kinematics_, frc::Rotation2d(), *positions_, frc::Pose2d());

    // our internal position object
    position_ = std::make_shared<frc::Pose2d>();

    // PID controllers
    double headingP = RobotParams::GetParam("swerve.heading_pid.kP", 0.0);
    double headingI = RobotParams::GetParam("swerve.heading_pid.kI", 0.0);
    double headingD = RobotParams::GetParam("swerve.heading_pid.kD", 0.0);

    heading_pid_ = std::make_shared<frc::PIDController>(headingP, headingI, headingD);

    // setpoint
    at_setpoint_ = false;
    

    // Reset everything
    OKC_CALL(ResetDriveEncoders());
    OKC_CALL(ResetGyro());
    return true;
}

bool SwerveDrive::UpdateModules() {
    // update modules
    OKC_CALL(left_front_module_->Update(this->interface_->left_front_drive_motor_enc, this->interface_->left_front_steer_motor_enc, this->interface_->left_front_drive_enc_vel, this->interface_->left_front_steer_enc_vel));
    OKC_CALL(left_back_module_->Update(this->interface_->left_back_drive_motor_enc, this->interface_->left_back_steer_motor_enc, this->interface_->left_back_drive_enc_vel, this->interface_->left_back_steer_enc_vel));
    OKC_CALL(right_front_module_->Update(this->interface_->right_front_drive_motor_enc, this->interface_->right_front_steer_motor_enc, this->interface_->right_front_drive_enc_vel, this->interface_->right_front_steer_enc_vel));
    OKC_CALL(right_back_module_->Update(this->interface_->right_back_drive_motor_enc, this->interface_->right_back_steer_motor_enc, this->interface_->right_back_drive_enc_vel, this->interface_->right_back_steer_enc_vel));

    // update module positions
    OKC_CALL(left_front_module_->GetSwerveModulePosition(left_front_pos_.get()));
    OKC_CALL(left_back_module_->GetSwerveModulePosition(left_back_pos_.get()));
    OKC_CALL(right_front_module_->GetSwerveModulePosition(right_front_pos_.get()));
    OKC_CALL(right_back_module_->GetSwerveModulePosition(right_back_pos_.get()));


    return true;
}

void SwerveDrive::Periodic() {
    VOKC_CHECK(interface_ != nullptr);
    
    // Update shuffleboard
    VOKC_CALL(UpdateShuffleboard());

    // get the heading
    double heading = 0;
    VOKC_CALL(this->GetHeading(&heading));

    VOKC_CHECK(left_front_module_ != nullptr);
    VOKC_CHECK(left_back_module_ != nullptr);
    VOKC_CHECK(right_front_module_ != nullptr);
    VOKC_CHECK(right_back_module_ != nullptr);

    VOKC_CALL(UpdateModules());

    // update odometry
    VOKC_CHECK(swerve_odometry_ != nullptr);
    swerve_odometry_->Update(frc::Rotation2d(units::degree_t(-heading)), *positions_); // negate heading because odometry expects counterclockwise to be positive, but the NavX is not

    // update our position object
    // *position_ = swerve_odometry_->GetPose();

    if (in_auto) {
            // if we're just starting auto
        if (auto_state_ == INIT) {
            std::cout << "INIT" << std::endl;

            // if keep_heading skip turning the robot
            if (this->auto_lock_heading_) {
                auto_state_ = ROTATE;
            
            // else turn the robot
            } else {
                VOKC_CHECK(this->left_front_module_ != nullptr);
                VOKC_CHECK(this->left_back_module_ != nullptr);
                VOKC_CHECK(this->right_front_module_ != nullptr);
                VOKC_CHECK(this->right_back_module_ != nullptr);

                //  rotate wheels to the 45/135 position to rotate the robot
                VOKC_CALL(this->left_front_module_->SetAngle(135));
                VOKC_CALL(this->left_back_module_->SetAngle(45));
                VOKC_CALL(this->right_front_module_->SetAngle(45));
                VOKC_CALL(this->right_back_module_->SetAngle(135));

                //  heading_pid to the NavX
                double heading = 0.0;
                VOKC_CALL(this->GetHeading(&heading));
                VOKC_CHECK(this->heading_pid_ != nullptr);
                double drive_output = this->heading_pid_->Calculate(heading);

                VOKC_CHECK(this->interface_ != nullptr);
                this->interface_->left_front_drive_motor_output = drive_output;
                this->interface_->left_back_drive_motor_output = drive_output;
                this->interface_->right_front_drive_motor_output = drive_output;
                this->interface_->right_back_drive_motor_output = drive_output;

                // done, go to ROTATE
                if (this->heading_pid_->AtSetpoint()) {
                    auto_state_ = ROTATE;
                }
            }
        // if we're to the turning stage
        } else if (auto_state_ == ROTATE) {
            VOKC_CHECK(this->left_front_module_ != nullptr);
            VOKC_CHECK(this->left_back_module_ != nullptr);
            VOKC_CHECK(this->right_front_module_ != nullptr);
            VOKC_CHECK(this->right_back_module_ != nullptr);

            // rotate wheels to face target
            if (this->auto_lock_heading_) {
                VOKC_CALL(this->left_front_module_->SetAngle(heading_to_goal_));
                VOKC_CALL(this->left_back_module_->SetAngle(heading_to_goal_));
                VOKC_CALL(this->right_front_module_->SetAngle(heading_to_goal_));
                VOKC_CALL(this->right_back_module_->SetAngle(heading_to_goal_));
            } else {
                // if not keep_heading, then they should all face forwards
                VOKC_CALL(this->left_front_module_->SetAngle(0.0));
                VOKC_CALL(this->left_back_module_->SetAngle(0.0));
                VOKC_CALL(this->right_front_module_->SetAngle(0.0));
                VOKC_CALL(this->right_back_module_->SetAngle(0.0));
            }

            std::cout << "ROTATING";

            double error = 0.0;
            this->left_front_module_->GetSteerError(&error);
            std::cout << error << std::endl;

            bool left_steer_complete = false;
            bool right_steer_complete = false;
            VOKC_CALL(this->left_front_module_->AtSteerSetpoint(&left_steer_complete));
            VOKC_CALL(this->right_back_module_->AtSteerSetpoint(&right_steer_complete));
            // if we've reached the setpoint
            if (left_steer_complete && right_steer_complete) {
                // proceed to next stage
                auto_state_ = TRANSLATE;
            }
        } else if (auto_state_ == TRANSLATE) {

            std::cout << "TRANSLATING" << std::endl;

            double error = 0.0;
            VOKC_CHECK(this->left_front_module_ != nullptr);
            VOKC_CALL(this->left_front_module_->GetDriveError(&error));
            // if we're close enough to the goal then we probably don't have to drive and this was just to turn
            if (abs(distance_to_goal_) < 0.1 || abs(error) < 0.1) {
                auto_state_ = COMPLETE;

                // stop the drive motors then
                VOKC_CHECK(this->interface_ != nullptr);
                this->interface_->left_front_drive_motor_output = 0.0;
                this->interface_->left_back_drive_motor_output = 0.0;
                this->interface_->right_front_drive_motor_output = 0.0;
                this->interface_->right_back_drive_motor_output = 0.0;

                this->interface_->left_front_steer_motor_output = 0.0;
                this->interface_->left_back_steer_motor_output = 0.0;
                this->interface_->right_front_steer_motor_output = 0.0;
                this->interface_->right_back_steer_motor_output = 0.0;
                in_auto = false;
            }

            VOKC_CHECK(this->left_front_module_ != nullptr);
            VOKC_CHECK(this->left_back_module_ != nullptr);
            VOKC_CHECK(this->right_front_module_ != nullptr);
            VOKC_CHECK(this->right_back_module_ != nullptr);

            // drive_pid the distance
            VOKC_CALL(this->left_front_module_->SetDistance(distance_to_goal_));
            VOKC_CALL(this->left_back_module_->SetDistance(distance_to_goal_));
            VOKC_CALL(this->right_front_module_->SetDistance(distance_to_goal_));
            VOKC_CALL(this->right_back_module_->SetDistance(distance_to_goal_));

            VOKC_CALL(this->left_front_module_->GetDriveOutput(&this->interface_->left_front_drive_motor_output));
            VOKC_CALL(this->left_back_module_->GetDriveOutput(&this->interface_->left_back_drive_motor_output));
            VOKC_CALL(this->right_front_module_->GetDriveOutput(&this->interface_->right_front_drive_motor_output));
            VOKC_CALL(this->right_back_module_->GetDriveOutput(&this->interface_->right_back_drive_motor_output));
        } else {
            // we shouldn't have gotten here, throw an error
            VOKC_CHECK_MSG(false, "error in autonomous state handling");
            return;
        }

        //TODO do I need to call Update() on the modules? periodic should still technically get called, I think
        VOKC_CHECK(this->left_front_module_ != nullptr);
        VOKC_CHECK(this->left_back_module_ != nullptr);
        VOKC_CHECK(this->right_front_module_ != nullptr);
        VOKC_CHECK(this->right_back_module_ != nullptr);

        // set motor outputs
        VOKC_CALL(this->left_front_module_->GetSteerOutput(&this->interface_->left_front_steer_motor_output));
        VOKC_CALL(this->left_back_module_->GetSteerOutput(&this->interface_->left_back_steer_motor_output));
        VOKC_CALL(this->right_front_module_->GetSteerOutput(&this->interface_->right_front_steer_motor_output));
        VOKC_CALL(this->right_back_module_->GetSteerOutput(&this->interface_->right_back_steer_motor_output));
    }
}

void SwerveDrive::SimulationPeriodic() {
    // SimulationPeriodic
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

bool SwerveDrive::TeleOpDrive(const double &drive, const double &strafe, const double &turn) {
    // get outputs from the kinematics object based on joystick inputs
    auto outputs = swerve_kinematics_->ToSwerveModuleStates(frc::ChassisSpeeds(units::meters_per_second_t(drive * 4), units::meters_per_second_t(strafe * 4), units::radians_per_second_t(turn * 1.5)));
    
    // set desired state in all the modules (set setpoints for PIDs)
    OKC_CALL(left_front_module_->SetDesiredState(outputs[0]));
    OKC_CALL(left_back_module_->SetDesiredState(outputs[1]));
    OKC_CALL(right_front_module_->SetDesiredState(outputs[2]));
    OKC_CALL(right_back_module_->SetDesiredState(outputs[3]));

    // set all the outputs in the interface
    // drive outputs
    OKC_CALL(left_front_module_->GetDriveOutput(&this->interface_->left_front_drive_motor_output));
    OKC_CALL(left_back_module_->GetDriveOutput(&this->interface_->left_back_drive_motor_output));

    OKC_CALL(right_front_module_->GetDriveOutput(&this->interface_->right_front_drive_motor_output));
    OKC_CALL(right_back_module_->GetDriveOutput(&this->interface_->right_back_drive_motor_output));
    
    // steer outputs
    OKC_CALL(left_front_module_->GetSteerOutput(&this->interface_->left_front_steer_motor_output));
    OKC_CALL(left_back_module_->GetSteerOutput(&this->interface_->left_back_steer_motor_output));
    
    OKC_CALL(right_front_module_->GetSteerOutput(&this->interface_->right_front_steer_motor_output));
    OKC_CALL(right_back_module_->GetSteerOutput(&this->interface_->right_back_steer_motor_output));

    // if we've made it here, we haven't errored, so return true
    return true;
}

bool SwerveDrive::VectorTeleOpDrive(const double &drive, const double &strafe, const double &turn) {
    double final_drive = drive * control_decay + last_drive * (1 - control_decay);
    double final_strafe = strafe * control_decay + last_strafe * (1 - control_decay);
    double final_turn = turn * control_decay  + last_turn * (1 - control_decay);

    //TODO convert `turn` to rad/sec
    //TODO convert drive and strafe to m/s I think
    // because tracklength/width are in meters

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


    // turn
    // double left_front_turn = atan2(B, D)  *  180.0/M_PI;
    // double left_back_turn = atan2(A, D)  *  180.0/M_PI;
    // double right_front_turn = atan2(B, C)  *  180.0/M_PI;
    // double right_back_turn = atan2(A, C)  *  180.0/M_PI;

    double right_front_turn = atan2(B, D)  *  180.0/M_PI;
    double right_back_turn = atan2(A, D)  *  180.0/M_PI;
    double left_front_turn = atan2(B, C)  *  180.0/M_PI;
    double left_back_turn = atan2(A, C)  *  180.0/M_PI;

    // keep the setpoints within [0, 360]
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
    left_back_module_->GetAngle(&left_front_angle);
    right_front_module_->GetAngle(&left_front_angle);
    right_back_module_->GetAngle(&left_front_angle);

    // do some funky invert stuff
    if (abs(left_front_angle - left_front_turn) > 90) {
        left_front_turn -= 180;
        left_front_speed *= -1;
    }

    if (abs(left_back_angle - left_back_turn) > 90) {
        left_back_turn -= 180;
        left_back_speed *= -1;
    }

    if (abs(right_front_angle - right_front_turn) > 90) {
        right_front_turn -= 180;
        right_front_speed *= -1;
    }

    if (abs(right_back_angle - right_back_turn) > 90) {
        right_back_turn -= 180;
        right_back_speed *= -1;
    }

    /**
     * 
     if (abs(right_back_angle - right_back_turn) > 90) {
        right_back_turn -= 180;
        right_back_speed *= -1;
    }

    so the diagonals travel more than 180 degrees for some reason
    they go from, say, 45 to 315 or something right
    which is in actuality only like 90 degrees, but
    315 - 45 > 90 so some wack stuff happens
     * 
    */


    // keep the setpoints within [0, 360]
    OKC_CALL(TeamOKC::WrapAngle(&left_front_turn));
    OKC_CALL(TeamOKC::WrapAngle(&left_back_turn));
    OKC_CALL(TeamOKC::WrapAngle(&right_front_turn));
    OKC_CALL(TeamOKC::WrapAngle(&right_back_turn));

    OKC_CHECK(this->left_front_module_ != nullptr);
    OKC_CHECK(this->left_back_module_ != nullptr);
    OKC_CHECK(this->right_front_module_ != nullptr);
    OKC_CHECK(this->right_back_module_ != nullptr);

    // really nice convoluted deadband
    // this is to stop the swerve modules from immediately trying to center themselves instead of
    // coasting until receiving another instruction so we don't tip
    if (abs(drive) > 0.05 || abs(strafe) > 0.05 || abs(turn) > 0.05) {
        OKC_CALL(this->left_front_module_->SetAngle(left_front_turn));
        OKC_CALL(this->left_back_module_->SetAngle(left_back_turn));
        OKC_CALL(this->right_front_module_->SetAngle(right_front_turn));
        OKC_CALL(this->right_back_module_->SetAngle(right_back_turn));
    }

    OKC_CALL(this->left_front_module_->GetSteerOutput(&this->interface_->left_front_steer_motor_output));
    OKC_CALL(this->left_back_module_->GetSteerOutput(&this->interface_->left_back_steer_motor_output));
    OKC_CALL(this->right_front_module_->GetSteerOutput(&this->interface_->right_front_steer_motor_output));
    OKC_CALL(this->right_back_module_->GetSteerOutput(&this->interface_->right_back_steer_motor_output));

    OKC_CHECK(this->interface_ != nullptr);
    this->interface_->left_front_drive_motor_output = left_front_speed;
    this->interface_->left_back_drive_motor_output = left_back_speed;
    this->interface_->right_front_drive_motor_output = right_front_speed;
    this->interface_->right_back_drive_motor_output = right_back_speed;

    // for control decay
    last_drive = drive;
    last_strafe = strafe;
    last_turn = turn;

    return true;
}



bool SwerveDrive::InitAuto(TeamOKC::Pose pos, bool keep_heading) {
    this->auto_lock_heading_ = keep_heading;

    //  1. figure out angle between here and there
    this->heading_to_goal_ = atan((position_->X().value() - pos.x) / (position_->Y().value() - pos.y));

    //  2. figure out distance
    this->distance_to_goal_ = sqrt(pow(position_->X().value() + pos.x, 2) + pow(position_->Y().value() + pos.x, 2));

    // set heading PID setpoint
    this->heading_pid_->SetSetpoint(this->heading_to_goal_);

    // initialization has passed, go ahead and start the autonomous
    auto_state_ = INIT;
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

    if (auto_state_ == COMPLETE) {
        *at = true;
    } else {
        *at = false;
    }

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
    // Get parameters
    double dist_p = RobotParams::GetParam("swerve.drive_pid.kP", 0.0);
    double dist_i = RobotParams::GetParam("swerve.drive_pid.kI", 0.0);
    double dist_d = RobotParams::GetParam("swerve.drive_pid.kD", 0.0);

    double steer_p = RobotParams::GetParam("swerve.steer_pid.kP", 0.0);
    double steer_i = RobotParams::GetParam("swerve.steer_pid.kI", 0.0);
    double steer_d = RobotParams::GetParam("swerve.steer_pid.kD", 0.0);

    // null pointer checks
    OKC_CHECK(SwerveDriveUI::nt_dist_kp);
    OKC_CHECK(SwerveDriveUI::nt_dist_ki);
    OKC_CHECK(SwerveDriveUI::nt_dist_kd);

    OKC_CHECK(SwerveDriveUI::nt_steer_kp);
    OKC_CHECK(SwerveDriveUI::nt_steer_ki);
    OKC_CHECK(SwerveDriveUI::nt_steer_kd);
    
    OKC_CHECK(SwerveDriveUI::nt_left_front_front_steer);
    OKC_CHECK(SwerveDriveUI::nt_left_back_front_steer);
    OKC_CHECK(SwerveDriveUI::nt_right_front_front_steer);
    OKC_CHECK(SwerveDriveUI::nt_right_back_front_steer);

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

    // === LOGGING ===
    OKC_CALL(right_front_module_->GetAngle(&encoder_tmp));
    left_front_setpoint_log_.Append(encoder_tmp);
    left_front_output_log_.Append(interface_->right_front_steer_motor_output);
    OKC_CALL(right_front_module_->GetSteerEncoderReading(&encoder_tmp));
    left_front_steer_enc_log_.Append(encoder_tmp);


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

            // check for nullptrs
            OKC_CHECK(SwerveDriveUI::nt_dist_kp != nullptr);
            OKC_CHECK(SwerveDriveUI::nt_dist_ki);
            OKC_CHECK(SwerveDriveUI::nt_dist_kd);

            OKC_CHECK(SwerveDriveUI::nt_steer_kp);
            OKC_CHECK(SwerveDriveUI::nt_steer_ki);
            OKC_CHECK(SwerveDriveUI::nt_steer_kd);

            // Get the values from shuffleboard.
            dist_p = SwerveDriveUI::nt_dist_kp->GetDouble(dist_p);
            dist_i = SwerveDriveUI::nt_dist_ki->GetDouble(dist_i);
            dist_d = SwerveDriveUI::nt_dist_kd->GetDouble(dist_d);

            steer_p = SwerveDriveUI::nt_steer_kp->GetDouble(steer_p);
            steer_i = SwerveDriveUI::nt_steer_ki->GetDouble(steer_i);
            steer_d = SwerveDriveUI::nt_steer_kd->GetDouble(steer_d);

            // Update PIDs with values
            OKC_CALL(left_front_module_->SetDrivePID(dist_p, dist_i, dist_d));
            OKC_CALL(left_back_module_->SetDrivePID(dist_p, dist_i, dist_d));
            OKC_CALL(right_front_module_->SetDrivePID(dist_p, dist_i, dist_d));
            OKC_CALL(right_back_module_->SetDrivePID(dist_p, dist_i, dist_d));

            OKC_CALL(left_front_module_->SetSteerPID(steer_p, steer_i, steer_d));
            OKC_CALL(left_back_module_->SetSteerPID(steer_p, steer_i, steer_d));
            OKC_CALL(right_front_module_->SetSteerPID(steer_p, steer_i, steer_d));
            OKC_CALL(right_back_module_->SetSteerPID(steer_p, steer_i, steer_d));
        }

        // Allow saving parameters in non-competition modes
        OKC_CHECK(SwerveDriveUI::nt_save != nullptr);
        if (SwerveDriveUI::nt_save->GetBoolean(true)) {
            // Save the parameters.
            OKC_CALL(RobotParams::SaveParameters(RobotParams::param_file));
            OKC_CALL(SwerveDriveUI::nt_save->SetBoolean(false));
        }
    }

    // Resetting the Gyro needs to always be available.
    OKC_CHECK(SwerveDriveUI::nt_reset_gyro != nullptr);
    if (SwerveDriveUI::nt_reset_gyro->GetBoolean(false)) {
        interface_->reset_gyro = true;
        OKC_CALL(SwerveDriveUI::nt_reset_gyro->SetBoolean(false));
    }

    return true;
}