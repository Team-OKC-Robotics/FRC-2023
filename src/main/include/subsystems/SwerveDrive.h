// Copyright (c) Team OKC Robotics

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <memory>

#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/array.h>
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"
#include <frc2/command/SubsystemBase.h>

#include "Parameters.h"
#include "ui/UserInterface.h"
#include "Utils.h"
#include "io/SwerveDriveIO.h"
#include "SwerveModule.h"
#include "Logging.h"
#include "wpi/DataLog.h"

enum AutoState {
    INIT,
    ROTATE,
    TRANSLATE,
    COMPLETE
};

class SwerveDrive : public frc2::SubsystemBase {
public:
    SwerveDrive(SwerveDriveSoftwareInterface *interface)
        : interface_(interface) {}
    ~SwerveDrive() {}

    bool Init();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs during
     * simulation.
     */
    void SimulationPeriodic() override;

    bool SetSpeedModifierDrive(const double &speed_mod);
    bool SetSpeedModifierSteer(const double &speed_mod);
    bool SetOpenLoopRampDrive(const double &open_loop_ramp);
    bool SetOpenLoopRampSteer(const double &open_loop_ramp);
    bool SetMaxOutputDrive(const double &max_output);
    bool SetMaxOutputSteer(const double &max_output);

    bool TeleOpDrive(const double &drive, const double &strafe, const double &turn);
    bool DumbTeleOpDrive(const double &drive, const double &strafe, const double &turn);
    bool VectorTeleOpDrive(const double &drive, const double &strafe, const double &turn);
    
    bool InitAuto(frc::Pose2d pos, bool keep_heading);
    bool RunAuto();

    bool GetLeftDriveEncoderAverage(double *avg);
    bool GetRightDriveEncoderAverage(double *avg);
    bool GetDriveEncoderAverage(double *avg);
    bool GetLeftSteerEncoderAverage(double *avg);
    bool GetRightSteerEncoderAverage(double *avg);
    bool GetHeading(double *heading);

    bool AtSetpoint(bool *at);
    
    bool ResetDriveEncoders();
    bool ResetSteerEncoders();
    bool ResetPIDs();
    bool ResetGyro();

private:
    // Shuffleboard functions
    bool InitShuffleboard();
    bool UpdateShuffleboard();

    // software interface
    SwerveDriveSoftwareInterface *const interface_;

    // swerve modules
    std::shared_ptr<SwerveModule> left_front_module_;
    std::shared_ptr<SwerveModule> left_back_module_;
    std::shared_ptr<SwerveModule> right_front_module_;
    std::shared_ptr<SwerveModule> right_back_module_;

    // swerve module positions
    std::shared_ptr<frc::Translation2d> left_front_loc_;
    std::shared_ptr<frc::Translation2d> left_back_loc_;
    std::shared_ptr<frc::Translation2d> right_front_loc_;
    std::shared_ptr<frc::Translation2d> right_back_loc_;

    // swerve module states
    std::shared_ptr<frc::SwerveModulePosition> left_front_pos_;
    std::shared_ptr<frc::SwerveModulePosition> left_back_pos_;
    std::shared_ptr<frc::SwerveModulePosition> right_front_pos_;
    std::shared_ptr<frc::SwerveModulePosition> right_back_pos_;    

    // swerve module positions
    std::shared_ptr<wpi::array<frc::SwerveModulePosition, 4>> positions_;

    // kinematics
    std::shared_ptr<frc::SwerveDriveKinematics<4>> swerve_kinematics_;

    // odometry
    std::shared_ptr<frc::SwerveDriveOdometry<4>> swerve_odometry_;

    // position
    std::shared_ptr<frc::Pose2d> position_;

    // Speed modifier (the joystick input is multiplied by this value)
    double speed_modifier_drive_ = 0.75;
    double speed_modifier_steer_ = 0.75;

    // max output
    double max_output_drive_ = 1;
    double max_output_steer_ = 1;

    // if the robot has reached the autonomous setpoint
    bool at_setpoint_ = false;

    double heading_to_goal_;
    double distance_to_goal_;

    double trackwidth_;
    double tracklength_;

    AutoState auto_state_;
    bool auto_lock_heading_;

    // pid controllers
    std::shared_ptr<frc::PIDController> heading_pid_;

    wpi::log::DoubleLogEntry left_front_setpoint_log_;
    wpi::log::DoubleLogEntry left_front_output_log_;
    wpi::log::DoubleLogEntry left_front_steer_enc_log_;
};
