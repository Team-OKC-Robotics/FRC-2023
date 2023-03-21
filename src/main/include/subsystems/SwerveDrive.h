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
#include "SlewRateLimiter.h"

#include <rev/CANSparkMax.h>

#define COAST rev::CANSparkMax::IdleMode::kCoast
#define BRAKE rev::CANSparkMax::IdleMode::kBrake



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
    bool SetIdleMode(rev::CANSparkMax::IdleMode mode);
    bool SetMaxOutputDrive(const double &max_output);
    bool SetMaxOutputSteer(const double &max_output);

    bool VectorTeleOpDrive(const double &drive, const double &strafe, const double &turn);
    
    bool InitAuto(TeamOKC::Pose pos, bool keep_heading);
    bool SetDistance(double dist);
    bool DriveAuto(double max_speed);
    bool AtDistSetpoint(bool *at);

    bool AutoBalance();
    bool AtBalanceSetpoint(bool *at);

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

    // slew rate limiters for steer
    std::shared_ptr<SlewRateLimiter> left_front_limiter_;
    std::shared_ptr<SlewRateLimiter> left_back_limiter_;
    std::shared_ptr<SlewRateLimiter> right_front_limiter_;
    std::shared_ptr<SlewRateLimiter> right_back_limiter_;

    double last_drive = 0.0;
    double last_strafe = 0.0;
    double last_turn = 0.0;

    double control_decay = 0.1;

    bool balanced_ = false;
    bool tilted_ = false;
    float last_pitch_ = 0.0;

    // max output
    double max_output_drive_ = 1;
    double max_output_steer_ = 1;

    // if the robot has reached the autonomous setpoint
    bool at_setpoint_ = false;

    double heading_to_goal_;
    double distance_to_goal_;

    double trackwidth_;
    double tracklength_;

    bool in_auto = false;
    bool auto_lock_heading_;
    TeamOKC::Pose position_;

    // pid controllers
    std::shared_ptr<frc::PIDController> heading_pid_;
    std::shared_ptr<frc::PIDController> dist_pid_;

    wpi::log::DoubleLogEntry left_front_setpoint_log_;
    wpi::log::DoubleLogEntry left_front_output_log_;
    wpi::log::DoubleLogEntry left_front_steer_enc_log_;

    wpi::log::DoubleLogEntry left_front_motor_output_log_;

    wpi::log::DoubleLogEntry drive_log_;
    wpi::log::DoubleLogEntry strafe_log_;
    wpi::log::DoubleLogEntry turn_log_;

    wpi::log::DoubleLogEntry imu_pitch_log_;
};
