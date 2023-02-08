
#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <memory>

#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <wpi/array.h>
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include <frc2/command/SubsystemBase.h>
#include "units/length.h"
#include "units/velocity.h"
#include "units/math.h"
#include "units/voltage.h"
#include "frc/controller/SimpleMotorFeedforward.h"

#include "Parameters.h"
#include "ui/UserInterface.h"
#include "Utils.h"
#include "io/SwerveDriveIO.h"

#include "frc/filter/SlewRateLimiter.h"
#include "units/base.h"

enum Location {
    LEFT_FRONT,
    LEFT_BACK,
    RIGHT_FRONT,
    RIGHT_BACK
};

class SwerveModule {
public:
    SwerveModule()
     : drive_pid_(), steer_pid_(), state_(), pos_(), trans_(), location_() {};
    ~SwerveModule() {}

    bool Init(Location loc);

    bool GetSwerveModulePosition(frc::SwerveModulePosition *pos);
    bool GetSwerveModuleState(frc::SwerveModuleState *state);

    bool GetLocationOnRobot(frc::Translation2d *loc);
    
    bool SetDesiredState(const frc::SwerveModuleState &state);
    bool SetAngle(double angle);
    bool SetDistance(double distance);

    bool GetAngle(double *angle);
    bool GetSteerError(double *error);
    bool GetDriveError(double *error);
    bool GetSteerEncoderReading(double *reading);
    
    bool GetDriveOutput(double *output); // PID
    bool GetSteerOutput(double *output); // PID, optimize angle

    bool SetDrivePID(double kP, double kI, double kD);
    bool SetSteerPID(double kP, double kI, double kD);

    bool AtSteerSetpoint(bool *at);

    bool Update(double drive_enc, double steer_enc, double drive_vel, double steer_vel);
    bool Reset();

private:
    std::shared_ptr<frc::PIDController> drive_pid_;
    std::shared_ptr<frc::PIDController> steer_pid_;

    // frc::SlewRateLimiter<units::scalar> steer_filter;

    frc::SwerveModuleState state_;
    frc::SwerveModulePosition pos_;

    frc::Translation2d trans_;

    Location location_;

    double drive_enc_;
    double steer_enc_;

    double drive_enc_vel_;
    double steer_enc_vel_;

    double offset_;

    double steer_max_output;

    double L2_GEAR_RATIO_;
    double WHEEL_DIAMETER_;
    double INCHES_TO_METERS_;
};