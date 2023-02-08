
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
     : drive_pid_(), steer_pid_(), location_() {};
    ~SwerveModule() {}

    bool Init(Location loc);
    
    bool SetAngle(double angle);
    bool GetAngle(double *angle);
    bool GetSteerEncoderReading(double *reading);
    bool GetSteerOutput(double *output); // PID, optimize angle
    bool GetSteerError(double *error);
    bool AtSteerSetpoint(bool *at);

    bool SetDistance(double distance);
    bool GetDriveError(double *error);
    bool GetDriveOutput(double *output); // PID

    bool SetDrivePID(double kP, double kI, double kD);
    bool SetSteerPID(double kP, double kI, double kD);

    bool Update(double drive_enc, double steer_enc, double drive_vel, double steer_vel);
    bool Reset();

private:
    std::shared_ptr<frc::PIDController> drive_pid_;
    std::shared_ptr<frc::PIDController> steer_pid_;

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