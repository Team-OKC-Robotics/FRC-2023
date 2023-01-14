#pragma once

#include <memory>

#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "AHRS.h"
#include "Utils.h"

enum DriveMode {
    ARCADE = 0,
    TANK = 1,
    CURVATURE = 2
};

typedef struct drivetrain_config_t {
    double max_output;
    double open_loop_ramp_rate;
} DrivetrainConfig;

typedef struct drivetrain_hardware_interface_t {
    // Left motors
    rev::CANSparkMax *const left_motor_1;
    rev::CANSparkMax *const left_motor_2;
    rev::CANSparkMax *const left_motor_3;

    // Right motors
    rev::CANSparkMax *const right_motor_1;
    rev::CANSparkMax *const right_motor_2;
    rev::CANSparkMax *const right_motor_3;

    // Drivetrain
    frc::DifferentialDrive *const diff_drive;

    // AHRS
    AHRS *const ahrs;

} DrivetrainHardwareInterface;

typedef struct drivetrain_software_interface_t {
    // SW INPUTS

    // IMU yaw angle
    double imu_yaw;

    // Encoders
    double left_encoder_avg;
    double right_encoder_avg;

    // SW OUTPUTS

    // Configure drivetrain variables
    DrivetrainConfig drive_config;
    bool update_config;

    // Tank or arcade drive selector.
    DriveMode drive_mode;

    // Input squaring
    bool square_inputs;

    // Arcade drive inputs
    double arcade_power;
    double arcade_turn;

    // Tank drive inputs
    double tank_left;
    double tank_right;

    // Curvature drive inputs
    // Note: use arcade drive inputs except for turn_in_place flag.
    bool turn_in_place;

    // Reset flags
    bool reset_encoders;
    bool reset_gyro;

} DrivetrainSoftwareInterface;

class DrivetrainIO : public frc2::SubsystemBase {
public:
    DrivetrainIO(DrivetrainHardwareInterface *hw_interface,
                 DrivetrainSoftwareInterface *sw_interface)
        : hw_interface_(hw_interface), sw_interface_(sw_interface) {}

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs during
     * simulation.
     */
    void SimulationPeriodic() override;

    bool ProcessIO();

private:
    bool UpdateDriveConfig(DrivetrainConfig &config);
    bool ResetEncoders();

    DrivetrainHardwareInterface *const hw_interface_;
    DrivetrainSoftwareInterface *const sw_interface_;
};
Footer
