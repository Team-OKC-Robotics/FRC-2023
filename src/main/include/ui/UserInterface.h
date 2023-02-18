
#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>

#include "Parameters.h"

namespace SwerveDriveUI {
    // Get the tab
    extern frc::ShuffleboardTab &nt_tab;

    // Add all the defaults
    // Write mode
    extern nt::GenericEntry *const nt_write_mode;

    // Encoder
    extern nt::GenericEntry *const nt_left_avg;
    extern nt::GenericEntry *const nt_right_avg;
    extern nt::GenericEntry *const nt_avg_dist;

    // Steer encoders
    extern nt::GenericEntry *const nt_left_front_front_steer;
    extern nt::GenericEntry *const nt_left_back_front_steer;
    extern nt::GenericEntry *const nt_right_front_front_steer;
    extern nt::GenericEntry *const nt_right_back_front_steer;

    // Distance PID
    extern nt::GenericEntry *const nt_dist_kp;
    extern nt::GenericEntry *const nt_dist_ki;
    extern nt::GenericEntry *const nt_dist_kd;

    // Steer PID
    extern nt::GenericEntry *const nt_steer_kp;
    extern nt::GenericEntry *const nt_steer_ki;
    extern nt::GenericEntry *const nt_steer_kd;

    extern nt::GenericEntry *const nt_left_front_steer_setpoint;
    extern nt::GenericEntry *const nt_left_back_steer_setpoint;
    extern nt::GenericEntry *const nt_right_front_steer_setpoint;
    extern nt::GenericEntry *const nt_right_back_steer_setpoint;

    // Output
    extern nt::GenericEntry *const nt_left_front_steer_output;

    // Gyro
    extern nt::GenericEntry *const nt_heading;
    extern nt::GenericEntry *const nt_reset_gyro;

    // Save drivetrain parameters
    extern nt::GenericEntry *const nt_save;
} // namespace SwerveDriveUI

namespace ArmUI {
    extern frc::ShuffleboardTab &nt_tab;

    extern nt::GenericEntry *const nt_arm_duty_cycle_encoder;
}