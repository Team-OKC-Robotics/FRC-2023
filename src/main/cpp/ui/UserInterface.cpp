#include "ui/UserInterface.h"

namespace SwerveDriveUI {
    // Get the tab
    frc::ShuffleboardTab &nt_tab = frc::Shuffleboard::GetTab("SwerveDrive");

    // Add all the defaults
    // Write mode
    nt::GenericEntry *const nt_write_mode =
        nt_tab.Add("Write Mode", false).GetEntry();

    // Encoder
    nt::GenericEntry *const nt_left_avg = nt_tab.Add("left avg dist", 0.0).GetEntry();
    nt::GenericEntry *const nt_right_avg = nt_tab.Add("right avg dist", 0.0).GetEntry();
    nt::GenericEntry *const nt_avg_dist = nt_tab.Add("avg dist", 0.0).GetEntry();

    // Steer encoders
    nt::GenericEntry *const nt_left_front_front_steer = nt_tab.Add("left front steer", 0.0).GetEntry();
    nt::GenericEntry *const nt_left_back_front_steer = nt_tab.Add("left back steer", 0.0).GetEntry();
    nt::GenericEntry *const nt_right_front_front_steer = nt_tab.Add("right front steer", 0.0).GetEntry();
    nt::GenericEntry *const nt_right_back_front_steer = nt_tab.Add("right back steer", 0.0).GetEntry();

    // Distance PID
    nt::GenericEntry *const nt_dist_kp = nt_tab.Add("Distance kP", 0.0).GetEntry();
    nt::GenericEntry *const nt_dist_ki = nt_tab.Add("Distance kI", 0.0).GetEntry();
    nt::GenericEntry *const nt_dist_kd = nt_tab.Add("Distance kD", 0.0).GetEntry();

    // Steer PID
    nt::GenericEntry *const nt_steer_kp = nt_tab.Add("Steer kP", 0.0).GetEntry();
    nt::GenericEntry *const nt_steer_ki = nt_tab.Add("Steer kI", 0.0).GetEntry();
    nt::GenericEntry *const nt_steer_kd = nt_tab.Add("Steer kD", 0.0).GetEntry();

    nt::GenericEntry *const nt_left_front_steer_setpoint = nt_tab.Add("left front setpoint", 0.0).GetEntry();
    nt::GenericEntry *const nt_left_back_steer_setpoint = nt_tab.Add("left back setpoint", 0.0).GetEntry();
    nt::GenericEntry *const nt_right_front_steer_setpoint = nt_tab.Add("right front setpoint", 0.0).GetEntry();
    nt::GenericEntry *const nt_right_back_steer_setpoint = nt_tab.Add("right back setpoint", 0.0).GetEntry();

    // output
    nt::GenericEntry *const nt_left_front_steer_output = nt_tab.Add("l f steer output", 0.0).GetEntry();

    // Gyro
    nt::GenericEntry *const nt_heading = nt_tab.Add("Heading", 0.0).GetEntry();
    nt::GenericEntry *const nt_reset_gyro =
        nt_tab.Add("Reset Gyro", false).GetEntry();

    // Save parameters button
    nt::GenericEntry *const nt_save = nt_tab.Add("Save", false).GetEntry();
} // namespace SwerveDriveUI

namespace ArmUI {
    // Get the tab
    frc::ShuffleboardTab &nt_tab = frc::Shuffleboard::GetTab("Arm");
    

nt::GenericEntry *const nt_arm_duty_cycle_encoder = nt_tab.Add("Arm Duty Cycles Encoder", 0.0).GetEntry();
    
}