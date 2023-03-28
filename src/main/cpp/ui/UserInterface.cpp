#include "ui/UserInterface.h"

namespace AutonUI {
    frc::ShuffleboardTab &nt_tab = frc::Shuffleboard::GetTab("Autos");

    nt::GenericEntry *const nt_auton_name = nt_tab.Add("current auton", "no auto selected").GetEntry();
}

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

    nt::GenericEntry *const nt_left_front_steer_setpoint = nt_tab.Add("left front setpoint", 0.0).GetEntry();
    nt::GenericEntry *const nt_left_back_steer_setpoint = nt_tab.Add("left back setpoint", 0.0).GetEntry();
    nt::GenericEntry *const nt_right_front_steer_setpoint = nt_tab.Add("right front setpoint", 0.0).GetEntry();
    nt::GenericEntry *const nt_right_back_steer_setpoint = nt_tab.Add("right back setpoint", 0.0).GetEntry();

    // output
    nt::GenericEntry *const nt_left_front_steer_output = nt_tab.Add("l f steer output", 0.0).GetEntry();

    // Gyro
    nt::GenericEntry *const nt_heading = nt_tab.Add("Heading", 0.0).GetEntry();
    nt::GenericEntry *const nt_pitch = nt_tab.Add("Pitch", 0.0).GetEntry();
    nt::GenericEntry *const nt_reset_gyro =
        nt_tab.Add("Reset Gyro", false).GetEntry();

    // Save parameters button
    nt::GenericEntry *const nt_save = nt_tab.Add("Save", false).GetEntry();
} // namespace SwerveDriveUI

namespace ArmUI {
    // Get the tab
    frc::ShuffleboardTab &nt_tab = frc::Shuffleboard::GetTab("Arm");
    
    nt::GenericEntry *const nt_arm_duty_cycle_encoder = nt_tab.Add("Arm Duty Cycles Encoder", 0.0).GetEntry();
    nt::GenericEntry *const nt_arm_setpoint = nt_tab.Add("arm setpoint", 0.0).GetEntry();
    nt::GenericEntry *const nt_arm_power = nt_tab.Add("arm power", 0.0).GetEntry();

    nt::GenericEntry *const nt_extend_encoder = nt_tab.Add("extend encodoer", 0.0).GetEntry();
    nt::GenericEntry *const nt_extend_setpoint = nt_tab.Add("extend setpoint", 0.0).GetEntry();
    nt::GenericEntry *const nt_extend_power = nt_tab.Add("extend power", 0.0).GetEntry();

    nt::GenericEntry *const nt_limit_switch = nt_tab.Add("extend limit switch", false).GetEntry();

    nt::GenericEntry *const arm_control_state = nt_tab.Add("arm control state", "init").GetEntry();
}

namespace ClawUI {
    frc::ShuffleboardTab &nt_tab = frc::Shuffleboard::GetTab("claw");

    nt::GenericEntry *const nt_claw_encoder = nt_tab.Add("claw encoder", 0.0).GetEntry();
}

namespace IntakeUI {
    frc::ShuffleboardTab &nt_tab = frc::Shuffleboard::GetTab("intake");

    nt::GenericEntry *const nt_tilt = nt_tab.Add("wrist encoder", 0.0).GetEntry();
    nt::GenericEntry *const nt_tilt_setpoint = nt_tab.Add("wrist setpoint", 0.0).GetEntry();
}