#pragma once

#include <frc/Errors.h>

#include <memory>
#include <vector>

#include "Parameters.h"
#include "RobotContainer.h"
#include "Utils.h"

// Hardware
#include "hardware/Hardware.h"

// I/O Subsystems
#include "io/SwerveDriveIO.h"
#include "subsystems/SwerveDrive.h"


// Subsystems
#include "subsystems/SwerveDrive.h"

// Gamepad
#include "ui/GamepadMap.h"
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>

/// Commands
// swerve
#include "commands/swerve/TeleOpSwerveCommand.h"
#include "commands/swerve/AutoSwerveCommand.h"

#include <frc2/command/Command.h>
#include <frc2/command/SubsystemBase.h>

#include "Logging.h"

//subsytems
#include "subsystems/Arm.h"
#include "subsystems/SwerveDrive.h"
#include "subsystems/Claw.h"

//I/O Subsystems

#include "io/ArmIO.h"
#include "io/ClawIO.h"

#include "commands/arm/ManualArmCommand.h"
#include "commands/arm/IncrementArmPresetPositionCommand.h"
#include "commands/arm/SetArmAngleCommand.h"
#include "commands/arm/SetArmExtensionCommand.h"
#include "commands/claw/AutomaticClawCommand.h"
 


/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
public:
    RobotContainer();

    std::shared_ptr<frc2::Command> GetAutonomousCommand();
    std::shared_ptr<frc2::Command> GetDriveCommand();

private:
    // Hardware Initialization
    bool InitHardware(std::unique_ptr<Hardware> &hardware);
    bool InitActuators(Actuators *actuators_interface);
    bool InitSensors(const Actuators &actuators,
                     Sensors *sensor_interface);

    // Command initialization
    bool InitCommands();

    // Gamepad initialization
    bool InitGamepads();
    void ConfigureButtonBindings();

    // subsystem initialization
    bool InitSwerve();
    bool InitArm();

    // Robot Hardware
    std::unique_ptr<Hardware> hardware_;
    std::shared_ptr<SwerveDriveHardwareInterface> swerve_drive_hw_;
    std::shared_ptr<ArmHardwareInterface> arm_hw_;
    std::shared_ptr<ClawHardwareInterface> claw_hw;



    // Hardware I/O interfaces
    std::shared_ptr<SwerveDriveIO> swerve_drive_io_;
    std::shared_ptr<ArmIO> arm_io_;

    // Robot software interfaces.
    std::shared_ptr<SwerveDriveSoftwareInterface> swerve_drive_sw_;
    std::shared_ptr<ArmSoftwareInterface> arm_sw_;
    std::shared_ptr<ClawSoftwareInterface> claw_sw_;

    // Subsystems
    std::shared_ptr<SwerveDrive> swerve_drive_;
    std::shared_ptr<Arm> arm_;
    std::shared_ptr<Claw> claw_;

    /**
     * User interfaces
     * - Gamepads
     * - Joystick Buttons
     */
    std::shared_ptr<frc::Joystick> gamepad1_;
    std::shared_ptr<frc::Joystick> gamepad2_;


    std::shared_ptr<frc2::JoystickButton> driver_a_button_;
    std::shared_ptr<frc2::JoystickButton> driver_b_button_;
    std::shared_ptr<frc2::JoystickButton> driver_back_button_;
    std::shared_ptr<frc2::JoystickButton> driver_left_bumper_;
    std::shared_ptr<frc2::JoystickButton> driver_right_bumper_;

    /**
     * Commands
     */
    std::shared_ptr<AutoSwerveCommand> m_autonomousCommand_;

    // swerve drive
    std::shared_ptr<TeleOpSwerveCommand> swerve_teleop_command_;

    //arm
    std::shared_ptr<ManualArmCommand> manual_arm_command_;

    std::shared_ptr<IncrementArmPresetPositionCommand> increment_arm_preset_command;

    std::shared_ptr<SetArmAngleCommand> set_arm_angle_command;

    std::shared_ptr<SetArmExtensionCommand> set_arm_extension_command; 

    //claw
    std::shared_ptr<AutomaticClawCommand> automatic_claw_command_; 
};


