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
#include "io/ArmIO.h"
#include "io/IntakeIO.h"

#include "AutoChooser.h"


// Subsystems
#include "subsystems/SwerveDrive.h"
#include "subsystems/Arm.h"
#include "subsystems/Intake.h"

// Gamepad
#include "ui/GamepadMap.h"
#include <frc/Joystick.h>
#include "frc2/command/button/JoystickButton.h"


/// Commands
// swerve
#include "commands/swerve/TeleOpSwerveCommand.h"
#include "commands/swerve/AutoDriveCommand.h"

// arm
#include "commands/arm/IncrementArmPresetPositionCommand.h"
#include "commands/arm/IncrementArmExtendCommand.h"
#include "commands/arm/ArmSetStateCommand.h"
#include "commands/arm/ArmSetStateDpadCommand.h"


//intake
#include "commands/intake/IntakeCommand.h"

// misc
#include <frc2/command/Command.h>
#include <frc2/command/SubsystemBase.h>

// autos
#include "autos/ScorePreloadedAuto.h"

#include "Logging.h"

#include "units/length.h"
#include "units/velocity.h"
#include "units/math.h"
#include "units/voltage.h"

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
    // std::shared_ptr<frc2::Command> GetSlowDriveCommand();
    // std::shared_ptr<frc2::Command> GetBoostDriveCommand();
    std::shared_ptr<Arm> GetArm();

private:
    // AutoChooser m_chooser_;

    // Hardware Initialization
    bool InitHardware(std::unique_ptr<Hardware> &hardware);
    bool InitActuators(Actuators *actuators_interface);
    bool InitSensors(const Actuators &actuators,
                     Sensors *sensor_interface);

    // Command initialization
    bool InitCommands();

    // Gamepad initialization
    bool InitGamepads();
    bool ConfigureButtonBindings();

    // subsystem initialization
    bool InitSwerve();
    bool InitArm();
    bool InitIntake();

    // Robot Hardware
    std::unique_ptr<Hardware> hardware_;
    std::shared_ptr<SwerveDriveHardwareInterface> swerve_drive_hw_;
    std::shared_ptr<ArmHardwareInterface> arm_hw_;
    std::shared_ptr<IntakeHardwareInterface> intake_hw_;



    // Hardware I/O interfaces
    std::shared_ptr<SwerveDriveIO> swerve_drive_io_;
    std::shared_ptr<ArmIO> arm_io_;
    std::shared_ptr<IntakeIO> intake_io_;

    // Robot software interfaces.
    std::shared_ptr<SwerveDriveSoftwareInterface> swerve_drive_sw_;
    std::shared_ptr<ArmSoftwareInterface> arm_sw_;
    std::shared_ptr<IntakeSoftwareInterface> intake_sw_;

    // Subsystems
    std::shared_ptr<SwerveDrive> swerve_drive_;
    std::shared_ptr<Arm> arm_;
    std::shared_ptr<Intake> intake_;

    /**
     * User interfaces
     * - Gamepads
     * - Joystick Buttons
     */
    std::shared_ptr<frc::Joystick> gamepad1_;
    std::shared_ptr<frc::Joystick> gamepad2_;

    std::shared_ptr<frc2::JoystickButton> driver_a_button_;
    std::shared_ptr<frc2::JoystickButton> driver_b_button_;
    std::shared_ptr<frc2::JoystickButton> driver_x_button_;
    std::shared_ptr<frc2::JoystickButton> driver_y_button_;
    std::shared_ptr<frc2::JoystickButton> driver_start_button_;
    std::shared_ptr<frc2::JoystickButton> driver_back_button_;
    std::shared_ptr<frc2::JoystickButton> driver_left_stick_button_;
    std::shared_ptr<frc2::JoystickButton> driver_right_stick_button_;
    std::shared_ptr<frc2::JoystickButton> driver_left_bumper_;
    std::shared_ptr<frc2::JoystickButton> driver_right_bumper_;

    std::shared_ptr<frc2::JoystickButton> manip_a_button_;
    std::shared_ptr<frc2::JoystickButton> manip_b_button_;
    std::shared_ptr<frc2::JoystickButton> manip_x_button_;
    std::shared_ptr<frc2::JoystickButton> manip_y_button_;
    std::shared_ptr<frc2::JoystickButton> manip_back_button_;
    std::shared_ptr<frc2::JoystickButton> manip_start_button_;
    std::shared_ptr<frc2::JoystickButton> manip_left_bumper_button_;
    std::shared_ptr<frc2::JoystickButton> manip_right_bumper_button_;
    std::shared_ptr<frc2::JoystickButton> manip_left_stick_button_;
    std::shared_ptr<frc2::JoystickButton> manip_right_stick_button_;

    /**
     * Commands
     */
    std::shared_ptr<ScorePreloadedAuto> m_autonomousCommand_;

    // swerve drive
    std::shared_ptr<TeleOpSwerveCommand> swerve_teleop_command_;
    std::shared_ptr<TeleOpSwerveCommand> slow_swerve_teleop_command_;
    std::shared_ptr<TeleOpSwerveCommand> fast_swerve_teleop_command_;

    //arm
    std::shared_ptr<IncrementArmExtendCommand> extendArmCommand;
    std::shared_ptr<IncrementArmExtendCommand> retractArmCommand;
    std::shared_ptr<IncrementArmPresetPositionCommand> raiseArmCommand;
    std::shared_ptr<IncrementArmPresetPositionCommand> lowerArmCommand;

    std::shared_ptr<ArmSetStateCommand> arm_carry_command_;
    std::shared_ptr<ArmSetStateCommand> arm_pickup_command_;
    std::shared_ptr<ArmSetStateCommand> arm_score_mid_command_;
    std::shared_ptr<ArmSetStateCommand> arm_score_high_command_;
    std::shared_ptr<ArmSetStateCommand> arm_short_carry_command_;

    std::shared_ptr<ArmSetStateDpadCommand> arm_dpad_set_state_command_;

    //intake
    std::shared_ptr<IntakeCommand> intake_command;
    std::shared_ptr<IntakeCommand> other_intake_command;
    std::shared_ptr<IntakeCommand> stop_intake_command;
};


