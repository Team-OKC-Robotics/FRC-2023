// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

#include "commands/ExampleCommand.h"
#include "subsystems/ExampleSubsystem.h"

#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>

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

  frc2::Command* GetAutonomousCommand();

 private:
  // The robot's subsystems and commands are defined here...
  ExampleSubsystem m_subsystem;
  ExampleCommand m_autonomousCommand;

  void ConfigureButtonBindings();

      std::shared_ptr<frc::Joystick> gamepad1_;
    std::shared_ptr<frc::Joystick> gamepad2_;

    std::shared_ptr<frc2::JoystickButton> driver_A_button_;
    std::shared_ptr<frc2::JoystickButton> driver_B_button_;
    std::shared_ptr<frc2::JoystickButton> driver_X_button_;
    std::shared_ptr<frc2::JoystickButton> driver_Y_button_;
    std::shared_ptr<frc2::JoystickButton> driver_RB_button_;
    std::shared_ptr<frc2::JoystickButton> driver_LB_button_;
    std::shared_ptr<frc2::JoystickButton> driver_RT_button_;
    std::shared_ptr<frc2::JoystickButton> driver_LT_button_;
    std::shared_ptr<frc2::JoystickButton> driver_back_button_;
    std::shared_ptr<frc2::JoystickButton> driver_start_button_;
    std::shared_ptr<frc2::JoystickButton> driver_mode_button_;

    std::shared_ptr<frc2::JoystickButton> manip_a_button_;
    std::shared_ptr<frc2::JoystickButton> manip_b_button_;
    std::shared_ptr<frc2::JoystickButton> manip_back_button_;
    std::shared_ptr<frc2::JoystickButton> manip_start_button_;
    std::shared_ptr<frc2::JoystickButton> manip_left_stick_button_;
    std::shared_ptr<frc2::JoystickButton> manip_right_stick_button;

    std::shared_ptr<frc2::JoystickButton> manip_trigger_button_;
    std::shared_ptr<frc2::JoystickButton> manip_left_button_;
    std::shared_ptr<frc2::JoystickButton> manip_bottom_right_button_;
    std::shared_ptr<frc2::JoystickButton> manip_top_right_button_;
    std::shared_ptr<frc2::JoystickButton> manip_botton_left_button_;
    std::shared_ptr<frc2::JoystickButton> manip_top_left_button_;

    std::shared_ptr<frc2::JoystickButton> manip_seven_button_;
    std::shared_ptr<frc2::JoystickButton> manip_eight_button_;
    std::shared_ptr<frc2::JoystickButton> manip_nine_button_;
    std::shared_ptr<frc2::JoystickButton> manip_ten_button_;
    std::shared_ptr<frc2::JoystickButton> manip_eleven_button_;
    std::shared_ptr<frc2::JoystickButton> manip_twelve_button_;


};


