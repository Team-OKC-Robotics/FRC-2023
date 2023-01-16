// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

 
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}

//Initialize the joystick buttons 
    driver_a_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), A_BUTTON);
    driver_b_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), B_BUTTON);
    driver_back_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), BACK_BUTTON);
    driver_left_bumper_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), LEFT_BUMPER);
    driver_right_bumper_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), RIGHT_BUMPER);


    // Arm
    manip_a_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), A_BUTTON);
    manip_b_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), B_BUTTON);
    manip_back_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), BACK_BUTTON);
    manip_start_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), START_BUTTON);
    manip_left_stick_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), LEFT_STICK_BUTTON);
    manip_right_stick_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), RIGHT_STICK_BUTTON);

    // 
    manip_trigger_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), TRIGGER_BUTTON);
    manip_left_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), LEFT_BUTTON);
    manip_bottom_right_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), BOTTOM_RIGHT_BUTTON);
    manip_top_right_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), TOP_RIGHT_BUTTON);
    manip_bottom_left_button_ = std::make_shared<frc2::JoystickButton>(
        gamepad2_.get(), BOTTOM_LEFT_BUTTON);
    manip_top_left_button_ = std::make_shared<frc2::JoystickButton>(
        gamepad2_.get(), TOP_LEFT_BUTTON);

    manip_seven_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), SEVEN_BUTTON);
    manip_eight_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), EIGHT_BUTTON);
    manip_nine_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), NINE_BUTTON);
    manip_ten_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), TEN_BUTTON);
    manip_eleven_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), ELEVEN_BUTTON);
    manip_twelve_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), TWELVE_BUTTON);

    manip_A_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), A_BUTTON);
    manip_B_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), B_BUTTON);
    manip_X_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), X_BUTTON);
    manip_Y_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), Y_BUTTON);
    manip_RB_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), RB_BUTTON);
    manip_LB_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), LB_BUTTON);
    manip_RT_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), RT_BUTTON);
    manip_LT_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), LT_BUTTON);
    manip_back_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), BACK_BUTTON);
    manip_start_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), START_BUTTON);
    manip_mode_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), MODE_BUTTON);