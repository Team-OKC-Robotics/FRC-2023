// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <cameraserver/CameraServer.h>

void Robot::RobotInit() {
    frc::CameraServer::StartAutomaticCapture().SetResolution(480, 240);

    m_container.GetArm()->SetControlMode(Auto);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
    m_auto_chooser_ = m_container.GetAutoChooser();
}

void Robot::DisabledPeriodic() {
    m_auto_chooser_->Update();
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    frc::DataLogManager::Start();

    m_container.GetArm()->AllowCalibration();

    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand != nullptr) {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != nullptr) {
        m_autonomousCommand->Cancel();
    }

    frc::DataLogManager::Start();

    teleop_command_ = m_container.GetDriveCommand();
    VOKC_CALL(teleop_command_ != nullptr);
    teleop_command_->Schedule();

    m_container.GetArm()->AllowCalibration();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {
}

void Robot::TestInit() {
    m_container.GetArm()->SetControlMode(Test);

    teleop_command_ = m_container.GetDriveCommand();
    VOKC_CALL(teleop_command_ != nullptr);
    teleop_command_->Schedule();
    VOKC_CHECK_MSG(false, "we are in fact reaching test init");
}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
