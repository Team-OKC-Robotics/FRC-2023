// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>


#include "RobotContainer.h"
#include "AutoChooser.h"


 //Arm hardware 






class Robot : public frc::TimedRobot {
public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;
    void TestInit() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

private:
    // Have it null by default so that if testing teleop it
    // doesn't have undefined behavior and potentially crash.
    std::shared_ptr<frc2::Command> m_autonomousCommand = nullptr;
    std::shared_ptr<frc2::Command> teleop_command_ = nullptr;
    std::shared_ptr<AutoChooserTeamOKC> m_auto_chooser_;

    RobotContainer m_container;
};
