#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Arm.h"
#include "subsystems/SwerveDrive.h"
#include "subsystems/Intake.h"

#include <stdio.h>

class Auton : public frc2::CommandHelper<frc2::SequentialCommandGroup, Auton> {
public:
    bool SetName(std::string name);
    std::string GetAutonName();
private:
    std::string name_;
};
