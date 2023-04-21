#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandHelper.h>
#include "Auton.h"

#include "subsystems/Arm.h"
#include "subsystems/SwerveDrive.h"
#include "subsystems/Intake.h"


class ScoreConeDriveAuto : public Auton {
public:
    explicit ScoreConeDriveAuto(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<Arm> arm, std::shared_ptr<Intake> intake);
private:
    
};