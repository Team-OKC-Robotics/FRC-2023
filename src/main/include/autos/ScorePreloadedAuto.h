#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Arm.h"
#include "subsystems/SwerveDrive.h"
#include "subsystems/Claw.h"


class ScorePreloadedAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, ScorePreloadedAuto> {
public:
    explicit ScorePreloadedAuto(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<Arm> arm, std::shared_ptr<Claw> claw);
private:
    
};