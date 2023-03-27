#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandHelper.h>

#include "commands/intake/IntakeBlockingPositionCommand.h"
#include "commands/arm/ArmSetStateCommand.h"

/**
 * Move arm to a preset position
 */
class TiltThenMoveArmCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, TiltThenMoveArmCommand> {
public:
    explicit TiltThenMoveArmCommand(std::shared_ptr<IntakeBlockingPositionCommand> move_intake, std::shared_ptr<ArmSetStateCommand> move_arm);

private:
};