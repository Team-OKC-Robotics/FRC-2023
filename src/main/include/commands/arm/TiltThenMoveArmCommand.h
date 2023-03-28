#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "commands/intake/IntakeBlockingPositionCommand.h"
#include "commands/arm/ArmSetStateCommand.h"

/**
 * Move arm to a preset position
 */
class TiltThenMoveArmCommand
    : public frc2::CommandHelper<frc2::CommandBase, TiltThenMoveArmCommand> {
public:
    explicit TiltThenMoveArmCommand(std::shared_ptr<IntakeBlockingPositionCommand> move_intake, std::shared_ptr<ArmSetStateCommand> move_arm);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<IntakeBlockingPositionCommand> move_intake_;
    std::shared_ptr<ArmSetStateCommand> move_arm_;

    bool intake_good = false;
    bool done_ = false;
};