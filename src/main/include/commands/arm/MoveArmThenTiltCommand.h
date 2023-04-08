#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "commands/intake/IntakeBlockingPositionCommand.h"
#include "commands/arm/ArmSetStateCommand.h"

/**
 * Move arm to a preset position
 */
class MoveArmThenTiltCommand
    : public frc2::CommandHelper<frc2::CommandBase, MoveArmThenTiltCommand> {
public:
    explicit MoveArmThenTiltCommand(std::shared_ptr<frc2::Command> move_intake, std::shared_ptr<frc2::Command> move_arm);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<frc2::Command> move_intake_;
    std::shared_ptr<frc2::Command> move_arm_;

    bool arm_good_ = false;
    bool done_ = false;

};