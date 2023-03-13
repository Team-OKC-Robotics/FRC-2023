#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

/**
 * Move arm to a preset position
 */
class ArmSetStateCommand
    : public frc2::CommandHelper<frc2::CommandBase, ArmSetStateCommand> {
public:
    /**
     * Creates a new ArmSetStateCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit ArmSetStateCommand(std::shared_ptr<Arm> arm, TeamOKC::ArmState state);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Arm> arm_;
    TeamOKC::ArmState state_;
};