#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

/**
 *
 */
class IncrementArmPresetPositionCommand
    : public frc2::CommandHelper<frc2::CommandBase, IncrementArmPresetPositionCommand> {
public:
    /**
     * Creates a new IncrementArmPresetPositionCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit IncrementArmPresetPositionCommand(std::shared_ptr<Arm> arm, double increment);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Arm> arm_;
    double increment_;
};