#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

/**
 * Move arm to a preset position
 */
class ArmPresetCommand
    : public frc2::CommandHelper<frc2::CommandBase, ArmPresetCommand> {
public:
    /**
     * Creates a new ArmPresetCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit ArmPresetCommand(std::shared_ptr<Arm> arm, double degrees, double extend);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Arm> arm_;
    double degrees_;
    double extend_;
    double arm_at_setpoint_;
    bool done_ = false;
};