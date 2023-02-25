#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

/**
 *
 */
class IncrementArmExtendCommand
    : public frc2::CommandHelper<frc2::CommandBase, IncrementArmExtendCommand> {
public:
    /**
     * Creates a new IncrementArmExtendCommand.
     *
     * @param arm The subsystem used by this command.
     */
    explicit IncrementArmExtendCommand(std::shared_ptr<Arm> arm, double extend);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Arm> arm_;
    double extend_;
};