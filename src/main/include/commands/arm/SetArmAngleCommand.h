#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

/**
 *
 */
class SetArmAngleCommand
    : public frc2::CommandHelper<frc2::CommandBase, SetArmAngleCommand> {
public:
    /**
     * Creates a new SetArmAngleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit SetArmAngleCommand(std::shared_ptr<Arm> arm, double degrees);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Arm> arm_;
    double degrees_;
};
