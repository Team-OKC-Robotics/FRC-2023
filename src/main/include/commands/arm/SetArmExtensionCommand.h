#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

/**
 *
 */
class SetArmExtensionCommand
    : public frc2::CommandHelper<frc2::CommandBase, SetArmExtensionCommand> {
public:
    /**
     * Creates a new SetArmExtensionCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit SetArmExtensionCommand(std::shared_ptr<Arm> arm, double extension);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Arm> arm_;
    double extension_;
};