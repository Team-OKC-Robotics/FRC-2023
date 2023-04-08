#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

/**
 * Move arm to a preset position
 */
class WaitForArmCommand
    : public frc2::CommandHelper<frc2::CommandBase, WaitForArmCommand> {
public:
    /**
     * @param subsystem The subsystem used by this command.
     */
    explicit WaitForArmCommand(std::shared_ptr<Arm> arm);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Arm> arm_; 
};