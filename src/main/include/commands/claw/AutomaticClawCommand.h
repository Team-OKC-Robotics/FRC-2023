#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Claw.h"


/**
 *
 */
class AutomaticClawCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutomaticClawCommand> {
public:
    /**
     * Creates a new SetArmExtensionCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit AutomaticClawCommand(std::shared_ptr<Claw> claw, double distance);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Claw> claw_;
    double distance_;
};