#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Intake.h"

/**
 *
 */
class IntakeCommand
    : public frc2::CommandHelper<frc2::CommandBase, IntakeCommand> {
public:
    /**
     * Creates a new IncrementArmExtendCommand.
     *
     * @param arm The subsystem used by this command.
     */
    explicit IntakeCommand(std::shared_ptr<Intake> intake, double degrees);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Intake> intake_;
    double degrees_;
    double turn_;
};