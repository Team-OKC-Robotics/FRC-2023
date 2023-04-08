#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Intake.h"

/**
 *
 */
class IntakeBlockingPositionCommand
    : public frc2::CommandHelper<frc2::CommandBase, IntakeBlockingPositionCommand> {
public:
    /**
     * Creates a new IncrementArmExtendCommand.
     *
     * @param arm The subsystem used by this command.
     */
    explicit IntakeBlockingPositionCommand(std::shared_ptr<Intake> intake, double angle);

    void Execute() override;
    void Initialize() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Intake> intake_;
    double angle_;
};