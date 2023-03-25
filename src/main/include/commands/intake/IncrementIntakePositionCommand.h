#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Intake.h"

/**
 *
 */
class IncrementIntakePositionCommand
    : public frc2::CommandHelper<frc2::CommandBase, IncrementIntakePositionCommand> {
public:
    /**
     * Creates a new IncrementArmExtendCommand.
     *
     * @param arm The subsystem used by this command.
     */
    explicit IncrementIntakePositionCommand(std::shared_ptr<Intake> intake, double angle);

    void Execute() override;
    void Initialize() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Intake> intake_;
    double angle_;
};