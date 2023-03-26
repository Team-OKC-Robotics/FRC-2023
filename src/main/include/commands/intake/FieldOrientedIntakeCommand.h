#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Intake.h"
#include "subsystems/SwerveDrive.h"

/**
 *
 */
class FieldOrientedIntakeCommand
    : public frc2::CommandHelper<frc2::CommandBase, FieldOrientedIntakeCommand> {
public:
    /**
     * Creates a new IncrementArmExtendCommand.
     *
     * @param arm The subsystem used by this command.
     */
    explicit FieldOrientedIntakeCommand(std::shared_ptr<Intake> intake, std::shared_ptr<SwerveDrive> swerve, double angle, double reverse_angle);

    void Execute() override;
    void Initialize() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Intake> intake_;
    std::shared_ptr<SwerveDrive> swerve_drive_;
    double angle_;
    double reverse_angle_;
};