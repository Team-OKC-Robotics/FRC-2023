#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/SwerveDrive.h"


/**
 * 
 */
class ResetGyroCommand
    : public frc2::CommandHelper<frc2::CommandBase, ResetGyroCommand> {
public:
    /**
     * @param swerve The swerve drive subsystem used by this command.
     */
    explicit ResetGyroCommand(std::shared_ptr<SwerveDrive> swerve);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<SwerveDrive> swerve_;
};