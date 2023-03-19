#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/SwerveDrive.h"


/**
 *
 */
class LockWheelsCommand
    : public frc2::CommandHelper<frc2::CommandBase, LockWheelsCommand> {
public:
    /**
     * Creates a new AutoSwerveCommand.
     * Should be the only autonomous swerve-drive-related command we need
     *
     * @param swerve The swerve drive used by this command.
     */
    explicit LockWheelsCommand(std::shared_ptr<SwerveDrive> swerve);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<SwerveDrive> swerve_;
};
