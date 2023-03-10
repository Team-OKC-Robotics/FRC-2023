#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/SwerveDrive.h"


/**
 *
 */
class AutoDriveCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoDriveCommand> {
public:
    /**
     * Creates a new AutoSwerveCommand.
     * Should be the only autonomous swerve-drive-related command we need
     *
     * @param swerve The swerve drive used by this command.
     */
    explicit AutoDriveCommand(std::shared_ptr<SwerveDrive> swerve, double dist, double max_speed);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<SwerveDrive> swerve_;
    double dist_;
    double max_speed_;
};
