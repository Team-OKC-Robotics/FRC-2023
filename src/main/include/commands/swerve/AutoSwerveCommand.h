#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/SwerveDrive.h"


/**
 *
 */
class AutoSwerveCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoSwerveCommand> {
public:
    /**
     * Creates a new AutoSwerveCommand.
     * Should be the only autonomous swerve-drive-related command we need
     *
     * @param swerve The swerve drive used by this command.
     */
    explicit AutoSwerveCommand(std::shared_ptr<SwerveDrive> swerve, frc::Pose2d end_pos, bool keep_heading);

    void Initialize() override;
    void Execute() override;
    void End(bool executed) override;
    bool IsFinished() override;

private:
    std::shared_ptr<SwerveDrive> swerve_;
    frc::Pose2d end_pos_;
    bool keep_heading_ = false;
};
