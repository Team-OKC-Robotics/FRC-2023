#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/SwerveDrive.h"


/**
 *
 */
class AutoTurnCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoTurnCommand> {
public:
    /**
     * Creates a new AutoSwerveCommand.
     * Should be the only autonomous swerve-drive-related command we need
     *
     * @param swerve The swerve drive used by this command.
     */
    explicit AutoTurnCommand(std::shared_ptr<SwerveDrive> swerve, double dist, double max_speed, double strafe);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<SwerveDrive> swerve_;
    double dist_;
    double max_speed_;
    double strafe_;
};
