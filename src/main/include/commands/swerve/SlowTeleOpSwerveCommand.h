#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>

#include "Utils.h"
#include "subsystems/SwerveDrive.h"


/**
 * 
 */
class SlowTeleOpSwerveCommand
    : public frc2::CommandHelper<frc2::CommandBase, SlowTeleOpSwerveCommand> {
public:
    /**
     * Creates a new TeleOpSwerveCommand.
     *
     * @param swerve The swerve drive subsystem used by this command.
     */
    explicit SlowTeleOpSwerveCommand(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<frc::Joystick> gamepad);

    void Initialize() override;
    void Execute() override;
    void End(bool executed) override;
    bool IsFinished() override;

private:
    std::shared_ptr<SwerveDrive> swerve_;
    std::shared_ptr<frc::Joystick> gamepad_;
};
