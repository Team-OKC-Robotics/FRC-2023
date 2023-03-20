#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>

#include "Utils.h"
#include "subsystems/SwerveDrive.h"


/**
 * 
 */
class TeleOpSwerveCommand
    : public frc2::CommandHelper<frc2::CommandBase, TeleOpSwerveCommand> {
public:
    /**
     * Creates a new TeleOpSwerveCommand.
     *
     * @param swerve The swerve drive subsystem used by this command.
     */
    explicit TeleOpSwerveCommand(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<frc::Joystick> gamepad, double speed_mod, double open_loop_ramp_rate, bool brake_mode);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<SwerveDrive> swerve_;
    std::shared_ptr<frc::Joystick> gamepad_;

    double speed_mod_;
    double open_loop_ramp_rate_;
    bool brake_mode_;
};
