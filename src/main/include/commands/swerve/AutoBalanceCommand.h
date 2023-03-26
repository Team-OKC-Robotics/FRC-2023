#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/SwerveDrive.h"


/**
 *
 */
class AutoBalanceCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoBalanceCommand> {
public:
    explicit AutoBalanceCommand(std::shared_ptr<SwerveDrive> swerve, double sign);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<SwerveDrive> swerve_;
    double sign_;
};
