#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Utils.h"
#include "subsystems/Claw.h"

class ManualClawCommand
    : public frc2::CommandHelper<frc2::CommandBase, ManualClawCommand> {
public:
    explicit ManualClawCommand(std::shared_ptr<Claw> claw, double power);

    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<Claw> claw_;
    double power_;
};