#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"
#include "subsystems/SwerveDrive.h"

class ArmFieldOrientedCommand
    : public frc2::CommandHelper<frc2::CommandBase, ArmFieldOrientedCommand> {
public:
    /**
     * Creates a new ArmSetStateCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit ArmFieldOrientedCommand(std::shared_ptr<Arm> arm, std::shared_ptr<SwerveDrive> swerve, TeamOKC::ArmState state, TeamOKC::ArmState reverse_state);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Arm> arm_;
    std::shared_ptr<SwerveDrive> swerve_drive_;
    TeamOKC::ArmState state_;
    TeamOKC::ArmState reverse_state_;
};