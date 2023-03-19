#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Joystick.h>
#include "Utils.h"
#include "subsystems/Arm.h"

/**
 * Move arm to a preset position
 */
class ArmSetStateDpadCommand
    : public frc2::CommandHelper<frc2::CommandBase, ArmSetStateDpadCommand> {
public:
    /**
     * Creates a new ArmSetStateCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit ArmSetStateDpadCommand(std::shared_ptr<Arm> arm, std::shared_ptr<frc::Joystick> gamepad);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Arm> arm_;
    std::shared_ptr<frc::Joystick> gamepad_;

    double negative_pickup_rotation_;
    double negative_pickup_extension_;
    double negative_score_mid_rotation_;
    double negative_score_mid_extension_;
    double negative_score_high_rotation_;
    double negative_score_high_extension_;
};