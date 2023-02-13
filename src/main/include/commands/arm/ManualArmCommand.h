#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>

#include "Utils.h"
#include "subsystems/Arm.h"


/**
 * 
 */
class ManualArmCommand
    : public frc2::CommandHelper<frc2::CommandBase, ManualArmCommand> {
public:
    /**
     * Creates a new ManualArmCommand.
     *
     * @param arm The arm drive subsystem used by this command.
     */
    explicit ManualArmCommand(std::shared_ptr<Arm> arm, std::shared_ptr<frc::Joystick> gamepad);

    void Initialize() override;
    void Execute() override;
    void End(bool executed) override;
    bool IsFinished() override;

private:
    std::shared_ptr<Arm> arm_;
    std::shared_ptr<frc::Joystick> gamepad_;
};
