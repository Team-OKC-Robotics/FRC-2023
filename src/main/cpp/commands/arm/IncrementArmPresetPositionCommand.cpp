

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

#include "commands/arm/IncrementArmPresetPositionCommand.h"

IncrementArmPresetPositionCommand::IncrementArmPresetPositionCommand(std::shared_ptr<Arm> arm, double increment) {
    // Set everything.
    arm_ = arm;
    increment_ = increment;

    if (arm_ != nullptr) {
        this->AddRequirements(arm_.get());
    }
}

void IncrementArmPresetPositionCommand::Initialize() {
    arm_->SetControlMode(Test);
}

void IncrementArmPresetPositionCommand::Execute() {
    VOKC_CHECK(arm_ != nullptr);
    VOKC_CALL(arm_->IncrementRotation(increment_));
}

bool IncrementArmPresetPositionCommand::IsFinished() {
    // Always end this command.
    return true;
}