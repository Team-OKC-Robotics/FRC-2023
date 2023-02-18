#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

#include "commands/arm/SetArmExtensionCommand.h"



SetArmExtensionCommand::SetArmExtensionCommand(std::shared_ptr<Arm> arm,
                                   double extension) {
    // Set everything.
    arm_ = arm;
    extension_ = extension;

    if (arm_ != nullptr) {
        this->AddRequirements(arm_.get());
    }
}

void SetArmExtensionCommand::Execute() {
    VOKC_CHECK(arm_ != nullptr);
    VOKC_CALL(arm_->SetExtend(extension_));
}

bool SetArmExtensionCommand::IsFinished() {
    // Always end this command.
    return true;
}