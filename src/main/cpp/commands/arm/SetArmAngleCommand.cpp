#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

#include "commands/arm/SetArmAngleCommand.h"



SetArmAngleCommand::SetArmAngleCommand(std::shared_ptr<Arm> arm, double degrees) {
    // Set everything.
    arm_ = arm;
    degrees_ = degrees;

    if (arm_ != nullptr) {
        this->AddRequirements(arm_.get());

        arm_->SetControlMode(Auto);
    }
}

void SetArmAngleCommand::Execute() {
    VOKC_CHECK(arm_ != nullptr);
    VOKC_CALL(arm_->SetDegrees(degrees_));
}

bool SetArmAngleCommand::IsFinished() {
    // Always end this command.
    return true;
}