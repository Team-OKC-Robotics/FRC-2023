#include "commands/arm/SetArmAngleCommand.h"

SetArmAngleCommand::SetArmAngleCommand(std::shared_ptr<Arm> arm,
                                   double degrees) {
    // Set everything.
    arm_ = arm;
    degrees_ = degrees;

    if (arm_ != nullptr) {
        this->AddRequirements(arm_.get());
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