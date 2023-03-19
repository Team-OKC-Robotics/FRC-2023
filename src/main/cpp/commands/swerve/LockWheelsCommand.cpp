
#include "commands/swerve/LockWheelsCommand.h"

LockWheelsCommand::LockWheelsCommand(std::shared_ptr<SwerveDrive> swerve) {
    swerve_ = swerve;

    if (swerve_ != nullptr) {
        this->AddRequirements(swerve_.get());
    }
}

void LockWheelsCommand::Initialize() {
    VOKC_CHECK(swerve_ != nullptr);

    VOKC_CALL(swerve_->SetIdleMode(BRAKE));
}

void LockWheelsCommand::Execute() {
    // VOKC_CALL(swerve_->LockWheels());
}

void LockWheelsCommand::End(bool interrupted) {
    // stop the motors
}

bool LockWheelsCommand::IsFinished() {
    return true;
}