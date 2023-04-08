
#include "commands/swerve/ResetGyroCommand.h"

ResetGyroCommand::ResetGyroCommand(std::shared_ptr<SwerveDrive> swerve) {
    swerve_ = swerve;
}


void ResetGyroCommand::Execute() {
    VOKC_CHECK(swerve_ != nullptr);

    VOKC_CALL(swerve_->ResetGyro());
}

bool ResetGyroCommand::IsFinished() {
    // always end this command
    return true;
}
