
#include "commands/swerve/AutoBalanceCommand.h"

AutoBalanceCommand::AutoBalanceCommand(std::shared_ptr<SwerveDrive> swerve) {
    swerve_ = swerve;

    if (swerve_ != nullptr) {
        this->AddRequirements(swerve_.get());
    }
}

void AutoBalanceCommand::Initialize() {
    VOKC_CHECK(swerve_ != nullptr);

    VOKC_CALL(swerve_->ResetPIDs());
    VOKC_CALL(swerve_->SetIdleMode(BRAKE));
}

void AutoBalanceCommand::Execute() {
    VOKC_CALL(swerve_->AutoBalance());
}

void AutoBalanceCommand::End(bool interrupted) {
    // stop the motors
    VOKC_CALL(swerve_->VectorTeleOpDrive(0, 0, 0));
}

bool AutoBalanceCommand::IsFinished() {
    // never finish this command
    return false;
}