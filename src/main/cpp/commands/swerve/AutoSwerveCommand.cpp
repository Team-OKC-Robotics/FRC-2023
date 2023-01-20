
#include "commands/swerve/AutoSwerveCommand.h"

AutoSwerveCommand::AutoSwerveCommand(std::shared_ptr<SwerveDrive> swerve, frc::Pose2d f_pos, bool keep_init_heading) {
    swerve_ = swerve;
    end_pos = f_pos;
    keep_heading = keep_init_heading;

    if (swerve_ != nullptr) {
        this->AddRequirements(swerve_.get());
    }
}

void AutoSwerveCommand::Initialize() {
    VOKC_CHECK(swerve_ != nullptr);

    VOKC_CALL(swerve_->ResetDriveEncoders());
    VOKC_CALL(swerve_->ResetSteerEncoders());
    VOKC_CALL(swerve_->ResetPIDs());

    VOKC_CALL(swerve_->InitAuto(end_pos, keep_heading));
}

void AutoSwerveCommand::Execute() {
    VOKC_CHECK(swerve_ != nullptr);

    VOKC_CALL(swerve_->RunAuto());
}

void AutoSwerveCommand::End(bool executed) {
    //TODO
    return;
}

bool AutoSwerveCommand::IsFinished() {
    // if the swerve subsystem doesn't exist
    if (swerve_ == nullptr) {
        return true; // then don't try to do anything on it because it will fail
    }

    bool *at;

    // if the swerve drive is at its setpoint
    if (swerve_->AtSetpoint(at)) {
        return true; // end the command
    }

    // otherwise, continue
    return false;
}
