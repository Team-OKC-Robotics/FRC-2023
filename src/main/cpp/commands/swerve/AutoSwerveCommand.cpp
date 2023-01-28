
#include "commands/swerve/AutoSwerveCommand.h"

AutoSwerveCommand::AutoSwerveCommand(std::shared_ptr<SwerveDrive> swerve, frc::Pose2d end_pos, bool keep_heading) {
    swerve_ = swerve;
    end_pos_ = end_pos;
    keep_heading_ = keep_heading;

    if (swerve_ != nullptr) {
        this->AddRequirements(swerve_.get());
    }
}

void AutoSwerveCommand::Initialize() {
    VOKC_CHECK(swerve_ != nullptr);

    VOKC_CALL(swerve_->ResetDriveEncoders());
    VOKC_CALL(swerve_->ResetPIDs());

    VOKC_CALL(swerve_->InitAuto(end_pos_, keep_heading_));
}

bool AutoSwerveCommand::IsFinished() {
    return true;
}
