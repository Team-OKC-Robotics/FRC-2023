
#include "commands/swerve/AutoDriveCommand.h"

AutoDriveCommand::AutoDriveCommand(std::shared_ptr<SwerveDrive> swerve, double dist, double max_speed, double strafe) {
    swerve_ = swerve;
    dist_ = dist;
    max_speed_ = max_speed;
    strafe_ = strafe;

    if (swerve_ != nullptr) {
        this->AddRequirements(swerve_.get());
    }
}

void AutoDriveCommand::Initialize() {
    VOKC_CHECK(swerve_ != nullptr);

    VOKC_CALL(swerve_->ResetDriveEncoders());
    VOKC_CALL(swerve_->ResetPIDs());

    VOKC_CALL(swerve_->SetDistance(dist_));
    VOKC_CALL(swerve_->SetIdleMode(BRAKE));
}

void AutoDriveCommand::Execute() {
    VOKC_CALL(swerve_->DriveAuto(max_speed_));
}

void AutoDriveCommand::End(bool interrupted) {
    // stop the motors
    VOKC_CALL(swerve_->VectorTeleOpDrive(0, 0, 0));
}

bool AutoDriveCommand::IsFinished() {
    bool done_ = false;

    OKC_CALL(swerve_->AtDistSetpoint(&done_));
    
    return done_;
}