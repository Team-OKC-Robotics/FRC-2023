
#include "commands/swerve/TeleOpSwerveCommand.h"

TeleOpSwerveCommand::TeleOpSwerveCommand(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<frc::Joystick> gamepad) {
    swerve_ = swerve;
    gamepad_ = gamepad;

    if (swerve_ != nullptr) {
        this->AddRequirements(swerve_.get());
    }
}

void TeleOpSwerveCommand::Initialize() {
    VOKC_CHECK(swerve_ != nullptr);
}

void TeleOpSwerveCommand::Execute() {
    VOKC_CHECK(swerve_ != nullptr);
    VOKC_CHECK(gamepad_ != nullptr);

    double drive_power_ = this->gamepad_->GetRawAxis(1);
    double strafe_power_ = this->gamepad_->GetRawAxis(0);
    // double turn_power_ = this->gamepad_->GetRawAxis(2);

    double turn_power_ = this->gamepad_->GetRawAxis(4);

    // VOKC_CALL(swerve_->TeleOpDrive(drive_power, strafe_power, turn_power));
    VOKC_CALL(swerve_->VectorTeleOpDrive(-drive_power_, -strafe_power_, -turn_power_*3));
}

void TeleOpSwerveCommand::End(bool interrupted) {
    // do nothing lol
}

bool TeleOpSwerveCommand::IsFinished() {
    // this command should never end, unless it is interrupted by another
    return false;
}
