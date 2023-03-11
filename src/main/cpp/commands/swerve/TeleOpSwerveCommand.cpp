
#include "commands/swerve/TeleOpSwerveCommand.h"

TeleOpSwerveCommand::TeleOpSwerveCommand(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<frc::Joystick> gamepad, double speed_mod, double open_loop_ramp_rate, bool brake_mode) {
    swerve_ = swerve;
    gamepad_ = gamepad;
    speed_mod_ = speed_mod;
    open_loop_ramp_rate_ = open_loop_ramp_rate;
    brake_mode_ = brake_mode;

    if (swerve_ != nullptr) {
        this->AddRequirements(swerve_.get());
    }
}

void TeleOpSwerveCommand::Initialize() {
    VOKC_CHECK(swerve_ != nullptr);

    swerve_->SetOpenLoopRampDrive(open_loop_ramp_rate_);
    
    if (brake_mode_) {
        swerve_->SetIdleMode(BRAKE);
    } else {
        swerve_->SetIdleMode(COAST);
    }
}

void TeleOpSwerveCommand::Execute() {
    VOKC_CHECK(swerve_ != nullptr);
    VOKC_CHECK(gamepad_ != nullptr);

    double drive_power_ = this->gamepad_->GetRawAxis(1);
    double strafe_power_ = this->gamepad_->GetRawAxis(0);
    double turn_power_ = this->gamepad_->GetRawAxis(4);

    VOKC_CALL(swerve_->VectorTeleOpDrive(drive_power_*speed_mod_, strafe_power_*speed_mod_, -turn_power_*3));
}

bool TeleOpSwerveCommand::IsFinished() {
    // this command should never end, unless it is interrupted by another
    return false;
}
