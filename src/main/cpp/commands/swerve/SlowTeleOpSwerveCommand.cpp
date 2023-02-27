
#include "commands/swerve/SlowTeleOpSwerveCommand.h"

// drive the robot slow for Eli
SlowTeleOpSwerveCommand::SlowTeleOpSwerveCommand(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<frc::Joystick> gamepad) {
    swerve_ = swerve;
    gamepad_ = gamepad;

    if (swerve_ != nullptr) {
        this->AddRequirements(swerve_.get());
    }
}

void SlowTeleOpSwerveCommand::Initialize() {
    VOKC_CHECK(swerve_ != nullptr);
}

void SlowTeleOpSwerveCommand::Execute() {
    VOKC_CHECK(swerve_ != nullptr);
    VOKC_CHECK(gamepad_ != nullptr);

    double drive_power_ = this->gamepad_->GetRawAxis(1);
    double strafe_power_ = this->gamepad_->GetRawAxis(0);
    double turn_power_ = this->gamepad_->GetRawAxis(4);

    // drive slower
    VOKC_CALL(swerve_->VectorTeleOpDrive(-drive_power_/4, -strafe_power_/4, -turn_power_));
}

void SlowTeleOpSwerveCommand::End(bool interrupted) {
    // do nothing lol
}

bool SlowTeleOpSwerveCommand::IsFinished() {
    // this command should never end, unless it is interrupted by another
    return false;
}
