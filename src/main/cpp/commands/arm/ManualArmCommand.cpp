#include "commands/arm/ManualArmCommand.h"

ManualArmCommand::ManualArmCommand(std::shared_ptr<Arm> arm, std::shared_ptr<frc::Joystick> gamepad) {
    arm_ = arm;
    gamepad_ = gamepad;

    if (arm_ != nullptr) {
        this->AddRequirements(arm_.get());
    }
}

void ManualArmCommand::Initialize() {
    VOKC_CHECK(arm_ != nullptr);

    arm_->SetControlMode(Manual);
}

void ManualArmCommand::Execute() {
    VOKC_CHECK(arm_ != nullptr);
    VOKC_CHECK(gamepad_ != nullptr);

   
    double lift_power_ = this->gamepad_->GetRawAxis(1);
    double up_power_ = this->gamepad_->GetRawAxis(0);
    double extend_power_ = this->gamepad_->GetRawAxis(2);

    VOKC_CALL(arm_->SetManualLiftPower(lift_power_));
    VOKC_CALL(arm_->SetManualUpPower(-lift_power_));
    // VOKC_CALL(arm_->SetManualExtendPower(extend_power_));

    
}

void ManualArmCommand::End(bool interrupted) {
    // do nothing lol
}

bool ManualArmCommand::IsFinished() {
    // this command should never end, unless it is interrupted by another
    return false;
}
