#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

#include "commands/arm/ArmPresetCommand.h"



ArmPresetCommand::ArmPresetCommand(std::shared_ptr<Arm> arm, double degrees, double extend) {
    // Set everything.
    arm_ = arm;
    degrees_ = degrees;
    extend_ = extend;

    if (arm_ != nullptr) {
        this->AddRequirements(arm_.get());

        arm_->SetControlMode(Auto);
    }
}

void ArmPresetCommand::Initialize() {
    VOKC_CHECK(arm_ != nullptr);

    // bring the extension in before we move
    VOKC_CALL(arm_->SetExtend(1));
}

void ArmPresetCommand::Execute() {
    VOKC_CHECK(arm_ != nullptr);

    // if the arm is all the way in
    bool safe_to_move_arm = false;
    VOKC_CALL(arm_->AtExtendSetpoint(&safe_to_move_arm));    

    // then raise/lower it
    if (safe_to_move_arm) {
        VOKC_CALL(arm_->SetDegrees(degrees_));
    }

    // if the arm is at its setpoint
    bool safe_to_extend_arm = false;
    VOKC_CALL(arm_->AtLiftSetpoint(&safe_to_extend_arm));

    // *then* we can extend it
    if (safe_to_extend_arm) {
        VOKC_CALL(arm_->SetExtend(extend));
        done_ = true;
    }
}

bool ArmPresetCommand::IsFinished() {
    // end the command when the lift and extend are at the setpoint
    return done_;
}