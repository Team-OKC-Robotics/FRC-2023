#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

#include "commands/arm/ArmSetStateDpadCommand.h"
#include "Parameters.h"



ArmSetStateDpadCommand::ArmSetStateDpadCommand(std::shared_ptr<Arm> arm, std::shared_ptr<frc::Joystick> gamepad) {
    // Set everything.
    arm_ = arm;
    gamepad_ = gamepad;

    negative_pickup_rotation_ = RobotParams::GetParam("arm.negative_pickup.arm_setpoint", 0.0);
    negative_pickup_extension_ = RobotParams::GetParam("arm.negative_pickup.extend_setpoint", 1.0);

    negative_score_mid_rotation_ = RobotParams::GetParam("arm.negative_score_medium.arm_setpoint", 0.0);
    negative_score_mid_extension_ = RobotParams::GetParam("arm.negative_score_medium.extend_setpoint", 1.0);
    
    negative_score_high_rotation_ = RobotParams::GetParam("arm.negative_score_high.arm_setpoint", 0.0);
    negative_score_high_extension_ = RobotParams::GetParam("arm.negative_score_high.extend_setpoint", 1.0);

    if (arm_ != nullptr) {
        this->AddRequirements(arm_.get());
    }
}

void ArmSetStateDpadCommand::Initialize() {
    VOKC_CHECK(arm_ != nullptr);

    VOKC_CALL(arm_->SetControlMode(Auto));
}

void ArmSetStateDpadCommand::Execute() {
    VOKC_CHECK(arm_ != nullptr);

    // use the directional pad for presets on the other side of the robot
    switch (gamepad_->GetPOV()) {
        case 0:
            VOKC_CALL(arm_->SetDesiredState(TeamOKC::ArmState(negative_pickup_extension_, negative_pickup_rotation_)));
            break;
        case 90:
            VOKC_CALL(arm_->SetDesiredState(TeamOKC::ArmState(negative_score_mid_extension_, negative_score_mid_rotation_)));
            break;
        case 180:
            VOKC_CALL(arm_->SetDesiredState(TeamOKC::ArmState(negative_score_high_extension_, negative_score_high_rotation_)));
            break;
        default:
            // don't do anything if nothing else is pressed
            break;
    }
}

bool ArmSetStateDpadCommand::IsFinished() {
    // don't end this command until it is interrupted
    return false;
}