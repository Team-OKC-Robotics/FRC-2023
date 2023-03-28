#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

#include "commands/arm/ArmFieldOrientedCommand.h"



ArmFieldOrientedCommand::ArmFieldOrientedCommand(std::shared_ptr<Arm> arm, std::shared_ptr<SwerveDrive> swerve, TeamOKC::ArmState state, TeamOKC::ArmState reverse_state) {
    // Set everything.
    arm_ = arm;
    swerve_drive_ = swerve;
    state_ = state;
    reverse_state_ = reverse_state;

    if (arm_ != nullptr) {
        this->AddRequirements(arm_.get());
    }
}

void ArmFieldOrientedCommand::Initialize() {
    VOKC_CHECK(arm_ != nullptr);

    VOKC_CALL(arm_->SetControlMode(Auto));
}

void ArmFieldOrientedCommand::Execute() {
    VOKC_CHECK(arm_ != nullptr);

    // get the current robot heading
    double heading = 0.0;
    VOKC_CALL(swerve_drive_->GetHeading(&heading));
    
    // convert to [-180, 180]
    TeamOKC::WrapAngle(&heading);

    // if the heading is pointing away from the scoring area
    if (abs(heading) < 90.0) {
        // score on the normal side of the robot
        // set the desired state of the arm
        VOKC_CALL(arm_->SetDesiredState(state_));
    } else {
        // otherwise, score on the other side
        // set the desired state of the arm
        VOKC_CALL(arm_->SetDesiredState(reverse_state_));
    }

}

bool ArmFieldOrientedCommand::IsFinished() {
    bool at = false;

    OKC_CALL(arm_->AtLiftSetpoint(&at));

    return at;
}