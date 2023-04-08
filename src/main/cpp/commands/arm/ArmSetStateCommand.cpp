#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

#include "commands/arm/ArmSetStateCommand.h"



ArmSetStateCommand::ArmSetStateCommand(std::shared_ptr<Arm> arm, TeamOKC::ArmState state) {
    // Set everything.
    arm_ = arm;
    state_ = state;

    if (arm_ != nullptr) {
        this->AddRequirements(arm_.get());
    }
}

void ArmSetStateCommand::Initialize() {
    VOKC_CHECK(arm_ != nullptr);

    
}

void ArmSetStateCommand::Execute() {
    VOKC_CHECK(arm_ != nullptr);

    // set the desired state of the arm
    VOKC_CALL(arm_->SetDesiredState(state_));
}

bool ArmSetStateCommand::IsFinished() {
    bool at = false;

    OKC_CALL(arm_->AtLiftSetpoint(&at));

    return at;
}