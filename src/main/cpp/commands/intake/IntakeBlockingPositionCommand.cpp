#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Intake.h"

#include "commands/intake/IntakeBlockingPositionCommand.h"

IntakeBlockingPositionCommand::IntakeBlockingPositionCommand(std::shared_ptr<Intake> intake, double angle) {
    // Set everything.
    intake_ = intake;
    angle_ = angle;

    if (intake_ != nullptr) {
        this->AddRequirements(intake_.get());
    }
}

void IntakeBlockingPositionCommand::Initialize() {
    VOKC_CHECK(intake_ != nullptr)
    VOKC_CALL(intake_->SetControlMode(Auto));
}


void IntakeBlockingPositionCommand::Execute() {
    VOKC_CHECK(intake_ != nullptr);
    VOKC_CALL(intake_->SetIntakeTilt(angle_));
}

bool IntakeBlockingPositionCommand::IsFinished() {
    // bool at = false;

    // OKC_CALL(intake_->AtSetpoint(&at));
    
    // return at;

    return true;
}