#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Intake.h"

#include "commands/intake/IncrementIntakePositionCommand.h"

IncrementIntakePositionCommand::IncrementIntakePositionCommand(std::shared_ptr<Intake> intake, double angle) {
    // Set everything.
    intake_ = intake;
    angle_ = angle;

    if (intake_ != nullptr) {
        this->AddRequirements(intake_.get());
    }
}

void IncrementIntakePositionCommand::Initialize() {
    VOKC_CALL(intake_->SetControlMode(Auto));
}


void IncrementIntakePositionCommand::Execute() {
    VOKC_CHECK(intake_ != nullptr);
    VOKC_CALL(intake_->IncrementIntakeTilt(angle_));
}

bool IncrementIntakePositionCommand::IsFinished() {
    // Always end this command.
    return true;
}