#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Intake.h"

#include "commands/intake/IntakePositionCommand.h"

IntakePositionCommand::IntakePositionCommand(std::shared_ptr<Intake> intake, double angle) {
    // Set everything.
    intake_ = intake;
    angle_ = angle;

    if (intake_ != nullptr) {
        this->AddRequirements(intake_.get());
    }
}

void IntakePositionCommand::Initialize() {
    VOKC_CHECK(intake_ != nullptr)
    VOKC_CALL(intake_->SetControlMode(Auto));
}


void IntakePositionCommand::Execute() {
    VOKC_CHECK(intake_ != nullptr);
    VOKC_CALL(intake_->SetIntakeTilt(angle_));
}

bool IntakePositionCommand::IsFinished() {
    // Always end this command.
    return true;
}