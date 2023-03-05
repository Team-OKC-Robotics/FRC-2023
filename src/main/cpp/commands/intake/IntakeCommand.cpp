#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Intake.h"

#include "commands/intake/IntakeCommand.h"

IntakeCommand::IntakeCommand(std::shared_ptr<Intake> intake,
                                   double power) {
    // Set everything.
    intake_ = intake;
    power_ = power;

    if (intake_ != nullptr) {
        this->AddRequirements(intake_.get());
    
        intake_->SetControlMode(Manual);
    }
}




void IntakeCommand::Execute() {
    VOKC_CHECK(intake_ != nullptr);
    VOKC_CALL(intake_->SetTurn(power_))


}

bool IntakeCommand::IsFinished() {
    // Always end this command.
    return true;
}