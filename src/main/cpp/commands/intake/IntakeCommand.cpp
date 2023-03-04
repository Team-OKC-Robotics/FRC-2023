#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Intake.h"

#include "commands/intake/IntakeCommand.h"

IntakeCommand::IntakeCommand(std::shared_ptr<Intake> intake,
                                   double degrees) {
    // Set everything.
    intake_ = intake;
    degrees_ = degrees;

    if (intake_ != nullptr) {
        this->AddRequirements(intake_.get());
    
        intake_->SetControlMode(Auto);
    }
}




void IntakeCommand::Execute() {
    VOKC_CHECK(intake_ != nullptr);
    VOKC_CALL(intake_->SetTurn(turn_))


}

bool IntakeCommand::IsFinished() {
    // Always end this command.
    return true;
}