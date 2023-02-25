#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Claw.h"

#include "commands/claw/ManualClawCommand.h"

ManualClawCommand::ManualClawCommand(std::shared_ptr<Claw> claw, double power) {
    claw_ = claw;
    power_ = power;

    if (claw_ != nullptr) {
        this->AddRequirements(claw_.get());
    
        claw_->SetControlMode(Manual);
    }
}

void ManualClawCommand::Execute() {
    VOKC_CHECK(claw_ != nullptr);
    VOKC_CALL(claw_->SetManualPower(power_))


}

void ManualClawCommand::End(bool interrupted) {
    VOKC_CALL(claw_->SetManualPower(0));
    return;
}

bool ManualClawCommand::IsFinished() {
    // Always end this command, as it should continually be scheduled using the WhileHeld() or something
    return true;
}