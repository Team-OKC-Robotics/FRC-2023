#include "commands/claw/AutomaticClawCommand.h"
#include "subsystems/Claw.h"



AutomaticClawCommand::AutomaticClawCommand(std::shared_ptr<Claw> claw,
                                   double distance) {
    // Set everything.
    claw_ = claw;
    distance_ = distance;

    if (claw_ != nullptr) {
        this->AddRequirements(claw_.get());
    }
}

void AutomaticClawCommand::Execute() {
    VOKC_CHECK(claw_ != nullptr);
    VOKC_CALL(claw_->ExpandClaw(distance_));
}

bool AutomaticClawCommand::IsFinished() {
    // Always end this command.
    return true;
}