#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

#include "commands/arm/IncrementArmExtendCommand.h"

IncrementArmExtendCommand::IncrementArmExtendCommand(std::shared_ptr<Arm> arm,
                                   double extend) {
    // Set everything.
    arm_ = arm;
    extend_ = extend;

    if (arm_ != nullptr) {
        this->AddRequirements(arm_.get());
    }
}

void IncrementArmExtendCommand::Initialize() {
    
}


void IncrementArmExtendCommand::Execute() {
    VOKC_CHECK(arm_ != nullptr);
    VOKC_CALL(arm_->IncrementExtend(extend_))
}

bool IncrementArmExtendCommand::IsFinished() {
    bool at = false;

    OKC_CALL(arm_->AtExtendSetpoint(&at));

    return at;
}
