#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Arm.h"

#include "commands/arm/WaitForArmCommand.h"



WaitForArmCommand::WaitForArmCommand(std::shared_ptr<Arm> arm) {
    // Set everything.
    arm_ = arm;

    if (arm_ != nullptr) {
        this->AddRequirements(arm_.get());
    }
}

void WaitForArmCommand::Execute() {
    VOKC_CHECK(arm_ != nullptr);

}

bool WaitForArmCommand::IsFinished() {
    return false;
}