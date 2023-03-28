#include "commands/arm/TiltThenMoveArmCommand.h"
#include "commands/intake/IntakePositionCommand.h"
#include "commands/arm/ArmSetStateCommand.h"

TiltThenMoveArmCommand::TiltThenMoveArmCommand(std::shared_ptr<IntakeBlockingPositionCommand> move_intake, std::shared_ptr<ArmSetStateCommand> move_arm) {
    move_intake_ = move_intake;
    move_arm_ = move_arm;

    intake_good = false;
    done_ = false;
}

void TiltThenMoveArmCommand::Initialize() {
    move_intake_->Initialize();

    intake_good = false;
    done_ = false;
}

void TiltThenMoveArmCommand::Execute() {
    if (!intake_good) {
        move_intake_->Execute();
    }

    if (move_intake_->IsFinished()) {
        intake_good = true;
    }

    if (intake_good) {
        move_arm_->Execute();

        done_ = true;
    }
}

bool TiltThenMoveArmCommand::IsFinished() {
    return done_;
}