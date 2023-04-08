#include "commands/arm/MoveArmThenTiltCommand.h"
#include "commands/intake/IntakePositionCommand.h"
#include "commands/arm/ArmSetStateCommand.h"

MoveArmThenTiltCommand::MoveArmThenTiltCommand(std::shared_ptr<frc2::Command> move_intake, std::shared_ptr<frc2::Command> move_arm) {
    move_intake_ = move_intake;
    move_arm_ = move_arm;

    arm_good_ = false;
    done_ = false;
}

void MoveArmThenTiltCommand::Initialize() {
    move_arm_->Initialize();

    arm_good_ = false;
    done_ = false;
}

void MoveArmThenTiltCommand::Execute() {
    if (!arm_good_) {
        move_arm_->Execute();
    }

    if (move_arm_->IsFinished()) {
        arm_good_ = true;
    }

    if (arm_good_) {
        move_intake_->Execute();

        done_ = true;
    }
}

bool MoveArmThenTiltCommand::IsFinished() {
    return done_;
}