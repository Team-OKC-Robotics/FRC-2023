#include "commands/arm/TiltThenMoveArmCommand.h"
#include "commands/intake/IntakePositionCommand.h"
#include "commands/arm/ArmSetStateCommand.h"

TiltThenMoveArmCommand::TiltThenMoveArmCommand(std::shared_ptr<IntakeBlockingPositionCommand> move_intake, std::shared_ptr<ArmSetStateCommand> move_arm) {

    AddCommands(
        // *move_intake,
        // *move_arm
    );
}