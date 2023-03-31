#include "autos/DriveForwardAuto.h"
#include "commands/swerve/AutoDriveCommand.h"
#include "frc2/command/WaitCommand.h"
#include "commands/intake/IntakeCommand.h"
#include "commands/arm/ArmSetStateCommand.h"
#include "commands/intake/IntakePositionCommand.h"
#include "Parameters.h"

DriveForwardAuto::DriveForwardAuto(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<Arm> arm, std::shared_ptr<Intake> intake) {
    SetName("drive forwards auto");

    AddCommands(
        frc2::WaitCommand(units::second_t(1.5)), // wait for the command to finish
        AutoDriveCommand(swerve, 4.7, 1) // back "slowly" away
    );
}