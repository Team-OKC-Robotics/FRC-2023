#include "autos/ScorePreloadedAuto.h"
#include "commands/swerve/AutoDriveCommand.h"
#include "frc2/command/WaitCommand.h"
#include "commands/intake/IntakeCommand.h"
#include "commands/arm/ArmSetStateCommand.h"
#include "Parameters.h"

ScorePreloadedAuto::ScorePreloadedAuto(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<Arm> arm, std::shared_ptr<Intake> intake) {
    double degrees = RobotParams::GetParam("arm.score_high.arm_setpoint", 0.0);
    double extend = RobotParams::GetParam("arm.score_high.extend_setpoint", 1.0);

    double pickup_degrees = RobotParams::GetParam("arm.pickup.arm_setpoint", 0.0);
    double pickup_extend = RobotParams::GetParam("arm.pickup.extend_setpoint", 1.0);


    AddCommands(
        IntakeCommand(intake, -0.3), // hold the cube/cone in
        frc2::WaitCommand(units::second_t(0.3)),
        ArmSetStateCommand(arm, TeamOKC::ArmState(extend, degrees)),
        frc2::WaitCommand(units::second_t(4)), // wait for the command to finish
        IntakeCommand(intake, 0.3), // drop the cube
        frc2::WaitCommand(units::second_t(0.5)), // wait for cube to be dropped
        ArmSetStateCommand(arm, TeamOKC::ArmState(1, 0)), // bring the arm back in the robot
        AutoDriveCommand(swerve, -4, 0.75), // back slowly away
        ArmSetStateCommand(arm, TeamOKC::ArmState(pickup_extend, pickup_degrees)) // get ready to pick another one up
    );
}