#include "autos/ScorePreloadedAuto.h"
#include "commands/swerve/AutoDriveCommand.h"
#include "frc2/command/WaitCommand.h"
#include "commands/intake/IntakeCommand.h"
#include "commands/arm/ArmSetStateCommand.h"
#include "Parameters.h"

ScorePreloadedAuto::ScorePreloadedAuto(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<Arm> arm, std::shared_ptr<Intake> intake) {
    SetName("score high back up");

    double degrees = RobotParams::GetParam("arm.score_high.arm_setpoint", 0.0);
    double extend = RobotParams::GetParam("arm.score_high.extend_setpoint", 1.0);

    double negative_pickup_degrees = RobotParams::GetParam("arm.negative_pickup.arm_setpoint", 0.0);
    double negative_pickup_extend = RobotParams::GetParam("arm.negative_pickup.extend_setpoint", 1.0);

    double score_position = RobotParams::GetParam("arm.score_high.intake_setpoint", 0.0);

    AddCommands(
       // IntakeCommand(intake, -0.1), // hold the cube/cone in
        // ArmSetStateCommand(arm, TeamOKC::ArmState(extend, degrees)),
        // frc2::WaitCommand(units::second_t(3.0)), // wait for the command to finish
        // frc2::WaitCommand(units::second_t(1.5)), // wait for the command to finish
        // IntakeCommand(intake, 0.3), // drop the cube
        // frc2::WaitCommand(units::second_t(0.5)), // wait for cube to be dropped
        // IntakeCommand(intake, 0), // stop the intake
        // ArmSetStateCommand(arm, TeamOKC::ArmState(1, 0)), // bring the arm back in the robot
        // frc2::WaitCommand(units::second_t(2)), // wait a second
        AutoDriveCommand(swerve, 2.0, 1, 0) // back slowly away
    );
}