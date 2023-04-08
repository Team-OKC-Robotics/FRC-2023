#include "autos/ScorePreloadedBalanceAuto.h"
#include "commands/swerve/AutoDriveCommand.h"
#include "frc2/command/WaitCommand.h"
#include "commands/intake/IntakeCommand.h"
#include "commands/swerve/AutoBalanceCommand.h"
#include "commands/arm/ArmSetStateCommand.h"
#include "commands/intake/IntakePositionCommand.h"
#include "Parameters.h"

ScorePreloadedBalanceAuto::ScorePreloadedBalanceAuto(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<Arm> arm, std::shared_ptr<Intake> intake) {
    SetName("score high balance");

    double degrees = RobotParams::GetParam("arm.score_high.arm_setpoint", 0.0);
    double extend = RobotParams::GetParam("arm.score_high.extend_setpoint", 1.0);

    double pickup_degrees = RobotParams::GetParam("arm.pickup.arm_setpoint", 0.0);
    double pickup_extend = RobotParams::GetParam("arm.pickup.extend_setpoint", 1.0);

    double score_position = RobotParams::GetParam("arm.score_high.intake_setpoint", 0.0);

    AddCommands(
        IntakeCommand(intake, 0.0), // hold the cube/cone in
        ArmSetStateCommand(arm, TeamOKC::ArmState(extend, degrees)), // move the arm to score high
        frc2::WaitCommand(units::second_t(2)), // wait for the command to finish
        frc2::WaitCommand(units::second_t(1.5)),
        IntakeCommand(intake, 1.0), // drop the cube
        frc2::WaitCommand(units::second_t(1)), // wait for cube to be dropped
        IntakeCommand(intake, 0), // stop the intake
        ArmSetStateCommand(arm, TeamOKC::ArmState(0.5, 0)), // bring the arm back in the robot
        frc2::WaitCommand(units::second_t(2)), // wait a second so the arm is mostly in
        AutoBalanceCommand(swerve, 1.0) // balance on the charge station
    );
}