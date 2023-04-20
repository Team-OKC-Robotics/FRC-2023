#include "autos/ScoreConeNoDriveAuto.h"
#include "commands/swerve/AutoDriveCommand.h"
#include "frc2/command/WaitCommand.h"
#include "commands/intake/IntakeCommand.h"
#include "commands/arm/ArmSetStateCommand.h"
#include "Parameters.h"

ScoreConeNoDriveAuto::ScoreConeNoDriveAuto(std::shared_ptr<Arm> arm, std::shared_ptr<Intake> intake) {
    SetName("no drive cone high");

    double degrees = RobotParams::GetParam("arm.score_high_cone.arm_setpoint", 0.0);
    double extend = RobotParams::GetParam("arm.score_high_cone.extend_setpoint", 1.0);

    double negative_pickup_degrees = RobotParams::GetParam("arm.negative_pickup.arm_setpoint", 0.0);
    double negative_pickup_extend = RobotParams::GetParam("arm.negative_pickup.extend_setpoint", 1.0);

    double score_position = RobotParams::GetParam("arm.score_high.intake_setpoint", 0.0);

    AddCommands(
        IntakeCommand(intake, 0.4), // hold the cone in
        ArmSetStateCommand(arm, TeamOKC::ArmState(extend, degrees)),
        frc2::WaitCommand(units::second_t(0.8)),
        IntakeCommand(intake, -0.1), // drop the cone
        frc2::WaitCommand(units::second_t(1.1)), // wait for cone to be dropped
        IntakeCommand(intake, 0), // stop the intake
        ArmSetStateCommand(arm, TeamOKC::ArmState(1, 0)), // bring the arm back in the robot
        frc2::WaitCommand(units::second_t(2.0)) // wait a second
    );
}