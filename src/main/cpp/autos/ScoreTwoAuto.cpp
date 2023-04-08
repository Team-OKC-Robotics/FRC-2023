#include "autos/ScoreTwoAuto.h"
#include "commands/swerve/AutoDriveCommand.h"
#include "frc2/command/WaitCommand.h"
#include "commands/intake/IntakeCommand.h"
#include "commands/arm/ArmSetStateCommand.h"
#include "Parameters.h"

ScoreTwoAuto::ScoreTwoAuto(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<Arm> arm, std::shared_ptr<Intake> intake) {
    SetName("score two cubes");

    double degrees = RobotParams::GetParam("arm.score_high.arm_setpoint", 0.0);
    double extend = RobotParams::GetParam("arm.score_high.extend_setpoint", 1.0);

    double negative_pickup_degrees = RobotParams::GetParam("arm.negative_pickup.arm_setpoint", 0.0);
    double negative_pickup_extend = RobotParams::GetParam("arm.negative_pickup.extend_setpoint", 1.0);

    AddCommands(
        IntakeCommand(intake, -0.1), // hold the cube/cone in
        ArmSetStateCommand(arm, TeamOKC::ArmState(extend, degrees)),
        frc2::WaitCommand(units::second_t(4.5)), // wait for the command to finish
        IntakeCommand(intake, 0.3), // drop the cube
        frc2::WaitCommand(units::second_t(0.5)), // wait for cube to be dropped
        IntakeCommand(intake, 0), // stop the intake
        ArmSetStateCommand(arm, TeamOKC::ArmState(1, 0)), // bring the arm back in the robot
        frc2::WaitCommand(units::second_t(2)), // wait a second
        AutoDriveCommand(swerve, 5, 1), // back slowly away
        ArmSetStateCommand(arm, TeamOKC::ArmState(negative_pickup_extend, negative_pickup_degrees)), // get ready to pick another one up
        IntakeCommand(intake, -0.3), // suck the cube in
        AutoDriveCommand(swerve, 0.5, 0.2), // drive into the cube
        IntakeCommand(intake, -0.1), // just hold the cube in, not too much power
        ArmSetStateCommand(arm, TeamOKC::ArmState(extend, degrees)), // set the arm to score high
        AutoDriveCommand(swerve, -5, 0.5), // drive back to the station
        IntakeCommand(intake, 1.0) // spit the cube out really hard, hopefully it lands in the right spot
        // and we should be out of time, really.
    );
}