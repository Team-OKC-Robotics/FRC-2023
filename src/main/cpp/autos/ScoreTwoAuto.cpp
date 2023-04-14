#include "autos/ScoreTwoAuto.h"
#include "commands/swerve/AutoDriveCommand.h"
#include "frc2/command/WaitCommand.h"
#include "commands/intake/IntakeCommand.h"
#include "commands/arm/ArmSetStateCommand.h"
#include "commands/swerve/ResetGyroCommand.h"
#include "Parameters.h"

ScoreTwoAuto::ScoreTwoAuto(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<Arm> arm, std::shared_ptr<Intake> intake) {
    SetName("score two cubes");

    double degrees = RobotParams::GetParam("arm.score_high_cone.arm_setpoint", 0.0);
    double extend = RobotParams::GetParam("arm.score_high_cone.extend_setpoint", 1.0);

    double negative_pickup_degrees = RobotParams::GetParam("arm.negative_pickup.arm_setpoint", 0.0);
    double negative_pickup_extend = RobotParams::GetParam("arm.negative_pickup.extend_setpoint", 1.0);

    AddCommands(
        ResetGyroCommand(swerve),
        IntakeCommand(intake, 0.4), // hold the cube/cone in
        ArmSetStateCommand(arm, TeamOKC::ArmState(extend, degrees)),
        frc2::WaitCommand(units::second_t(0.8)),
        IntakeCommand(intake, -0.1), // drop the cube
        frc2::WaitCommand(units::second_t(1.1)), // wait for cube to be dropped
        IntakeCommand(intake, 1.0), // run intake in
        ArmSetStateCommand(arm, TeamOKC::ArmState(negative_pickup_extend, negative_pickup_degrees)), // get ready to pick another one up
        frc2::WaitCommand(units::second_t(1.3)),
        AutoDriveCommand(swerve, 5.3, 0.5, -17.0), // back slowly away
        frc2::WaitCommand(units::second_t(0.3)),
        ArmSetStateCommand(arm, TeamOKC::ArmState(extend, degrees)), // set the arm to score high
        IntakeCommand(intake, 0.4), // just hold the cube in, not too much power
        AutoDriveCommand(swerve, -5.0, 0.6, 5.0), // back slowly away
        IntakeCommand(intake, -1.0) // spit the cube out really hard, hopefully it lands in the right spot
        // and we should be out of time, really.    
    );
}