#include "autos/ScorePreloadedAuto.h"
#include "commands/swerve/AutoDriveCommand.h"
#include "commands/claw/ManualClawCommand.h"
#include "frc2/command/WaitCommand.h"
#include "Parameters.h"

ScorePreloadedAuto::ScorePreloadedAuto(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<Arm> arm, std::shared_ptr<Claw> claw) {
    double degrees = RobotParams::GetParam("arm.score_high.arm_setpoint", 0.0);
    double extend = RobotParams::GetParam("arm.score_high.extend_setpoint", 0.0);

    double pickup_degrees = RobotParams::GetParam("arm.pickup.arm_setpoint", 0.0);
    double pickup_extend = RobotParams::GetParam("arm.pickup.extend_setpoint", 0.0);


    AddCommands(
        ManualClawCommand(claw, -0.3), // hold the cube in
        frc2::WaitCommand(units::second_t(0.3))
        // SetArmAngleCommand(arm, degrees), // raise the arm
        // frc2::WaitCommand(units::second_t(3)),
        // SetArmExtensionCommand(arm, extend), // extend it out
        // frc2::WaitCommand(units::second_t(3)),
        // ManualClawCommand(claw, 0.3), // drop the cube
        // frc2::WaitCommand(units::second_t(0.5)),
        // SetArmExtensionCommand(arm, pickup_extend), // take it back in
        // frc2::WaitCommand(units::second_t(1)),
        // AutoDriveCommand(swerve, -4, 0.5), // back slowly away
        // SetArmAngleCommand(arm, pickup_degrees)
         // lower the arm
    );
}