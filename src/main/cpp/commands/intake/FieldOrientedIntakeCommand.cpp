#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Intake.h"

#include "commands/intake/FieldOrientedIntakeCommand.h"

FieldOrientedIntakeCommand::FieldOrientedIntakeCommand(std::shared_ptr<Intake> intake, std::shared_ptr<SwerveDrive> swerve, double angle, double reverse_angle) {
    // Set everything.
    intake_ = intake;
    angle_ = angle;
    swerve_drive_ = swerve;
    reverse_angle_ = reverse_angle;

    if (intake_ != nullptr) {
        this->AddRequirements(intake_.get());
    }
}

void FieldOrientedIntakeCommand::Initialize() {
    VOKC_CHECK(intake_ != nullptr)
    VOKC_CALL(intake_->SetControlMode(Auto));
}


void FieldOrientedIntakeCommand::Execute() {
    VOKC_CHECK(intake_ != nullptr);

    // get the current robot heading
    double heading = 0.0;
    VOKC_CALL(swerve_drive_->GetHeading(&heading));
    
    // convert to [-180, 180]
    TeamOKC::WrapAngle(&heading);

    // if the heading is pointing away from the scoring area
    if (abs(heading) < 90.0) {
        // score on the normal side of the robot
        // set the desired state of the arm
        VOKC_CALL(intake_->SetIntakeTilt(angle_));
    } else {
        // otherwise, score on the other side
        // set the desired state of the arm
        VOKC_CALL(intake_->SetIntakeTilt(reverse_angle_));
    }
    
}

bool FieldOrientedIntakeCommand::IsFinished() {
    // Always end this command.
    return true;
}