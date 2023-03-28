#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Intake.h"

#include "commands/intake/IntakePositionCommand.h"

IntakePositionCommand::IntakePositionCommand(std::shared_ptr<Intake> intake, double angle) {
    // Set everything.
    intake_ = intake;
    angle_ = angle;

    if (intake_ != nullptr) {
        this->AddRequirements(intake_.get());
    }
}

void IntakePositionCommand::Initialize() {
    VOKC_CHECK(intake_ != nullptr)
    VOKC_CALL(intake_->SetControlMode(Auto));
}


void IntakePositionCommand::Execute() {
    VOKC_CHECK(intake_ != nullptr);

    // if the setpoint is 0
    if (angle_ == 0) {
        // then we have to do some special stuff because we can't have the wrist be at 0 inside the robot with a game piece, so we want to slide it either to 80 or -80

        double tilt = 0.0;
        VOKC_CALL(intake_->GetIntakeTilt(&tilt));

        // if we're already more than 0, send it to positive 80
        if (tilt > 0) {
            intake_->SetIntakeTilt(80);
        } else {
            // otherwise slide it to -80
            intake_->SetIntakeTilt(-80);
        }
    } else {
        VOKC_CALL(intake_->SetIntakeTilt(angle_));
    }
}

bool IntakePositionCommand::IsFinished() {
    // Always end this command.
    return true;
}