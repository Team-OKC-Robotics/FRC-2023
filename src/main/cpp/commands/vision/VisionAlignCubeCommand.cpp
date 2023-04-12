
#include "commands/vision/VisionCubeAlignCommand.h"

VisionCubeAlignCommand::VisionCubeAlignCommand(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<Vision> vision) {
    swerve_ = swerve;
    vision_ = vision;

    if (swerve_ != nullptr && vision_ != nullptr) {
        this->AddRequirements(swerve_.get());
        this->AddRequirements(vision_.get());
    }
}

void VisionCubeAlignCommand::Initialize() {
    VOKC_CHECK(swerve_ != nullptr);
    VOKC_CHECK(vision_ != nullptr);

    VOKC_CALL(vision_->Reset());
}

void VisionCubeAlignCommand::Execute() {
    double output = 0.0;

    VOKC_CALL(vision_->GetCubeOutput(&output));

    VOKC_CALL(swerve_->VectorTeleOpDrive(0, 0, output));
}

void VisionCubeAlignCommand::End(bool interrupted) {
    // stop the motors
    VOKC_CALL(swerve_->VectorTeleOpDrive(0, 0, 0));
}

bool VisionCubeAlignCommand::IsFinished() {
    bool at = false;

    VOKC_CALL(vision_->AtSetpoint(&at));
    
    return at;
}