#include "io/VisionIO.h"

void VisionIO::Periodic() {
    VOKC_CALL(ProcessIO());
}

void VisionIO::SimulationPeriodic() {}

bool VisionIO::ProcessIO() {
    OKC_CHECK(sw_interface_ != nullptr);

    if (sw_interface_->update_config) {
        OKC_CALL(UpdateVisionConfig(sw_interface_->vision_config));
        sw_interface_->update_config = false;
    }

    OKC_CALL(ProcessInputs());
    OKC_CALL(SetOutputs());
    return true;
}

bool VisionIO::UpdateVisionConfig(VisionConfig &config) {
    return true;
}

bool VisionIO::ProcessInputs() {
    OKC_CHECK(hw_interface_ != nullptr);
    OKC_CHECK(sw_interface_ != nullptr);

    photonlib::PhotonPipelineResult result =
        this->hw_interface_->camera->GetLatestResult();

    if (result.HasTargets()) {
        photonlib::PhotonTrackedTarget target = result.GetBestTarget();
        sw_interface_->cube_yaw = target.GetYaw();
        sw_interface_->cube_dist = target.GetArea();
    }

    return true;
}

bool VisionIO::SetOutputs() {
    return true;
}