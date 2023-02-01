#include "io/VisionIO.h"

void VisionIO::Periodic() {
    ProcessIO();
}

void VisionIO::SimulationPeriodic() {

}

bool VisionIO::ProcessIO() {
    // if we need to update the config
    if (sw_interface_->update_config) {
        UpdateVisionConfig(sw_interface_->vision_config);

        // Lower the update flag
        sw_interface_->update_config = false;
    }

    ProcessInputs();
    SetOutputs();
}

bool VisionIO::UpdateVisionConfig(VisionConfig &config) {

}

bool VisionIO::ProcessInputs() {
    photonlib::PhotonPipelineResult result = this->hw_interface->camera.GetLatestResult();
    if (result.HasTargets()) {
        photonlib::PhotonTrackedTarget target = result.GetBestTarget();

        // cone error (x-axis)
        sw_interface_->error = result.GetYaw();
        sw_interface_->cone = result.GetArea();
    }
    cube;
    angle;
}

bool VisionIO::SetOutputs() {

}