#include "IO/VisionIO.h"
void VisionIO::Periodic() {
ProcessIO();
}
void VisionIO::SimulationPeriodic() {

}
bool Visionconfig::UpdateVisionConfig {
    return true;
}
bool VisionIO::ProcessIO() {
if (sw_interface_->update_config0) {
    UpdateVisionConfig(sw_interface_->vision_config);
    sw_interface_->update_config = false;
}
ProcessInputs();
SetOutputs();
}
void VisionIO::UpdateVisionConfig(Visionconfig &config) {

}
bool VisionIOL::ProcessInputs() {
    photonlib::PhotonPipelineResult result = this->hw_interface->camera.GetLatestResult();
    if (result.HasTargets()) {
        photonlib::PhotonTrackedTarget target = result.GetBestTarget();
        sw_interface_->error = result.GetYaw();
    }
    sw_interface_->cone = result.GetArea();
sw_interface->error = hw_interface_->camera;
}
bool VisionIO::SetOutputs () {
return true;
}