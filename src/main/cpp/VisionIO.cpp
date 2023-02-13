#include "io/VisionIO.h"
void VisionIO::Periodic() {
VOKC_CALL(ProcessIO());
}
void VisionIO::SimulationPeriodic() {

}

bool VisionIO::ProcessIO() {
if (sw_interface_->update_config) {
UpdateVisionConfig(sw_interface_->vision_config);
sw_interface_->update_config = false;
}
ProcessInputs();
SetOutputs();
return true;
}
bool VisionIO::UpdateVisionConfig(VisionConfig &config) {
return true;
}
bool VisionIO::ProcessInputs() {
photonlib::PhotonPipelineResult result = this->hw_interface_->camera->GetLatestResult();
if (result.HasTargets()) {
    photonlib::PhotonTrackedTarget target = result.GetBestTarget();
    sw_interface_->error = target.GetYaw();
    sw_interface_->cone = target.GetArea();
}
return true;
}
bool VisionIO::SetOutputs () {
return true;
}