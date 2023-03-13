
#include "io/ClawIO.h"

void ClawIO::Periodic() {
    // Process all the inputs and outputs to/from high level software.
    VOKC_CALL(ProcessIO());
}

void ClawIO::SimulationPeriodic() {
    // SimulationPeriodic
}

bool ClawIO::ProcessIO() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);
    OKC_CHECK(hw_interface_->claw_open_and_close_encoder != nullptr);
    OKC_CHECK(hw_interface_->claw_open_and_close_motor != nullptr);

    // Set the software outputs
    // If the encoder should be reset, reset it
    if (sw_interface_->reset_claw_open_and_close) {
        hw_interface_->claw_open_and_close_encoder->SetPosition(0.0);
        
        // Lower the encoder reset flag
        sw_interface_->reset_claw_open_and_close = false;
    }

    if (sw_interface_->update_config) {
        UpdateConfig(sw_interface_->config);

        sw_interface_->update_config = false;
    }

    hw_interface_->claw_open_and_close_motor->Set(sw_interface_->claw_open_and_close_power);
    sw_interface_->encoder_reading = hw_interface_->claw_open_and_close_encoder->GetPosition();

    return true;
}

bool ClawIO::UpdateConfig(ClawConfig &config) {
    OKC_CHECK(&config != nullptr); // ?
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);

    hw_interface_->claw_open_and_close_motor->SetSmartCurrentLimit(config.current_limit);

    return true;
}
