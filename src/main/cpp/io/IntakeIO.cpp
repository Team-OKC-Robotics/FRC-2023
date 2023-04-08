#include "io/IntakeIO.h"
#include "Parameters.h"

bool IntakeIO::Init() {
    
    hw_interface_->intake_motor->SetSmartCurrentLimit(30);

    hw_interface_->intake_motor->SetSmartCurrentLimit(30);

    return true;
}

void IntakeIO::Periodic() {
    // Process all the inputs and outputs to/from high level software.
    
    VOKC_CALL(ProcessIO());
}

void IntakeIO::SimulationPeriodic() {
    // SimulationPeriodic
}

bool IntakeIO::ProcessIO() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);

    // Set the software outputs
    // If the intake configuration needs to be updated, update it.
    if (sw_interface_->update_config) {
        OKC_CALL(UpdateIntakeConfig(sw_interface_->intake_config));

        // Lower the update flag
        sw_interface_->update_config = false;
    }

    // If the encoder should be reset, reset it
    if (sw_interface_->reset_encoders) {
        OKC_CALL(ResetEncoders());

        // Lower the encoder reset flag
        sw_interface_->reset_encoders = false;
    }

    // === inputs ===
    // intake encoder
    OKC_CHECK(hw_interface_->intake_motor != nullptr);
    sw_interface_->intake_encoder = hw_interface_->intake_encoder->GetPosition();

   

    // === outputs ===
    hw_interface_->intake_motor->Set(sw_interface_->intake_power);

    return true;
}

bool IntakeIO::UpdateIntakeConfig(IntakeConfig &config) {
    OKC_CHECK(hw_interface_ != nullptr);
    
    // TODO

    return true;
}
    
//HACK: this is a hack
bool IntakeIO::ResetEncoders() {
    OKC_CHECK(hw_interface_ != nullptr);

    hw_interface_->intake_motor->GetEncoder().SetPosition(0.0);
 
    return true;
}

