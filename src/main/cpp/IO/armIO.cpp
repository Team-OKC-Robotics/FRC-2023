
#include "io/ArmIO.h"

void ArmIO::Periodic() {
    // Process all the inputs and outputs to/from high level software.
    VOKC_CALL(ProcessIO());
}

void ArmIO::SimulationPeriodic() {
    // SimulationPeriodic
}

bool ArmIO::ProcessIO() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);

    // Set the software outputs
    //If the intake configuration needs to be updated, update it.
        if (sw_interface_->update_config) {
        OKC_CALL(UpdateArmConfig(sw_interface_->arm_config));

        //Lower the update flag
        sw_interface_->update_config = false;
    }

     //If the encoder should be reset, reset it
    if (sw_interface_->reset_encoders) {
        OKC_CALL(ResetEncoders());

        // Lower the encoder reset flag
        sw_interface_->reset_encoders = false;
    }

    // if the encoder should be set to a specific value
    if (sw_interface_->set_encoder_to_val) {
        OKC_CALL(SetEncoder(sw_interface_->encoder_val_to_set))

        sw_interface_->set_encoder_to_val = false;
    }


    // Get the hardware sensor values.
    // limit switches
    sw_interface_->deployed_limit_switch_val = hw_interface_->deploy_limit_switch->Get(); //???
    sw_interface_->retracted_limit_switch_val = hw_interface_->retracted_limit_switch->Get(); //???

    // intake position encoder
    sw_interface_->arm_position_encoder_val =
        hw_interface_->arm_position_motor->GetEncoder().GetPosition();

    return true;
}

bool ArmIO::UpdateArmConfig(ArmConfig &config) {
    OKC_CHECK(hw_interface_ != nullptr);

    // Get the configuration
    double open_loop_ramp = config.open_loop_ramp_rate;
    //double max_output_deploy = config.max_output_deploy;
    //double max_output_retract = config.max_output_retract;
    double max_indexer_current = config.max_indexer_current;

    // Apply the configuration
    // Open Loop Ramp Rate
    hw_interface_->arm_up_motor->SetOpenLoopRampRate(open_loop_ramp);
    hw_interface_->indexer_motor->SetOpenLoopRampRate(open_loop_ramp);
    
    // Java code has this commented out as well, I'm assuming because it was meesing something up.
    // well, the intake works (probably, been a while since it's been actually plugged in) without this,
    // so leaving commented out for now
    // hw_interface_->intake_position_motor->SetOpenLoopRampRate(open_loop_ramp);
    
    // current limiting, so the neo 550 on the indexer doesn't stall and smoke
    hw_interface_->indexer_motor->SetSmartCurrentLimit(max_indexer_current);

    return true;
}

bool ArmIO::ResetEncoders() {
    OKC_CHECK(hw_interface_ != nullptr);

    hw_interface_->arm_position_motor->GetEncoder().SetPosition(0.0);
    
    // we currently don't use the encoder for these other two motors, so we don't need to reset them
    // yet. yet. 
    // hw_interface_->intake_motor->GetEncoder().SetPosition(0.0);
    // hw_interface_->indexer_motor->GetEncoder().SetPosition(0.0);
    
    return true;
}

bool ArmIO::SetEncoder(double &val) {
    OKC_CHECK(hw_interface_ != nullptr);

    hw_interface_->arm_position_motor->GetEncoder().SetPosition(val);

    return true;
}