
#include "io/ArmIO.h"
#include "Parameters.h"

bool ArmIO::Init() {
    // read limits and stuff from the parameters file
    offset = RobotParams::GetParam("arm.offset", -330);
    lift_limit = RobotParams::GetParam("arm.lift_limit", 100);
    extend_limit = RobotParams::GetParam("arm.extend_limit", 100);
    max_output = RobotParams::GetParam("arm.max_output", 0.2);

    double arm_open_loop_ = RobotParams::GetParam("arm.arm_open_loop", 1);
    double extend_open_loop_ = RobotParams::GetParam("arm.extend_open_loop", 1);

    this->sw_interface_->arm_config = ArmConfig {
        arm_open_loop_, // that's what we're here for
        80 // default current limit
    };

    // bool workey_or_no_workey_ = RobotParams::GetParam("robot.work", false);

    // if (workey_or_no_workey_) {
    //     std::cout << "ARM INITIALIZED" << std::endl;
    // } else {
    //     return false; // break the init chain so everything fails
    // }

    return true;
}

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
    // If the intake configuration needs to be updated, update it.
    if (sw_interface_->update_config) {
        OKC_CALL(UpdateArmConfig(sw_interface_->arm_config));

        // Lower the update flag
        sw_interface_->update_config = false;
    }

    // If the encoder should be reset, reset it
    if (sw_interface_->reset_encoders) {
        OKC_CALL(ResetEncoders());

        // Lower the encoder reset flag
        sw_interface_->reset_encoders = false;
    }

    // intake position encoder
    OKC_CHECK(hw_interface_->arm_extend_motor != nullptr);
    OKC_CHECK(hw_interface_->arm_up_motor != nullptr);
   
    sw_interface_->arm_encoder = hw_interface_->arm_lift_encoder->GetPosition();
    sw_interface_->arm_extend_encoder = hw_interface_->arm_extend_encoder->GetPosition();
    sw_interface_->arm_duty_cycle_encoder = hw_interface_->arm_duty_cycle_encoder->GetAbsolutePosition() * 360  +  offset; // offset of 330

    // angle wrapping
    if (sw_interface_->arm_duty_cycle_encoder < -180) {
        sw_interface_->arm_duty_cycle_encoder += 360;
    }

    // clamp maxmium output 
    TeamOKC::Clamp(-max_output, max_output, &sw_interface_->arm_lift_power);
    TeamOKC::Clamp(-max_output, max_output, &sw_interface_->arm_extend_power);

    // if the absolute encoder is >110 degrees
    // if (sw_interface_->arm_duty_cycle_encoder >= lift_limit) {
    //     // and we're trying to go farther positive
    //     if (sw_interface_->arm_lift_power > 0) {
    //         // stop it
    //         hw_interface_->arm_lift_motor->Set(0);
    //         hw_interface_->arm_up_motor->Set(0);
    //     } else {
    //         // otherwise, it's cool
    //         hw_interface_->arm_lift_motor->Set(sw_interface_->arm_lift_power);
    //         hw_interface_->arm_up_motor->Set(-sw_interface_->arm_lift_power);
    //     }
    // } else if (sw_interface_->arm_duty_cycle_encoder <= -lift_limit) {
    //     // and we're trying to go farther negative
    //     if (sw_interface_->arm_lift_power < 0) {
    //         // stop it
    //         hw_interface_->arm_lift_motor->Set(0);
    //         hw_interface_->arm_up_motor->Set(0);
    //     } else {
    //         // otherwise, it's cool
            hw_interface_->arm_lift_motor->Set(sw_interface_->arm_lift_power);
            hw_interface_->arm_up_motor->Set(-sw_interface_->arm_lift_power);
    //     }
    // } else { // if neither limit is reached
    //     hw_interface_->arm_lift_motor->Set(sw_interface_->arm_lift_power);
    //     hw_interface_->arm_up_motor->Set(-sw_interface_->arm_lift_power);
    // }

    sw_interface_->extend_limit_switch = !hw_interface_->extend_limit_switch->Get(); // limit switch is inverse logic

    // if the IR sensor is getting a read
    if (sw_interface_->extend_limit_switch) {
        // reset arm encoder
        hw_interface_->arm_extend_encoder->SetPosition(0.0);

        // if the motor power is positive
        if (sw_interface_->arm_extend_power > 0) {
            // let it pass, so the arm can move outwards
            hw_interface_->arm_extend_motor->Set(sw_interface_->arm_extend_power);
        } else {
            // otherwise, limit it to 0 and don't let it move
            hw_interface_->arm_extend_motor->Set(0);
        }
    } else { // if the limit switch isn't pressed
        // if we're instead at the farthest point
        // if (sw_interface_->arm_extend_encoder > extend_limit) {
            // // and power is positive
            // if (sw_interface_->arm_extend_power > 0) {
            //     // don't let it go farther
            //     hw_interface_->arm_extend_motor->Set(0);
            // } else {
            //     // otherwise let the arm go wherever the heck it wants to go
                hw_interface_->arm_extend_motor->Set(sw_interface_->arm_extend_power);
        //     }
        // // else, if neither limit is reached
        // } else {
        //     // set the power of the motor
        //     hw_interface_->arm_extend_motor->Set(sw_interface_->arm_extend_power);
        // }
    }

    return true;
}

bool ArmIO::UpdateArmConfig(ArmConfig &config) {
    OKC_CHECK(hw_interface_ != nullptr);

    // Get the configuration
    double open_loop_ramp = config.open_loop_ramp_rate;
    // double max_output_deploy = config.max_output_deploy;
    // double max_output_retract = config.max_output_retract;
    double max_indexer_current = config.max_indexer_current;

    // Apply the configuration
    // Open Loop Ramp Rate
    hw_interface_->arm_lift_motor->SetOpenLoopRampRate(open_loop_ramp);
    hw_interface_->arm_up_motor->SetOpenLoopRampRate(open_loop_ramp);
    hw_interface_->arm_extend_motor->SetOpenLoopRampRate(open_loop_ramp);

    // current limiting, so the neo 550 on the indexer doesn't stall and smoke
    hw_interface_->arm_lift_motor->SetSmartCurrentLimit(max_indexer_current);
    hw_interface_->arm_up_motor->SetSmartCurrentLimit(max_indexer_current);
    hw_interface_->arm_extend_motor->SetSmartCurrentLimit(max_indexer_current);

    return true;
}
//HACK: this is a hack
bool ArmIO::ResetEncoders() {
    OKC_CHECK(hw_interface_ != nullptr);

    hw_interface_->arm_lift_motor->GetEncoder().SetPosition(0.0);
    hw_interface_->arm_up_motor->GetEncoder().SetPosition(0.0);
    hw_interface_->arm_extend_motor->GetEncoder().SetPosition(0.0);

    return true;
}

