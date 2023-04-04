#include "subsystems/Arm.h"
#include "Parameters.h"
#include "ui/UserInterface.h"
#include "Utils.h"


bool Arm::Init() {
    //pulls PID values from the parameters.toml file and initializes the PID controllers
    double arm_kP = RobotParams::GetParam("arm.lift_pid.kP", 0.005);
    double arm_kI = RobotParams::GetParam("arm.lift_pid.kI", 0.0);
    double arm_kD = RobotParams::GetParam("arm.lift_pid.kD", 0.0);
    double arm_kF = RobotParams::GetParam("arm.lift_pid.kF", 0.05);
    arm_pid_ = std::make_shared<frc::PIDController>(arm_kP, arm_kI, arm_kD);
    arm_pid_->SetTolerance(2.5, 3.0);
    arm_kF_ = arm_kF;

    double extend_kP = RobotParams::GetParam("arm.extend_pid.kP", 0.005);
    double extend_kI = RobotParams::GetParam("arm.extend_pid.kI", 0.0);
    double extend_kD = RobotParams::GetParam("arm.extend_pid.kD", 0.0);
    inches_pid_ = std::make_shared<frc::PIDController>(extend_kP, extend_kI, extend_kD);
    inches_pid_->SetTolerance(0.7, 2.0);

    // logs
    arm_lift_output_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/lift_output");
    arm_lift_enc_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/lift_enc");
    arm_lift_setpoint_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/lift_setpoint");

    arm_extend_output_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/extend_output");
    arm_extend_enc_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/extend_enc");
    arm_extend_setpoint_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/extend_setpoint"); 


   

    // pull limits from the parameters file
    lift_limit_ = RobotParams::GetParam("arm.lift_limit", 100.0);
    extend_limit_ = RobotParams::GetParam("arm.extend_limit", 100.0);
  
    // initialize with default state
    state_ = TeamOKC::ArmState(0, 0);
    desired_state_ = TeamOKC::ArmState(0, 0);

    // done initializing, go to calibration for the extension
    control_state_ = CALIBRATING;

    return true;

}

bool Arm::SetControlMode(const ControlMode & mode) {
        mode_ = mode;

        return true; 
    }

bool Arm::ManualControl() {
    OKC_CHECK(interface_ != nullptr);

    interface_->arm_lift_power = lift_power_;
    interface_->arm_extend_power = extend_power_;

    return true;
}


bool Arm::SetDesiredState(TeamOKC::ArmState state) {
    this->desired_state_ = state;

    control_state_ = ROTATING;

    has_been_commanded_ = true;

    return true;
}

bool Arm::IncrementRotation(double increment) {
    this->desired_state_.rotation += increment;

    has_been_commanded_ = true;
    control_state_ = ROTATING;

    return true;
}

bool Arm::IncrementExtend(double increment) {
    this->desired_state_.extension += increment;

    has_been_commanded_ = true;
    control_state_ = ROTATING;

    return true;
}

bool Arm::AtExtendSetpoint(bool *at) {
    // if the error is less than the threshold, then we're there
    // *at = abs(this->desired_state_.extension - this->state_.extension) < extend_threshold_;
    *at = abs(this->desired_state_.extension - this->state_.extension) < 0.5;

    return true;
}

bool Arm::AtLiftSetpoint(bool *at) {
    // if the error is less than the threshold, then we're there
    // *at = abs(this->desired_state_.rotation - this->state_.rotation) < rotation_threshold_;
    *at = arm_pid_->AtSetpoint() && inches_pid_->AtSetpoint();

    return true;
}


// test mode for creating setpoints
// NOTE: this does not prevent you from extending into the robot. BE CAREFUL
bool Arm::TestControl() {
    OKC_CHECK(interface_ != nullptr);

    // if we haven't actually started
    if (!calibration_allowed_) {
        // don't let the arm do anything
        return true;
    }


    // zero the extension encoder on startup
    if (!has_calibrated_) {
        // if we should be calibrating
        if (control_state_ == CALIBRATING) {
            // check the limit switch
            if (this->interface_->extend_limit_switch) {
                // we're hitting the limit switch, so we're done calibrating
                control_state_ = ROTATING;

                has_calibrated_ = true;

                // initialize the desired state to the state when we are calibrated                
                this->desired_state_.rotation = this->interface_->arm_duty_cycle_encoder;
                this->desired_state_.extension = 1; // slightly not 0 just because
            } else {
                // otherwise, we haven't hit it yet, so set the motor to a small negative power until we do
                this->interface_->arm_extend_power = -0.1;
            }
        }

        return true;
    }

    // limit rotation
    if (this->desired_state_.rotation > lift_limit_) {
        this->desired_state_.rotation = lift_limit_;
    } else if (this->desired_state_.rotation < -lift_limit_) {
        this->desired_state_.rotation = -lift_limit_;
    }

    // limit extension
    if (this->desired_state_.extension > extend_limit_) {
        this->desired_state_.extension = extend_limit_;
    } else if (this->desired_state_.extension < 0) {
        this->desired_state_.extension = 0;
    }

    // set setpoints
    this->arm_pid_->SetSetpoint(this->desired_state_.rotation);
    this->inches_pid_->SetSetpoint(this->desired_state_.extension);

    // calculate feedforward
    // take the sign of the desired state, multiply it by the feedforward gain times the desired rotation squared. this looks like output = signum(setpoint) * kF * setpoint^2
    double ff = TeamOKC::sign(this->desired_state_.rotation) * arm_kF_ * this->desired_state_.rotation * this->desired_state_.rotation;
    
    this->interface_->arm_lift_power = this->arm_pid_->Calculate(this->state_.rotation) + ff;
    this->interface_->arm_extend_power = this->inches_pid_->Calculate(this->state_.extension);

    return true;
}

bool Arm::AllowCalibration() {
    OKC_CHECK(interface_ != nullptr);

    calibration_allowed_ = true;

    return true;
}

bool Arm::AutoControl() {
    OKC_CHECK(interface_ != nullptr);
    OKC_CHECK(this->arm_pid_ != nullptr);
    OKC_CHECK(this->inches_pid_ != nullptr);

    /**
     * TODO: zero all flags at start of auto and start of teleop
    */

    // if we haven't actually started
    if (!calibration_allowed_) {
        // don't let the arm do anything
        return true;
    }

    // zero the extension encoder on startup
    if (!has_calibrated_) {
        // if we should be calibrating
        if (this->interface_->extend_limit_switch) {
            // we're hitting the limit switch, so we're done calibrating
            control_state_ = ROTATING;

            has_calibrated_ = true;

            this->interface_->arm_extend_power = 0.0;
            
            std::cout << "CALIBRATION COMPLETE" << std::endl;
        } else {
            // otherwise, we haven't hit it yet, so set the motor to a small negative power until we do
            this->interface_->arm_extend_power = -0.1;
        }
    
        return true;
    }

    // if we haven't been told to do something yet
    if (!has_been_commanded_) {
        // then don't move anything
        return true;
    }

    // alright, we've made it thus far, so we need to get down to business and start moving
    if (control_state_ == ROTATING) {
        // bring the extension in whenever we rotate the arm, to reduce bounce
        this->inches_pid_->SetSetpoint(0.5);

        // if we have brought the extension in
        if (abs(1.0 - state_.extension) < 1.0) {
            // then move the arm
            this->arm_pid_->SetSetpoint(this->desired_state_.rotation);
            this->interface_->arm_lift_power = this->arm_pid_->Calculate(this->interface_->arm_duty_cycle_encoder);

            //TODO limit us from rotating too far

            // keep the extension where it needs to be
            this->interface_->arm_extend_power = this->inches_pid_->Calculate(this->interface_->arm_extend_encoder);
            
            // if we've reached our desired rotation
            if (abs(desired_state_.rotation - state_.rotation) < 2) {
                // move to the next stage
                this->control_state_ = EXTENDING;
            }
        } else { // otherwise we haven't brought the extension in
            // so keep the arm where it is
            this->arm_pid_->SetSetpoint(this->state_.rotation);
            this->interface_->arm_lift_power = this->arm_pid_->Calculate(this->interface_->arm_duty_cycle_encoder);

            // and keep bringing the extension in
            this->interface_->arm_extend_power = this->inches_pid_->Calculate(this->interface_->arm_extend_encoder);
        }
    // we've rotated the arm to the desired setpoint, so now extend it
    } else if (control_state_ == EXTENDING) {
        //TODO limit extension?
        //TODO slew rate limiter on the arm power and stuff

        // extend the arm
        this->inches_pid_->SetSetpoint(this->desired_state_.extension);
        this->interface_->arm_extend_power = this->inches_pid_->Calculate(this->interface_->arm_extend_encoder);

        // and keep rotation where it is

        // if the arm is trying to go to 0, and we're close to 0
        if (abs(this->desired_state_.rotation) < 2.0 && abs(this->state_.rotation) < 7.0) {
            // to prevent it from oscillating just set it to 0
            this->interface_->arm_lift_power = 0.0;
        } else {
            // otherwise
            this->interface_->arm_lift_power = this->arm_pid_->Calculate(this->interface_->arm_duty_cycle_encoder);
        }
    } else {
        OKC_CHECK_MSG(false, "arm state machine unknown state");
    }

    return true;
}

void Arm::Periodic() {
    // control the arm either using the raw axis values or PID controllers
    switch (mode_) {
        case Auto:
            VOKC_CALL(this->AutoControl());
            break;
        default:
            VOKC_CHECK_MSG(false, "Unhandled enum");
    }

    // update state
    this->state_.rotation = this->interface_->arm_duty_cycle_encoder;
    this->state_.extension = this->interface_->arm_extend_encoder;

    // shuffleboard value updating
    VOKC_CALL(ArmUI::nt_arm_duty_cycle_encoder->SetDouble(interface_->arm_duty_cycle_encoder));
    VOKC_CALL(ArmUI::nt_arm_setpoint->SetDouble(this->arm_pid_->GetSetpoint()));
    VOKC_CALL(ArmUI::nt_arm_power->SetDouble(interface_->arm_lift_power));

    VOKC_CALL(ArmUI::nt_extend_encoder->SetDouble(interface_->arm_extend_encoder));
    VOKC_CALL(ArmUI::nt_extend_setpoint->SetDouble(this->inches_pid_->GetSetpoint()));
    VOKC_CALL(ArmUI::nt_extend_power->SetDouble(interface_->arm_extend_power));

    VOKC_CALL(ArmUI::nt_limit_switch->SetBoolean(interface_->extend_limit_switch));

    switch(control_state_) {
        case CALIBRATING:
            VOKC_CALL(ArmUI::arm_control_state->SetString("CALIBRATING"));
            break;
        case ROTATING:
            VOKC_CALL(ArmUI::arm_control_state->SetString("ROTATING"));
            break;
        case EXTENDING:
            VOKC_CALL(ArmUI::arm_control_state->SetString("EXTENDING"));
            break;
        default:
            VOKC_CALL(ArmUI::arm_control_state->SetString("unknown"));
    }

    // and log the values
    arm_lift_output_log_.Append(interface_->arm_lift_power);
    arm_lift_enc_log_.Append(interface_->arm_duty_cycle_encoder);
    arm_lift_setpoint_log_.Append(arm_pid_->GetSetpoint());

    arm_extend_output_log_.Append(interface_->arm_extend_power);
    arm_extend_enc_log_.Append(interface_->arm_extend_encoder);
    arm_extend_setpoint_log_.Append(inches_pid_->GetSetpoint());
}