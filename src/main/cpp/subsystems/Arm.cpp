#include "subsystems/Arm.h"
#include "Parameters.h"
#include "ui/UserInterface.h"

//pulls PID values from the parameters.toml file
bool Arm::Init() {
    
    double arm_kP = RobotParams::GetParam("arm.lift_pid.kP", 0.005);
    double arm_kI = RobotParams::GetParam("arm.lift_pid.kI", 0.0);
    double arm_kD = RobotParams::GetParam("arm.lift_pid.kD", 0.0);

    double extend_kP = RobotParams::GetParam("arm.extend_pid.kP", 0.005);
    double extend_kI = RobotParams::GetParam("arm.extend_pid.kI", 0.0);
    double extend_kD = RobotParams::GetParam("arm.extend_pid.kD", 0.0);

    arm_pid_ = std::make_shared<frc::PIDController>(arm_kP, arm_kI, arm_kD);

    inches_pid_ = std::make_shared<frc::PIDController>(extend_kP, extend_kI, extend_kD);

    arm_lift_output_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/lift_output");
    arm_lift_enc_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/lift_enc");
    arm_lift_setpoint_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/lift_setpoint");

    arm_extend_output_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/extend_output");
    arm_extend_enc_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/extend_enc");
    arm_extend_setpoint_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/extend_setpoint");

    this->arm_pid_->SetSetpoint(0);
    this->inches_pid_->SetSetpoint(0);

    lift_limit_ = RobotParams::GetParam("arm.lift_limit", 100);
    extend_limit_ = RobotParams::GetParam("arm.extend_limit", 100);

    offset_ = RobotParams::GetParam("arm.offset", -330);
     

    return true;
}

bool Arm::SetControlMode(const ControlMode &mode){
    mode_= mode;

    return true;
}

bool Arm::SetDegrees(double degrees) {
    
    OKC_CHECK(this->arm_pid_ != nullptr);

    // limit the lift PID setpoint to the actual hardware limits
    if (degrees < lift_limit_) {
        if (degrees > -lift_limit_) {
            this->arm_pid_->SetSetpoint(degrees);
        } else {
            this->arm_pid_->SetSetpoint(-lift_limit_);
        }
    } else {
        this->arm_pid_->SetSetpoint(lift_limit_);
    }

    return true;
}

bool Arm::SetExtend(double inches) {
    OKC_CHECK(this->inches_pid_ != nullptr);

    // limit the extend PID setpoint to the actual hardware limits
    if (inches < extend_limit_) {
        if (inches > 0) {
            this->inches_pid_->SetSetpoint(inches);
        } else {
            this->inches_pid_->SetSetpoint(0);
        }
    } else {
        this->inches_pid_->SetSetpoint(extend_limit_);
    }


    return true;
}

bool Arm::SetPreset(double increment) {
    OKC_CHECK(this->arm_pid_ != nullptr);

    // limit the lift PID setpoint to the actual hardware limits
    if (arm_pid_->GetSetpoint() + increment < lift_limit_) {
        if (arm_pid_->GetSetpoint() + increment > -lift_limit_) {
            this->arm_pid_->SetSetpoint(arm_pid_->GetSetpoint() + increment);
        } else {
            this->arm_pid_->SetSetpoint(-lift_limit_);
        }
    } else {
        this->arm_pid_->SetSetpoint(lift_limit_);
    }

    return true;
}

bool Arm::IncrementExtend(double increment) {
    OKC_CHECK(this->inches_pid_ != nullptr);

    // limit the extend PID setpoint to the actual hardware limits
    if (inches_pid_->GetSetpoint() + increment  <  extend_limit_) {
        if (inches_pid_->GetSetpoint() + increment > 0) {
            this->inches_pid_->SetSetpoint(inches_pid_->GetSetpoint() + increment);
        } else {
            this->inches_pid_->SetSetpoint(0);
        }
    } else {
        this->inches_pid_->SetSetpoint(extend_limit_);
    }

    return true;
}

bool Arm::SetManualLiftPower(double power) {
    lift_power_ = power;

    return true;
}

bool Arm::SetManualExtendPower(double power) {
    extend_power_ = power;

    return true;
}

bool Arm::ManualControl() {
    OKC_CHECK(interface_ != nullptr);

    interface_->arm_lift_power = lift_power_;
    interface_->arm_extend_power = extend_power_;

    return true;
}

bool Arm::SetManualUpPower(double power) {
    up_power_ = power;

    return true;

}

bool Arm::AutoControl() {
    OKC_CHECK(interface_ != nullptr);
    OKC_CHECK(this->arm_pid_ != nullptr);
    OKC_CHECK(this->inches_pid_ != nullptr);

    // if the setpoint is 0 (ie the arm hasn't been set to go somewhere yet, like it's just been enabled)
    if (this->arm_pid_->GetSetpoint() == 0) {
        // don't do anything
        this->interface_->arm_lift_power = 0;
    } else { // otherwise
        // PID to the setpoint
        this->interface_->arm_lift_power = this->arm_pid_->Calculate(this->interface_->arm_duty_cycle_encoder);
    }

    // this doesn't apply to extend because it is unlikely we would break something with this *knock on wood*
    this->interface_->arm_extend_power = this->inches_pid_->Calculate(this->interface_->arm_extend_encoder);


    return true;
}

void Arm::Periodic() {
        
    switch (mode_) {
        case Manual:
            VOKC_CALL(this->ManualControl());
            break;
        case Auto:
            VOKC_CALL(this->AutoControl());
            break;
        default:
            VOKC_CHECK_MSG(false, "Unhandled enum");
    }

    // shuffleboard value updating
    VOKC_CALL(ArmUI::nt_arm_duty_cycle_encoder->SetDouble(interface_->arm_duty_cycle_encoder));
    VOKC_CALL(ArmUI::nt_arm_setpoint->SetDouble(this->arm_pid_->GetSetpoint()));
    VOKC_CALL(ArmUI::nt_arm_power->SetDouble(interface_->arm_lift_power));

    VOKC_CALL(ArmUI::nt_extend_encoder->SetDouble(interface_->arm_extend_encoder));
    VOKC_CALL(ArmUI::nt_extend_setpoint->SetDouble(this->inches_pid_->GetSetpoint()));
    VOKC_CALL(ArmUI::nt_extend_power->SetDouble(interface_->arm_extend_power));

    // and log the values
    arm_lift_output_log_.Append(interface_->arm_lift_power);
    arm_lift_enc_log_.Append(interface_->arm_duty_cycle_encoder * 360 + offset_);
    arm_lift_setpoint_log_.Append(arm_pid_->GetSetpoint());

    arm_extend_output_log_.Append(interface_->arm_extend_power);
    arm_extend_enc_log_.Append(interface_->arm_extend_encoder);
    arm_extend_setpoint_log_.Append(inches_pid_->GetSetpoint());
}