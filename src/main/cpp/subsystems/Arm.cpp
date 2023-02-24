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

    return true;
}

bool Arm::SetControlMode(const ArmMode &mode){
    mode_= mode;

    return true;
}
bool Arm::SetDegrees(double degrees) {
    
    OKC_CHECK(this->arm_pid_ != nullptr);
    this->arm_pid_->SetSetpoint(degrees);

    return true;
}

bool Arm::SetExtend(double inches) {
    OKC_CHECK(this->inches_pid_ != nullptr);
    this->inches_pid_->SetSetpoint(inches);

    return true;
}

bool Arm::SetPreset(double increment) {
    OKC_CHECK(this->arm_pid_ != nullptr);
    this->arm_pid_->SetSetpoint(arm_pid_->GetSetpoint() + increment);

    return true;
}

bool Arm::IncrementExtend(double increment) {
    OKC_CHECK(this->arm_pid_ != nullptr);
    this->inches_pid_->SetSetpoint(inches_pid_->GetSetpoint() + increment);

    return true;
}

bool Arm::SetManualLiftPower(double power)
{
    lift_power_ = power;

    return true;
}

bool Arm::SetManualExtendPower(double power)
{
    extend_power_ = power;

    return true;
}

bool Arm::ManualControl() {
    OKC_CHECK(interface_ != nullptr);

    interface_->arm_lift_power = lift_power_;
    interface_->arm_extend_power = extend_power_;

    return true;
}
bool Arm::SetManualUpPower(double power)
{
    up_power_ = power;

    return true;

}

void Arm::Periodic() {
        
    switch (mode_) {
        case Manual:
            VOKC_CALL(this->ManualControl());
            break;
        case Auto:
            VOKC_CHECK(interface_ != nullptr);
            VOKC_CHECK(this->arm_pid_ != nullptr);
            this->interface_->arm_lift_power = this->arm_pid_->Calculate(this->interface_->arm_duty_cycle_encoder);
            this->interface_->arm_extend_power = this->inches_pid_->Calculate(this->interface_->arm_extend_encoder);
            break;
        default:
            VOKC_CHECK_MSG(false, "Unhandled enum");
    }

    VOKC_CALL(ArmUI::nt_arm_duty_cycle_encoder->SetDouble(interface_->arm_duty_cycle_encoder));
    VOKC_CALL(ArmUI::nt_arm_setpoint->SetDouble(this->arm_pid_->GetSetpoint()));
    VOKC_CALL(ArmUI::nt_arm_power->SetDouble(interface_->arm_lift_power));

    VOKC_CALL(ArmUI::nt_extend_encoder->SetDouble(interface_->arm_extend_encoder));
    VOKC_CALL(ArmUI::nt_extend_setpoint->SetDouble(this->inches_pid_->GetSetpoint()));
    VOKC_CALL(ArmUI::nt_extend_power->SetDouble(interface_->arm_extend_power));

    arm_lift_output_log_.Append(interface_->arm_lift_power);
    arm_lift_enc_log_.Append(interface_->arm_duty_cycle_encoder);
    arm_lift_setpoint_log_.Append(arm_pid_->GetSetpoint());

    arm_extend_output_log_.Append(interface_->arm_extend_power);
    arm_extend_enc_log_.Append(interface_->arm_extend_encoder);
    arm_extend_setpoint_log_.Append(inches_pid_->GetSetpoint());
}