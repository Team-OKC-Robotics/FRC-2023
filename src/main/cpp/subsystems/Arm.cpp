#include "subsystems/Arm.h"

bool Arm::Init() {
    arm_pid_ = std::make_shared<frc::PIDController>(0, 0, 0);

    inches_pid_ = std::make_shared<frc::PIDController>(0, 0, 0);

    arm_lift_output_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/lift_output");
    arm_lift_enc_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/arm/lift_enc");

    return true;
}
bool Arm::SetControlMode(const ArmMode &mode){
    mode_= mode;

    return true;
}
bool Arm::SetDegrees(double degrees) {
    this->arm_pid_->SetSetpoint(degrees);

    return true;
}


bool Arm::SetExtend(double inches) {
    this->inches_pid_->SetSetpoint(inches);

    return true;
}

bool Arm::SetPreset(double increment) {
    this->arm_pid_->SetSetpoint(increment);

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
    interface_->arm_up_power = up_power_;
    interface_->arm_extend_power = extend_power_;

    arm_lift_output_log_.Append(interface_->arm_lift_power);
    arm_lift_enc_log_.Append(interface_->arm_lift_encoder_val);


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
            //this->interface_->arm_lift_power = this->arm_pid_->Caluculate(this->interface_->arm_encoder);
            //this->interface_->arm_up_power = this->arm_pid_->Caluculate(this->interface_->arm_extend_encoder);
            break;
        default:
            VOKC_CHECK_MSG(false, "Unhandled enum")    
    }
}

// TODO: Understand this example code and implement it correctly
// Note: this is public in the header file
// bool Subsystem::SetPosition(const double &position) {
//     this.position_ = position;
//     return true;
// }

// void Subsystem::Periodic() {
//     // The subsystem does different things based on what mode it is put in
//     // by the user.
//     switch (mode) {
//     case Manual:
//         OKC_CALL(SetUserPower());
//         break;
//     case AutoPosition:
//         OKC_CALL(GoToPosition());
//         break;
//     default:
//         break;
//     }
// }

// bool Subsystem::SetPosition(const double &position) {
//     this.position_ = position;
//     return true;
// }
