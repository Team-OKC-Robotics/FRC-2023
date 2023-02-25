#include "subsystems/Arm.h"

bool Arm::Init() {
    arm_pid_ = std::make_shared<frc::PIDController>(0, 0, 0);

    inches_pid_ = std::make_shared<frc::PIDController>(0, 0, 0);

    // Initialize the logger for the Arm.
    arm_logger.Init("arm_log");

    return true;
}

bool Arm::SetControlMode(const ArmMode &mode) {
    mode_ = mode;

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
    interface_->arm_up_power = up_power_;
    interface_->arm_extend_power = extend_power_;

    // Log arm data.
    arm_logger.Log("/arm/lift_output", interface_->arm_lift_power);
    arm_logger.Log("/arm/lift_enc", interface_->arm_lift_encoder_val);

    return true;
}
bool Arm::SetManualUpPower(double power) {
    up_power_ = power;

    return true;
}

void Arm::Periodic() {

    switch (mode_) {
    case Manual:
        VOKC_CALL(this->ManualControl());
        break;
    case Auto:
        // this->interface_->arm_lift_power =
        // this->arm_pid_->Caluculate(this->interface_->arm_encoder);
        // this->interface_->arm_up_power =
        // this->arm_pid_->Caluculate(this->interface_->arm_extend_encoder);
        break;
    default:
        VOKC_CHECK_MSG(false, "Unhandled enum")
    }
}
