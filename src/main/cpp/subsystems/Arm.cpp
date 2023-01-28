#include "subsystems/Arm.h"

bool Arm::Init() {
    arm_pid_ = std::make_shared<frc::PIDController>(0, 0, 0);

    inches_pid_ = std::make_shared<frc::PIDController>(0, 0, 0);
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

void Arm::Periodic() {
    this->interface_->arm_lift_power =
        this->arm_pid_->Calculate(this->interface_->arm_encoder);
    this->interface_->arm_extend_power =
        this->inches_pid_->Calculate(this->interface_->arm_extend_encoder);
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
