/// @brief
/// @return 

#include "subsystems/Arm.h"

bool Arm::init(){ 
    Arm_pid = std::make_shared< frc::PIDController>();
    return true;
}

bool Arm::SetDegrees(double degrees){ 
this->Arm_pid->SetSetpoint (degrees);

    return true;
}

bool Arm::SetExtend(double inches){
    return true;
}

bool Arm::Function(int number){
    return true;
}

void Arm::Periodic(){
    this->interface_->arm_lift_power = this->Arm_pid->Calculate(this->interface_->arm_encoder);

}