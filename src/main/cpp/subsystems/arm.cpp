#include "subsystems/Arm.h"

bool Arm::init(){ 
    Arm_pid = std::make_shared<frc::PIDController>(0, 0, 0);

    Inches_pid = std::make_shared<frc::PIDController>(0,0,0);
    return true;
}

bool Arm::SetDegrees(double degrees){ 
this->Arm_pid->SetSetpoint (degrees);

    return true;
}

bool Arm::SetExtend(double inches){
this->Inches_pid->SetSetpoint (inches);
   
    return true;
}


void Arm::Periodic(){
    this->interface_->arm_lift_power = this->Arm_pid->Calculate(this->interface_->arm_encoder);
    this->interface_->arm_extend_power = this->Inches_pid->Calculate(this->interface_->arm_extend_encoder);

}