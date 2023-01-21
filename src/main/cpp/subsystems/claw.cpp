#pragma once

#include <frc2/command/SubsystemBase.h>
#include "subsystems/claw.h"

bool Claw::init() { 
    Claw_pid = std::make_shared<frc::PIDController>();
    return true;
}

bool Claw::SetPosition(double inches){ 
this->Claw_pid->SetSetpoint (inches);

    return true;
}

   void Arm::Periodic(){
    this->interface_->claw_lift_power = this->Claw_pid->Calculate(this->interface_->claw_encoder);

    }
