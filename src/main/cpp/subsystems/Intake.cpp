#include "subsystems/Intake.h"
#include "Parameters.h"
#include "ui/UserInterface.h"

//pulls PID values from the parameters.toml file
bool Intake::Init() {

this->intake_pid_->SetSetpoint(0);


return true;
}

bool Intake::SetControlMode(const ControlMode &mode){
    mode_= mode;

    return true;

}
bool Intake::SetTurn(double degrees) {
    
    OKC_CHECK(this->intake_pid_ != nullptr);
 return true;
}



bool Intake::SetIntakePower(double power) {
    intake_power_ = power;

    return true;
}

bool Intake::ManualControl() {
    OKC_CHECK(interface_ != nullptr);

    interface_->intake_power = intake_power_;
   

    return true;
}


bool Intake::AutoControl() {
    OKC_CHECK(interface_ != nullptr);
    OKC_CHECK(this->intake_pid_ != nullptr);
   return true;
}

void Intake::Periodic() {
        
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
}
    