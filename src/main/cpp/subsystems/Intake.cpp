#include "subsystems/Intake.h"
#include "Parameters.h"
#include "ui/UserInterface.h"

//pulls PID values from the parameters.toml file
bool Intake::Init() {
    return true;
}

bool Intake::SetControlMode(const ControlMode &mode) {
    mode_= mode;

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

void Intake::SimulationPeriodic() {
    
}

void Intake::Periodic() {
        
    switch (mode_) {
        case Manual:
            VOKC_CALL(this->ManualControl());
            break;
        default:
            VOKC_CHECK_MSG(false, "Unhandled enum");
    }
}



