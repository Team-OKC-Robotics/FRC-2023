#include "subsystems/Intake.h"
#include "Parameters.h"
#include "ui/UserInterface.h"

//pulls PID values from the parameters.toml file
bool Intake::Init() {
    return true;
}



bool Intake::SetIntakePower(double power) {
    intake_power_ = power;

    return true;
}


void Intake::SimulationPeriodic() {
    
}

void Intake::Periodic() {
        
 }




