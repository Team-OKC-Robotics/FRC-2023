#include "subsystems/Claw.h"

bool Claw::Init(){
    return true;
}

bool Claw::ResetPositionEncoder(){
    return true;
}
bool Claw::ResetPositionPID(){
    return true;
}
bool Claw::SetPosition(){
    // SetPosition (30); //cone
    // SetPosition (0); //close
    // SetPosition (50);//cube
    return true;
}
bool Claw::Reset(){
    return true;
}


bool Claw::ExpandClaw(double distance){
    this->claw_pid_->SetSetpoint(distance);

    return true;

}
void Claw::Periodic(){
    return;
}
void Claw::SimulationPeriodic(){
    return;
}
