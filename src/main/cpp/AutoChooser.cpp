#include "AutoChooser.h"

void AutoChooserTeamOKC::AddAutos(Auton auton) {
    // autos.push_back(auton);
}

void AutoChooserTeamOKC::AddGamepad(std::shared_ptr<frc::Joystick> gamepad) {
    gamepad_ = gamepad;
}

std::shared_ptr<frc2::Command> AutoChooserTeamOKC::GetAutoCommand() {
    return autos.at(index);
}

void AutoChooserTeamOKC::Update() {
    int pov = gamepad_->GetPOV();

    if(pov == -1) {
        wasPressed = false;
    } else if (!wasPressed) {
        if(pov == 180 && index < autos.size()-1) { 
            index += 1;
            wasPressed = true;
        } else if (pov == 0 && index > 0) {
            index -= 1;
            wasPressed = true;
        }
    }
}