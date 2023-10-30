#include "AutoChooser.h"

void AutoChooser::AddAutos(Auto auto) {
    autos = new ArrayList<>();
    for (Auto auto : autons) {
        autos.add(auto);
    }
}

void AutoChooser::addGamepad(frc::Joystick gamepad_) {
    gamepad_ = gamepad_;
}
frc2::CommandBase AutoChooser::getAutoCommand() {
        return autos.at(index);
}

void AutoChooser::update() {
        int pov = gamepad.atPOV();
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
        if (increaseAutoIndex.getBoolean(false)) {
            if (index < autos.size()-1) {
                index += 1;
            }
            increaseAutoIndex.setBoolean(false);
        } else if (decreaseAutoIndex.getBoolean(false)) {
            if (index > 0) {
                index -= 1;
            }
            decreaseAutoIndex.setBoolean(false);
        }
}
    