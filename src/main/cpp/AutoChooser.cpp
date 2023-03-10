#include "AutoChooser.h"

void AutoChooser::AddAutos(Auto auto) {
    autos = new ArrayList<>();
    for (Auto auto : autons) {
        autos.add(auto);
    }
}

void addGamepad::(Joystick gamepad_) {
    gamepad = gamepad_;
}
void Command::getAutoCommand() {
        return autos.get(index);
}

void update::() {
        int pov = gamepad.getPOV();
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