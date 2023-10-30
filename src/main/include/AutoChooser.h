#include "Auto.h"
#include <frc/Joystick.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class AutoChooser {
private:
    frc2::CommandBase<Auto> autos;
     int index = 0;
    frc::Joystick gamepad;
    bool wasPressed = false;

    public:
    void AddAutos(Auto auto);
public:
void addGamepad(frc::Joystick gamepad_);
frc2::CommandBase getAutoCommand();
void update();
}

class Auto {  
bool increaseAutoIndex;
bool decreaseAutoIndex;
}
class JoystickGamepad {
    bool Joystick;
    bool Joystickgamepad_;
}