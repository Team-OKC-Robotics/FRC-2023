#include "Auto.h"
class AutoChooser {
private:
    ArrayList<Auto> autos;
     int index = 0;
    Joystick gamepad;
    private bool wasPressed = false;

    public:
    void AddAutos(Auto auto);
public:
void addGamepad(JoystickGamepad_);
void Command getAutoCommand();
void update();
}