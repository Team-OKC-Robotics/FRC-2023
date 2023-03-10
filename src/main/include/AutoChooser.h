#include "Auton.h"
#include "vector"

#include <frc/Joystick.h>
#include "frc2/command/Command.h"


class AutoChooserTeamOKC {
private:
    std::vector<std::shared_ptr<Auton>> autos;
    int index = 0;

    std::shared_ptr<frc::Joystick> gamepad_;
    bool wasPressed = false;

public:
    void AddAutos(Auton auton);
    void AddGamepad(std::shared_ptr<frc::Joystick> gamepad);
    std::shared_ptr<frc2::Command> GetAutoCommand();
    void Update();
};