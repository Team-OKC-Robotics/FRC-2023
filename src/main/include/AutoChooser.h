#pragma once

#include "Auton.h"
#include "vector"

#include <frc/Joystick.h>
#include "frc2/command/Command.h"


class AutoChooserTeamOKC {
public:
    void AddAutos(std::shared_ptr<Auton> auton);
    void AddGamepad(std::shared_ptr<frc::Joystick> gamepad);
    std::shared_ptr<frc2::Command> GetAutoCommand();
    void Update();

private:
    std::vector<std::shared_ptr<Auton>> autos;
    unsigned int index = 0;

    std::shared_ptr<frc::Joystick> gamepad_;
    bool wasPressed = false;
};