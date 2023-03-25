// Copyright (c) Team OKC Robotics

#pragma once

#include "io/LEDsIO.h"


class LEDs : public frc2::SubsystemBase {
public:
    LEDs(LEDsSoftwareInterface *interface)
        : interface_(interface) {}
    ~LEDs() {}

    bool Init();
    bool SetPattern(double pattern);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs during
     * simulation.
     */
    void SimulationPeriodic() override;

private:
    // software interface
    LEDsSoftwareInterface *const interface_;
};
