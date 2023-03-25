#pragma once

#include <memory>

#include <frc2/command/SubsystemBase.h>

#include "Utils.h"



typedef struct leds_hardware_interface_t {
} LEDsHardwareInterface;

typedef struct leds_software_interface_t {
    double pattern;
} LEDsSoftwareInterface;

class LEDsIO : public frc2::SubsystemBase {
public:
    LEDsIO(LEDsHardwareInterface *hw_interface,
                 LEDsSoftwareInterface *sw_interface)
        : hw_interface_(hw_interface), sw_interface_(sw_interface) {}

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs during
     * simulation.
     */
    void SimulationPeriodic() override;

    bool ProcessIO();

private:
    LEDsHardwareInterface *const hw_interface_;
    LEDsSoftwareInterface *const sw_interface_;
};