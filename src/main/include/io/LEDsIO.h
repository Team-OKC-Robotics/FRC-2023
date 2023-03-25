#pragma once

#include <memory>

#include <frc2/command/SubsystemBase.h>
#include "frc/AddressableLED.h"

#include "Utils.h"



typedef struct leds_hardware_interface_t {
    frc::AddressableLED *const led;
} LEDsHardwareInterface;

typedef struct leds_software_interface_t {
    std::shared_ptr<std::array<frc::AddressableLED::LEDData, 20>> buffer;
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