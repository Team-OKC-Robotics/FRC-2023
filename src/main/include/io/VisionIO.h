#pragma once

#include "frc/Relay.h"
#include "photonlib.h"

typedef struct vision_hardware_interface_t {
    photonlib::PhotonCamera *const camera;
    frc::Relay *const LEDs;
} VisionHardwareInterface;

typedef struct vision_software_interface_t {
    double error;
    double cone;
    double cube;
    double angle;

    // Reset flags
    bool reset_subsystem;
} VisionSoftwareInterface;

class VisionIO : public frc2::SubsystemBase {
public:
    VisionIO(VisionHardwareInterface *hw_interface,
                 VisionSoftwareInterface *sw_interface)
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
    bool UpdateVisionConfig(VisionConfig &config);

    bool ProcessInputs();
    bool SetOutputs();

    VisionHardwareInterface *const hw_interface_;
    VisionSoftwareInterface *const sw_interface_;
};