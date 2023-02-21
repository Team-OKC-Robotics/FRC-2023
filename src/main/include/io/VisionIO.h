#pragma once

#include "frc/Relay.h"

#include <photonlib/PhotonCamera.h>

#include <frc2/command/SubsystemBase.h>

#include "Utils.h"
#include <memory>

typedef struct vision_hardware_interface_t {
    photonlib::PhotonCamera *const camera;
    frc::Relay *const LEDs;
} VisionHardwareInterface;

typedef struct vision_config_t {

} VisionConfig;

typedef struct vision_software_interface_t {
    double error;
    double cone;
    double cube;
    double angle;
    bool reset_subsystem;
    bool update_config;
    VisionConfig vision_config;
} VisionSoftwareInterface;

class VisionIO : public frc2::SubsystemBase {
public:
    VisionIO(VisionHardwareInterface *hw_interface,
             VisionSoftwareInterface *sw_interface)
        : hw_interface_(hw_interface), sw_interface_(sw_interface) {}

    void Periodic() override;

    void SimulationPeriodic() override;

    bool ProcessIO();

private:
    bool UpdateVisionConfig(VisionConfig &config);

    bool ProcessInputs();
    bool SetOutputs();
    VisionHardwareInterface *const hw_interface_;
    VisionSoftwareInterface *const sw_interface_;
};