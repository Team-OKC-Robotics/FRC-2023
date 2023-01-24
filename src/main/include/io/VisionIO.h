#pragma once

#include "frc/Relay.h"

typedef struct vision_hardware_interface_t {
photonlib::PhotonCamera *const camera;
frc::Relay *const LEDs;
} VisionHardwareInterface;

typedef struct vision_hardware_interface_t {
    double error;
    double cone;
    double cube;
    double angle;
    bool reset_subsystem;
} VisionSoftwareInterface;

class VisionIO : public frc2::SubsystemBase {
    public:
    VisionIO(VisionHardwareInterface *hw_intereface,
                  VisionSoftwareInterface *sw_interface)
                  :hw_interface_(hw_interface), sw_interface_(sw_interface) {}

                  void Periodic() override;

                  void SimulationPeriodic()P override;

                  bool ProcessIO();

            private:
                bool UpdateVisionConfig(VisionConfig &config);

                bool ProcessInputs();
                bool SetOutputs();
                VisionHardwareInterface *const hw_interface_;
                VisionSoftwareInterface (const sw_interface_;)
};