#pragma once

#include <memory>

#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>

#include "Utils.h"


typedef struct claw_config_t {
    
} ClawConfig;

typedef struct claw_hardware_interface_t {
    rev::CANSparkMax *const claw_open_and_close_motor;
    rev:SparkMaxRelativeEncoder *const claw_open_and_close_encoder;
} ClawHardwareInterface;

typedef struct claw_software_interface_t {
    
    // actuator outputs
    double claw_open_and_close_power; 
    double encoder_reading;
    bool reset_claw_open_and_close;
} ClawSoftwareInterface;

class ClawIO : public frc2::SubsystemBase {
public:
    ClawIO(ClawHardwareInterface *hw_interface,
                 ClawSoftwareInterface *sw_interface)
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
    ClawHardwareInterface *const hw_interface_;
    ClawSoftwareInterface *const sw_interface_;
};