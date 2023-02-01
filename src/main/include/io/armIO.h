#pragma once

#include <memory>

#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>

#include "Utils.h"


typedef struct Arm_config_t {
    
} ArmConfig;

typedef struct arm_hardware_interface_t {
     std::unique_ptr<rev::CANSparkMax> arm_lift_motor;
    std::unique_ptr<rev::CANSparkMax> arm_up_motor;
    std::unique_ptr<rev::CANSparkMax> arm_extend_motor;
} ArmHardwareInterface;

typedef struct arm_software_interface_t {
    
    // actuator outputs
    double arm_lift_power;
    double arm_up_power;
    double arm_extend_power;
} ArmSoftwareInterface;

class ArmIO : public frc2::SubsystemBase {
public:
    ArmIO(ArmHardwareInterface *hw_interface,
                 ArmSoftwareInterface *sw_interface)
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
    bool UpdateArmConfig(ArmConfig &config);
    bool ResetEncoders();
    bool SetEncoder(double &val);

    ArmHardwareInterface *const hw_interface_;
    ArmSoftwareInterface *const sw_interface_;
};