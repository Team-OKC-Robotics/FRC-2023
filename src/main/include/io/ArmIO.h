#pragma once

#include <memory>

#include <frc/DigitalInput.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "frc/AnalogEncoder.h"
#include "frc/DutyCycleEncoder.h"

#include "Utils.h"

typedef struct arm_config_t {
    double open_loop_ramp_rate;
    double max_indexer_current;
} ArmConfig;

typedef struct arm_hardware_interface_t {
    rev::CANSparkMax *const  arm_lift_motor;
    rev::CANSparkMax *const arm_up_motor;
    rev::CANSparkMax *const arm_extend_motor;

    rev::RelativeEncoder *const arm_lift_encoder;
    frc::DutyCycleEncoder *const arm_duty_cycle_encoder;
    rev::RelativeEncoder *const arm_extend_encoder;
    frc::DigitalInput *const extend_limit_switch;
} ArmHardwareInterface;

typedef struct arm_software_interface_t {
    // actuator outputs
    double arm_lift_power;
    double arm_extend_power;

    // sensor inputs
    double arm_encoder;
    double arm_extend_encoder;
    double arm_duty_cycle_encoder;
    bool extend_limit_switch;

    // config
    ArmConfig arm_config;
    bool update_config;
    bool reset_encoders;
} ArmSoftwareInterface;

//arm class
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

    bool Init();

private:
    bool UpdateArmConfig(ArmConfig &config);
    bool ResetEncoders();
    bool SetEncoder(double &val);

    ArmHardwareInterface *const hw_interface_;
    ArmSoftwareInterface *const sw_interface_;

    double offset;
    double lift_limit;
    double extend_limit;
    double max_output;
};