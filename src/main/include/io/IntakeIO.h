#pragma once

#include <memory>

#include <frc/DigitalInput.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "frc/AnalogEncoder.h"
#include "frc/DutyCycleEncoder.h"

#include "Utils.h"

typedef struct intake_config_t {
    double open_loop_ramp_rate;
    double max_indexer_current;
} IntakeConfig;

typedef struct intake_hardware_interface_t {
    rev::CANSparkMax *const  intake_motor;
  

    rev::RelativeEncoder *const intake_encoder;
 
} IntakeHardwareInterface;

typedef struct intake_software_interface_t {
    // actuator outputs
    double intake_power;
    


    // sensor inputs
    double intake_encoder;
  

    // config
    IntakeConfig intake_config;
    bool update_config;
    bool reset_encoders;
} IntakeSoftwareInterface;

//arm class
class IntakeIO : public frc2::SubsystemBase {
public:
    IntakeIO(IntakeHardwareInterface *hw_interface,
          IntakeSoftwareInterface *sw_interface)
        : hw_interface_(hw_interface), sw_interface_(sw_interface) {}

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    void SimulationPeriodic() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs during
     * simulation.
     */
   

    bool ProcessIO();

    bool Init();

private:
    bool UpdateIntakeConfig(IntakeConfig &config);
    bool ResetEncoders();
    bool SetEncoder(double &val);

    IntakeHardwareInterface *const hw_interface_;
    IntakeSoftwareInterface *const sw_interface_;

    double offset;
    double lift_limit;
    double extend_limit;
    double max_output;
};