#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <io/ArmIO.h>
#include <memory>

#include "Logging.h"
#include "wpi/DataLog.h"

enum ControlMode {
    Manual,
    Auto
};
class Arm : public frc2::SubsystemBase {
public:
    Arm(ArmSoftwareInterface *interface) : interface_(interface) {}
    ~Arm() {}

    bool SetDegrees(double degrees);
    bool SetPreset(double increment);
    bool SetExtend(double inches);
    bool Init();
    bool IncrementExtend(double extend);
    void Periodic() override;
    bool SetManualLiftPower(double power);
    bool SetManualUpPower(double power);
    bool SetManualExtendPower(double power);

    bool SetControlMode(const ControlMode &mode);
    bool ManualControl();
    bool AutoControl();
    


private:
    ArmSoftwareInterface *const interface_;
    std::shared_ptr<frc::PIDController> arm_pid_;
    std::shared_ptr<frc::PIDController> inches_pid_;
    double preset_;
   
    ControlMode mode_;
    
    double lift_power_;
    double up_power_;
    double extend_power_;

    wpi::log::DoubleLogEntry arm_lift_output_log_;
    wpi::log::DoubleLogEntry arm_lift_enc_log_;
    wpi::log::DoubleLogEntry arm_lift_setpoint_log_;

    wpi::log::DoubleLogEntry arm_extend_output_log_;
    wpi::log::DoubleLogEntry arm_extend_enc_log_;
    wpi::log::DoubleLogEntry arm_extend_setpoint_log_;
};
