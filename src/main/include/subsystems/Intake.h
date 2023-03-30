#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <io/IntakeIO.h>
#include <memory>
#include "subsystems/Arm.h"

#include "Logging.h"
#include "wpi/DataLog.h"
#include "ui/UserInterface.h"
#include "wpi/DataLog.h"


class Intake : public frc2::SubsystemBase {
public:
    Intake(IntakeSoftwareInterface *interface) : interface_(interface) {}
    ~Intake() {}

    bool Init();

    bool SetIntakePower(double power);
    bool SetIntakeTilt(double tilt);
    bool GetIntakeTilt(double *tilt);
    bool IncrementIntakeTilt(double inc);

    bool SetControlMode(const ControlMode &mode);
    bool ManualControl();
    bool AutoControl();
    bool AtSetpoint(bool *at);

    bool Reset();

    void Periodic() override;
    void SimulationPeriodic() override;

private:
    IntakeSoftwareInterface *const interface_;

    std::shared_ptr<frc::PIDController> wrist_pid_;
   
    double intake_power_;
   
    ControlMode mode_;

    wpi::log::DoubleLogEntry setpoint_log_;
    wpi::log::DoubleLogEntry output_log_;
    wpi::log::DoubleLogEntry encoder_log_;
};
