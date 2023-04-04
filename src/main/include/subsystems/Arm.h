#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <io/ArmIO.h>
#include <memory>
#include "Utils.h"

#include "Logging.h"
#include "wpi/DataLog.h"

enum ControlMode {
    Manual,
    Auto,
    Test
};

enum ArmControlState {
    INIT,
    STANDBY,
    CALIBRATING,
    EXTENDING,
    ROTATING,
    DONE
};

class Arm : public frc2::SubsystemBase {
public:
    Arm(ArmSoftwareInterface *interface) : interface_(interface) {}
    ~Arm() {}

    bool Init();
    void Periodic() override;

    // to control the arm
    bool IncrementExtend(double increment);
    bool IncrementRotation(double increment);
    bool SetDesiredState(TeamOKC::ArmState state);

    // for autonomous, to know if we can move on to the next step
    bool AtExtendSetpoint(bool *at);
    bool AtLiftSetpoint(bool *at);

    // control modes
    bool SetControlMode(const ControlMode &mode);
    bool AutoControl();
    bool TestControl();
    bool ManualControl();
    bool AllowCalibration();
    

    bool arm_first_ = false;

private:
    // software interface
    ArmSoftwareInterface *const interface_;

    // PID controllers
    std::shared_ptr<frc::PIDController> arm_pid_;
    std::shared_ptr<frc::PIDController> inches_pid_;
    double arm_kF_;

    // controls stuff (I'd call it a state machine but it isn't really a state machine)
    ControlMode mode_;
    ArmControlState control_state_;
    TeamOKC::ArmState state_;
    TeamOKC::ArmState desired_state_;

    // control flags
    bool has_been_commanded_ = false;
    bool has_calibrated_ = false;
    bool calibration_allowed_ = false; // to avoid accidentally triggering the limit switch during setup
    
    // limits
    double lift_limit_;
    double extend_limit_;

    double lift_power_;
    double extend_power_;

    // logs
    wpi::log::DoubleLogEntry arm_lift_output_log_;
    wpi::log::DoubleLogEntry arm_lift_enc_log_;
    wpi::log::DoubleLogEntry arm_lift_setpoint_log_;

    wpi::log::DoubleLogEntry arm_extend_output_log_;
    wpi::log::DoubleLogEntry arm_extend_enc_log_;
    wpi::log::DoubleLogEntry arm_extend_setpoint_log_;
};
