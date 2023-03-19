#pragma once

#include <frc2/command/SubsystemBase.h>
#include "io/ClawIO.h"
#include <frc/controller/PIDController.h>
#include <memory>
#include "Logging.h"
#include "wpi/DataLog.h"
#include "subsystems/Arm.h"

enum ClawPreset {
  CONE,
  CUBE,
  OPEN
};

class Claw : public frc2::SubsystemBase {
 public:
  Claw(ClawSoftwareInterface *interface) : interface_(interface) {}
  ~Claw() {}

  bool Init();
  
  bool ResetPositionEncoder();
  bool ResetPositionPID();

  bool SetPreset(ClawPreset preset);
  bool SetPosition(double pos);
  bool Reset();

  bool SetManualPower(double power);
  bool SetControlMode(ControlMode mode);
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  ClawSoftwareInterface *const interface_;
  std::shared_ptr<frc::PIDController> claw_pid_;

  ControlMode mode_;

  double claw_power_;
  double open_pos_;
  double cube_pos_;
  double cone_pos_;

  wpi::log::DoubleLogEntry claw_output_log_;
  wpi::log::DoubleLogEntry claw_enc_log_;
  wpi::log::DoubleLogEntry claw_setpoint_log_;
};

  
