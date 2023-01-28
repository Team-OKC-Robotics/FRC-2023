#pragma once

#include <frc2/command/SubsystemBase.h>

class ClawSubsystem : public frc2::SubsystemBase {
 public:
  ClawSubsystem();

  bool Init();
  
  bool ResetPositionEncoder();
  bool ResetPositionPID();

  bool SetPosition();
  
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
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
};

  