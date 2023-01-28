<<<<<<< HEAD
=======
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

>>>>>>> c15cb656c009cc713f5a1d80074841e5558f53b3
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

  
