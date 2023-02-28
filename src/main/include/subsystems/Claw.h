#pragma once
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <io/ClawIO.h>
#include <memory>



class Claw : public frc2::SubsystemBase {
 public:
  Claw();
  ~Claw();

  bool Init();
  
  bool ResetPositionEncoder();
  bool ResetPositionPID();
  bool ExpandClaw(double distance);

  bool SetPosition();
  bool Reset();
  
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
    double preset_;

 



  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
};

  