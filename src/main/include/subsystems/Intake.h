#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <io/IntakeIO.h>
#include <memory>
#include "subsystems/Arm.h"

#include "Logging.h"
#include "wpi/DataLog.h"

class Intake : public frc2::SubsystemBase {
public:
    Intake(IntakeSoftwareInterface *interface) : interface_(interface) {}
    ~Intake() {}

    bool SetControlMode(const ControlMode &mode);
    bool Init();
    bool SetTurn(double degrees);
    bool ManualControl();
    bool AutoControl();
    bool SetIntakePower(double power);
    void Periodic() override;
    void SimulationPeriodic() override;
   
    


   
    


private:
    IntakeSoftwareInterface *const interface_;
    std::shared_ptr<frc::PIDController> intake_pid_;
   
    double preset_;
    double intake_power_;

   
    ControlMode mode_;
    
};
