#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <io/ArmIO.h>
#include <memory>

class Arm : public frc2::SubsystemBase {
public:
    Arm(ArmSoftwareInterface *interface) : interface_(interface) {}
    ~Arm() {}

    bool SetDegrees(double degrees);
    bool SetExtend(double inches);
    bool Init();
    void Periodic() override;

private:
    ArmSoftwareInterface *const interface_;
    std::shared_ptr<frc::PIDController> arm_pid_;
    std::shared_ptr<frc::PIDController> inches_pid_;
    double preset_;
};
