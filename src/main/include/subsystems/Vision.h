#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>

#include "io/VisionIO.h"

class Vision : public frc2::SubsystemBase {
public:
    Vision(VisionSoftwareInterface *interface) : interface_(interface) {}

    bool Init();
    void Periodic() override;
    void SimulationPeriodic() override;

    bool Reset();

    bool GetCubeDistance(double *cube);
    bool GetCubeAngle(double *angle);

    bool GetCubeOutput(double *output);
    bool AtSetpoint(bool *at);
private:
    VisionSoftwareInterface *const interface_;

    std::shared_ptr<frc::PIDController> vision_pid_;
};