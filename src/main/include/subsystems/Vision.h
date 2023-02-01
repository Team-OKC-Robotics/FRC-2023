// Copyright (c) Team OKC Robotics

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "io/VisionIO.h"

class Vision : public frc2::SubsystemBase {
public:
    Vision(VisionSoftwareInterface *interface)
        : interface_(interface) {}
    ~Vision() {}

    bool Init();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    void SimulationPeriodic() override;

    /*
    a function to get horizontal error between the robot's center and the cone poles
    a function to get the estimated distance to the cone pole
    a function to get distance to the cube scoring location
    a function to get angle of the cube scoring location (iirc there's only an AprilTag on the middle one, so to align with the top one, we'd need to add an offset)
    a function to reset the subsystem (although this doesn't need to currently do anything)
*/

    bool GetConeError(double *error);
    bool GetConeDistance(double *distance);

    bool GetCubeAngle(double *angle);
    bool GetCubeDistance(double *distance);

    bool ResetSubsystem();

private:
    // software interface
    VisionSoftwareInterface *const interface_;
};