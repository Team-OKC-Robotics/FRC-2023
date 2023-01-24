#pragma once

#include "frc2/Subsystembase.h"

class Vision : public frc2::Subsystem {
    public:
    Vision(VisionSoftwareInterface *interface)
        : interface_(inetrface) {}


        bool Init();
        void Periodic() override;
        void SimulationPeriodic() override;

        bool GetConeError (double *error);
        bool GetConeDistance (double *cone);
        bool GetCubeDistance (double *cube);
        bool GetCubeAngle (double *angle);
        bool ResetSubsystem ();

        private:
        VisionSoftwareInterface *const interface_;

};