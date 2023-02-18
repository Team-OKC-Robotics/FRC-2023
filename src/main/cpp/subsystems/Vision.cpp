#include "subsystems/Vision.h"

bool Vision::Init() {
    ResetSubsystem();
    return true;
}

void Vision::Periodic() {}
void Vision::SimulationPeriodic() {}

bool Vision::GetConeError(double *error) {
    *error = interface_->error;
    return true;
}
bool Vision::GetConeDistance(double *cone) {
    *cone = interface_->cone;
    return true;
}
bool Vision::GetCubeDistance(double *cube) {
    *cube = interface_->cube;
    return true;
}

bool Vision::GetCubeAngle(double *angle) {
    *angle = interface_->angle;
    return true;
}
bool Vision::ResetSubsystem() {
    interface_->reset_subsystem = true;
    return true;
}