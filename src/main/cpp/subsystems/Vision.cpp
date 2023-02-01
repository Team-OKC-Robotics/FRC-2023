#include "subsystems/Vision.h"

bool Vision::Init() {
    ResetSubsystem();

    return true;
}

void Vision::Periodic() {

}

void Vision::SimulationPeriodic() {

}

bool Vision::GetConeError(double *error) {
    *error = interface_->cone; // cone error

    return true;
}

bool Vision::GetConeDistance(double *distance) {
    return true;
}

bool Vision::GetCubeAngle(double *angle) {
    return true;
}

bool Vision::GetCubeDistance(double *distance) {
    return true;
}

bool Vision::ResetSubsystem() {
    interface_->reset_subsystem = true;

    return true;
}