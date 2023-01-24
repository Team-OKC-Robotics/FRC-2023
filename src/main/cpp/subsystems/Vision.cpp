#include "subsystems/Vision.h"

bool Vision::Init() {
    ResetSubsystem();

    return true;
}

void Periodic() {

}

void SimulationPeriodic() {

}

bool GetConeError(double *error) {
    *error = interface_->cone_error;

    return true;
}

bool GetConeDistance(double *distance) {

}

bool GetCubeAngle(double *angle) {

}

bool GetCubeDistance(double *distance) {

}

bool ResetSubsystem() {
    interface_->reset_subsystem = true;

    return true;
}