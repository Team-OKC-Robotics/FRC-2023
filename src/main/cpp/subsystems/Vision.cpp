#include "subsystems/Vision.h"

bool Vision::Init() {
    OKC_CALL(ResetSubsystem());
    return true;
}

void Vision::Periodic() {}
void Vision::SimulationPeriodic() {}

bool Vision::GetConeError(double *error) {
    OKC_CHECK(interface_ != nullptr);
    OKC_CHECK(error != nullptr);

    *error = interface_->error;
    return true;
}
bool Vision::GetConeDistance(double *cone) {
    OKC_CHECK(interface_ != nullptr);
    OKC_CHECK(cone != nullptr);

    *cone = interface_->cone;
    return true;
}
bool Vision::GetCubeDistance(double *cube) {
    OKC_CHECK(interface_ != nullptr);
    OKC_CHECK(cube != nullptr);

    *cube = interface_->cube;
    return true;
}

bool Vision::GetCubeAngle(double *angle) {
    OKC_CHECK(interface_ != nullptr);
    OKC_CHECK(angle != nullptr);

    *angle = interface_->angle;
    return true;
}
bool Vision::ResetSubsystem() {
    OKC_CHECK(interface_ != nullptr);

    interface_->reset_subsystem = true;
    return true;
}