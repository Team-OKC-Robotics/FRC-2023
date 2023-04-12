#include "subsystems/Vision.h"
#include "Parameters.h"

bool Vision::Init() {
    double vision_kp = RobotParams::GetParam("vision.kP", 0.01);
    double vision_ki = RobotParams::GetParam("vision.kI", 0.0);
    double vision_kd = RobotParams::GetParam("vision.kD", 0.0);
    vision_pid_ = std::make_shared<frc::PIDController>(vision_kp, vision_ki, vision_kd);

    OKC_CALL(Reset());

    return true;
}

bool Vision::Reset() {
    //TODO

    vision_pid_->Reset();

    return true;
}

void Vision::Periodic() {

}

void Vision::SimulationPeriodic() {

}


bool Vision::GetCubeDistance(double *cube) {
    OKC_CHECK(interface_ != nullptr);
    OKC_CHECK(cube != nullptr);

    *cube = interface_->cube_dist;

    return true;
}

bool Vision::GetCubeAngle(double *angle) {
    OKC_CHECK(interface_ != nullptr);
    OKC_CHECK(angle != nullptr);

    *angle = interface_->cube_yaw;

    return true;
}

bool Vision::GetCubeOutput(double *output) {
    OKC_CHECK(vision_pid_ != nullptr);

    double angle = 0.0;
    OKC_CALL(GetCubeAngle(&angle));

    *output = vision_pid_->Calculate(angle);

    return true;
}

bool Vision::AtSetpoint(bool *at) {
    OKC_CHECK(vision_pid_ != nullptr);

    *at = vision_pid_->GetSetpoint();

    return true;
}