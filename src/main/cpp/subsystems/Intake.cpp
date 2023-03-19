#include "subsystems/Intake.h"
#include "Parameters.h"
#include "ui/UserInterface.h"

//pulls PID values from the parameters.toml file
bool Intake::Init() {
    OKC_CHECK(this->interface_ != nullptr);

    // the PID controller for the wrist
    double wrist_kP = RobotParams::GetParam("intale.wrist_pid.kP", 0.005);
    double wrist_kI = RobotParams::GetParam("intale.wrist_pid.kI", 0.0);
    double wrist_kD = RobotParams::GetParam("intale.wrist_pid.kD", 0.0);
    wrist_pid_ = std::make_shared<frc::PIDController>(wrist_kP, wrist_kI, wrist_kD);
    
    // wrist_pid_->SetTolerance(1, 1);

    //TODO verify that this is a good idea
    this->wrist_pid_->SetSetpoint(0.0);

    return true;
}

bool Intake::Reset() {
    OKC_CHECK(this->interface_ != nullptr);

    this->wrist_pid_->Reset();

    return true;
}

bool Intake::SetIntakePower(double power) {
    intake_power_ = power;

    return true;
}

bool Intake::SetIntakeTilt(double degrees) {
    OKC_CHECK(this->wrist_pid_ != nullptr);

    this->wrist_pid_->SetSetpoint(degrees);
    
    return true;
}

bool Intake::SetControlMode(const ControlMode &mode){
    mode_= mode;

    return true;
}

bool Intake::ManualControl() {
    OKC_CHECK(interface_ != nullptr);

    interface_->intake_power = intake_power_;
   
    return true;
}

bool Intake::AutoControl() {
    OKC_CHECK(interface_ != nullptr);
    OKC_CHECK(this->wrist_pid_ != nullptr);
    
    interface_->tilt_power = this->wrist_pid_->Calculate(interface_->tilt_encoder);
 
    return true;
}

void Intake::SimulationPeriodic() {
    
}

void Intake::Periodic() {
    switch (mode_) {
        case Manual:
            VOKC_CALL(this->ManualControl());
            break;
        case Auto:
            VOKC_CALL(this->AutoControl());
            break;
        default:
            VOKC_CHECK_MSG(false, "unhandled intake enum");
    }
}



