#include "subsystems/Intake.h"
#include "Parameters.h"
#include "ui/UserInterface.h"

//pulls PID values from the parameters.toml file
bool Intake::Init() {
    OKC_CHECK(this->interface_ != nullptr);

    // the PID controller for the wrist
    double wrist_kP = RobotParams::GetParam("intake.wrist_pid.kP", 0.005);
    double wrist_kI = RobotParams::GetParam("intake.wrist_pid.kI", 0.0);
    double wrist_kD = RobotParams::GetParam("intake.wrist_pid.kD", 0.0);
    wrist_pid_ = std::make_shared<frc::PIDController>(wrist_kP, wrist_kI, wrist_kD);
    
    // wrist_pid_->SetTolerance(1, 1);

    setpoint_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/tilt/setpoint");
    output_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/tilt/output");
    encoder_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/tilt/encoder");

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

bool Intake::IncrementIntakeTilt(double degrees) {
    OKC_CHECK(this->wrist_pid_ != nullptr);

    this->wrist_pid_->SetSetpoint(this->wrist_pid_->GetSetpoint() + degrees);

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
    
    interface_->tilt_power = -this->wrist_pid_->Calculate(interface_->tilt_encoder);
    // TeamOKC::Clamp(-0.5, 0.5, &interface_->tilt_power);
    interface_->intake_power = intake_power_;
 
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

    VOKC_CALL(IntakeUI::nt_tilt->SetDouble(this->interface_->tilt_encoder));

    setpoint_log_.Append(this->wrist_pid_->GetSetpoint());
    output_log_.Append(this->interface_->tilt_power);
    encoder_log_.Append(this->interface_->tilt_encoder);
}



