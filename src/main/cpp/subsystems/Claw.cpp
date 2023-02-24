#include "subsystems/Claw.h"
#include "Parameters.h"
#include "Utils.h"
#include "ui/UserInterface.h"

bool Claw::Init() {
    double kP = RobotParams::GetParam("claw.claw_pid.kP", 0.01);
    double kI = RobotParams::GetParam("claw.claw_pid.kI", 0.0);
    double kD = RobotParams::GetParam("claw.claw_pid.kD", 0.0);

    claw_pid_ = std::make_shared<frc::PIDController>(kP, kI, kD);
    claw_power_ = 0;

    claw_output_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/claw/claw_output");
    claw_enc_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/claw/claw_enc");
    claw_setpoint_log_ = wpi::log::DoubleLogEntry(TeamOKC::log, "/claw/claw_setpoint");

    open_pos_ = RobotParams::GetParam("claw.open", 0);
    cube_pos_ = RobotParams::GetParam("claw.cube", 0);
    cone_pos_ = RobotParams::GetParam("claw.cone", 0);

    this->Reset();

    return true;
}

bool Claw::ResetPositionEncoder() {
    interface_->reset_claw_open_and_close = true;

    return true;
}

bool Claw::ResetPositionPID() {
    claw_pid_->Reset();

    return true;
}

bool Claw::SetPreset(ClawPreset preset) {
    if (preset == OPEN) {
        this->SetPosition(open_pos_);
    } else if (preset == CUBE) {
        this->SetPosition(cube_pos_);
    } else if (preset == CONE) {
        this->SetPosition(cone_pos_);
    } else {
        OKC_CHECK_MSG(false, "unhandled claw enum value");
    }

    return true;
}

bool Claw::SetPosition(double pos) {
    claw_pid_->SetSetpoint(pos);

    return true;
}

bool Claw::Reset() {
    this->ResetPositionEncoder();
    this->ResetPositionPID();

    return true;
}

void Claw::Periodic() {
    interface_->claw_open_and_close_power = this->claw_pid_->Calculate(interface_->encoder_reading);

    // logs
    claw_output_log_.Append(interface_->claw_open_and_close_power);
    claw_enc_log_.Append(interface_->encoder_reading);
    claw_setpoint_log_.Append(claw_pid_->GetSetpoint());

    // shuffleboard
    VOKC_CALL(ClawUI::nt_claw_encoder->SetDouble(interface_->encoder_reading));
}

void Claw::SimulationPeriodic() {
    // do nothing
    return;
}
