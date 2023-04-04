
#include "io/SwerveDriveIO.h"
#include "Utils.h"

void SwerveDriveIO::Periodic() {
    // Process all the inputs and outputs to/from high level software.
    VOKC_CALL(ProcessIO());
}

void SwerveDriveIO::SimulationPeriodic() {
    // SimulationPeriodic
}

bool SwerveDriveIO::ProcessIO() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);

    // Set the software outputs
    // If the swerve drive configuration needs to be updated, update it.
    if (sw_interface_->update_config) {
        OKC_CALL(UpdateDriveConfig(sw_interface_->drive_config));

        // Lower the update flag
        sw_interface_->update_config = false;
    }

    // If the encoders should be reset, reset them
    if (sw_interface_->reset_drive_encoders) {
        OKC_CALL(ResetDriveEncoders());

        // Lower the encoder reset flag
        sw_interface_->reset_drive_encoders = false;
    }

    if (sw_interface_->reset_steer_encoders) {
        OKC_CALL(ResetSteerEncoders());

        // Lower the encoder reset flag
        sw_interface_->reset_steer_encoders = false;
    }

    // ===/flags===

    // If the navX should be reset, reset it.
    if (sw_interface_->reset_gyro) {
        OKC_CHECK(hw_interface_ != nullptr);
        hw_interface_->ahrs->Reset();

        // Lower the navX reset flag
        sw_interface_->reset_gyro = false;
    }

    OKC_CALL(ProcessInputs());
    OKC_CALL(SetOutputs());

    return true;
}

bool SwerveDriveIO::UpdateDriveConfig(SwerveDriveConfig &config) {
    OKC_CHECK(hw_interface_ != nullptr);

    OKC_CHECK(hw_interface_->left_front_drive_motor != nullptr);
    OKC_CHECK(hw_interface_->left_back_drive_motor != nullptr);
    OKC_CHECK(hw_interface_->right_front_drive_motor != nullptr);
    OKC_CHECK(hw_interface_->right_back_drive_motor != nullptr);

    OKC_CHECK(hw_interface_->left_front_steer_motor != nullptr);
    OKC_CHECK(hw_interface_->left_back_steer_motor != nullptr);
    OKC_CHECK(hw_interface_->right_front_steer_motor != nullptr);
    OKC_CHECK(hw_interface_->right_back_steer_motor != nullptr);

    // Get the configuration
    double open_loop_ramp_drive = config.open_loop_ramp_rate_drive;
    double open_loop_ramp_steer = config.open_loop_ramp_rate_steer;
    double max_output_drive = config.max_output_drive;
    double max_output_steer = config.max_output_steer;
    double current_limit = config.current_limit;

    // Apply the configuration
    // Open Loop Ramp Rate
    hw_interface_->left_front_drive_motor->SetOpenLoopRampRate(open_loop_ramp_drive);
    hw_interface_->left_back_drive_motor->SetOpenLoopRampRate(open_loop_ramp_drive);
    hw_interface_->right_front_drive_motor->SetOpenLoopRampRate(open_loop_ramp_drive);
    hw_interface_->right_back_drive_motor->SetOpenLoopRampRate(open_loop_ramp_drive);

    hw_interface_->left_front_steer_motor->SetOpenLoopRampRate(open_loop_ramp_steer);
    hw_interface_->left_back_steer_motor->SetOpenLoopRampRate(open_loop_ramp_steer);
    hw_interface_->right_front_steer_motor->SetOpenLoopRampRate(open_loop_ramp_steer);
    hw_interface_->right_back_steer_motor->SetOpenLoopRampRate(open_loop_ramp_steer);

    hw_interface_->left_front_drive_motor->SetIdleMode(config.idle_mode);
    hw_interface_->left_back_drive_motor->SetIdleMode(config.idle_mode);
    hw_interface_->right_front_drive_motor->SetIdleMode(config.idle_mode);
    hw_interface_->right_back_drive_motor->SetIdleMode(config.idle_mode);

    hw_interface_->left_front_steer_motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    hw_interface_->left_back_steer_motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    hw_interface_->right_front_steer_motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    hw_interface_->right_back_steer_motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    hw_interface_->left_back_drive_motor->SetSmartCurrentLimit(current_limit);
    hw_interface_->left_front_steer_motor->SetSmartCurrentLimit(current_limit);
    hw_interface_->right_back_drive_motor->SetSmartCurrentLimit(current_limit);
    hw_interface_->right_front_drive_motor->SetSmartCurrentLimit(current_limit);

    hw_interface_->left_front_drive_motor->SetSmartCurrentLimit(current_limit);
    hw_interface_->left_back_steer_motor->SetSmartCurrentLimit(current_limit);
    hw_interface_->right_front_steer_motor->SetSmartCurrentLimit(current_limit);
    hw_interface_->right_back_steer_motor->SetSmartCurrentLimit(current_limit);

    return true;
}

bool SwerveDriveIO::ResetDriveEncoders() {
    OKC_CHECK(hw_interface_ != nullptr);

    OKC_CHECK(hw_interface_->left_front_drive_encoder != nullptr);
    OKC_CHECK(hw_interface_->left_back_drive_encoder != nullptr);
    OKC_CHECK(hw_interface_->right_front_drive_encoder != nullptr);
    OKC_CHECK(hw_interface_->right_back_drive_encoder != nullptr);

    OKC_CHECK(hw_interface_->left_front_drive_motor != nullptr);
    OKC_CHECK(hw_interface_->left_back_drive_motor != nullptr);
    OKC_CHECK(hw_interface_->right_front_drive_motor != nullptr);
    OKC_CHECK(hw_interface_->right_back_drive_motor != nullptr);

    // internal encoders
    hw_interface_->left_front_drive_encoder->SetPosition(0);
    hw_interface_->left_back_drive_encoder->SetPosition(0);
    hw_interface_->right_front_drive_encoder->SetPosition(0);
    hw_interface_->right_back_drive_encoder->SetPosition(0);

    return true;
}

bool SwerveDriveIO::ResetSteerEncoders() {
    OKC_CHECK(hw_interface_ != nullptr);

    OKC_CHECK(hw_interface_->left_front_steer_vel_encoder != nullptr);
    OKC_CHECK(hw_interface_->left_back_steer_vel_encoder != nullptr);
    OKC_CHECK(hw_interface_->right_back_steer_vel_encoder != nullptr);
    OKC_CHECK(hw_interface_->right_front_steer_vel_encoder != nullptr);

    hw_interface_->left_front_steer_vel_encoder->SetPosition(0);
    hw_interface_->left_back_steer_vel_encoder->SetPosition(0);
    hw_interface_->right_back_steer_vel_encoder->SetPosition(0);
    hw_interface_->right_front_steer_vel_encoder->SetPosition(0);

    return true;
}

bool SwerveDriveIO::ProcessInputs() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);

    // Get the hardware sensor values.
    // navX IMU:
    OKC_CHECK(hw_interface_->ahrs != nullptr);
    #ifdef __FRC_ROBORIO__
        sw_interface_->imu_yaw = hw_interface_->ahrs->GetAngle();
        sw_interface_->imu_pitch = hw_interface_->ahrs->GetRoll();
    #endif
    #ifndef __FRC_ROBORIO__
        sw_interface_->imu_yaw = 0.0;
        sw_interface_->imu_pitch = 0.0;
    #endif

    // Encoders
    // position
    // drive
    sw_interface_->left_front_drive_motor_enc =
        hw_interface_->left_front_drive_encoder->GetPosition();
    sw_interface_->left_back_drive_motor_enc =
        hw_interface_->left_back_drive_encoder->GetPosition();
    sw_interface_->right_front_drive_motor_enc =
        hw_interface_->right_front_drive_encoder->GetPosition();
    sw_interface_->right_back_drive_motor_enc =
        hw_interface_->right_back_drive_encoder->GetPosition();

    // steer
    sw_interface_->left_front_steer_motor_enc =
        hw_interface_->left_front_steer_encoder->GetAbsolutePosition();
    sw_interface_->left_back_steer_motor_enc =
        hw_interface_->left_back_steer_encoder->GetAbsolutePosition();
    sw_interface_->right_front_steer_motor_enc =
        hw_interface_->right_front_steer_encoder->GetAbsolutePosition();
    sw_interface_->right_back_steer_motor_enc =
        hw_interface_->right_back_steer_encoder->GetAbsolutePosition();

    // sw_interface_->left_front_steer_motor_enc =
    // hw_interface_->left_front_steer_vel_encoder->GetPosition();
    // sw_interface_->left_back_steer_motor_enc =
    // hw_interface_->left_back_steer_vel_encoder->GetPosition();
    // sw_interface_->right_front_steer_motor_enc =
    // hw_interface_->right_front_steer_vel_encoder->GetPosition();
    // sw_interface_->right_back_steer_motor_enc =
    // hw_interface_->right_back_steer_vel_encoder->GetPosition();

    // velocity
    // drive
    sw_interface_->left_front_drive_enc_vel =
        hw_interface_->left_front_drive_encoder->GetVelocity();
    sw_interface_->left_back_drive_enc_vel =
        hw_interface_->left_back_drive_encoder->GetVelocity();
    sw_interface_->right_front_drive_enc_vel =
        hw_interface_->right_front_drive_encoder->GetVelocity();
    sw_interface_->right_back_drive_enc_vel =
        hw_interface_->right_back_drive_encoder->GetVelocity();

    // steer
    sw_interface_->left_front_steer_enc_vel =
        hw_interface_->left_front_steer_vel_encoder->GetVelocity();
    sw_interface_->left_back_steer_enc_vel =
        hw_interface_->left_back_steer_vel_encoder->GetVelocity();
    sw_interface_->right_front_steer_enc_vel =
        hw_interface_->right_front_steer_vel_encoder->GetVelocity();
    sw_interface_->right_back_steer_enc_vel =
        hw_interface_->right_back_steer_vel_encoder->GetVelocity();

    return true;
}

bool SwerveDriveIO::SetOutputs() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);

    OKC_CHECK(hw_interface_->left_front_drive_motor != nullptr);
    OKC_CHECK(hw_interface_->left_back_drive_motor != nullptr);
    OKC_CHECK(hw_interface_->right_front_drive_motor != nullptr);
    OKC_CHECK(hw_interface_->right_back_drive_motor != nullptr);

    OKC_CHECK(hw_interface_->left_front_steer_motor != nullptr);
    OKC_CHECK(hw_interface_->left_back_steer_motor != nullptr);
    OKC_CHECK(hw_interface_->right_front_steer_motor != nullptr);
    OKC_CHECK(hw_interface_->right_back_steer_motor != nullptr);

    // Set the drive outputs.
    // hw_interface_->left_front_drive_motor->Set(TeamOKC::Clamp(-this->sw_interface_->drive_config.max_output_drive,
    // this->sw_interface_->drive_config.max_output_drive,
    // &this->sw_interface_->left_front_drive_motor_output));
    // hw_interface_->left_back_drive_motor->Set(TeamOKC::Clamp(-this->sw_interface_->drive_config.max_output_drive,
    // this->sw_interface_->drive_config.max_output_drive,
    // &this->sw_interface_->left_back_drive_motor_output));
    // hw_interface_->right_front_drive_motor->Set(TeamOKC::Clamp(-this->sw_interface_->drive_config.max_output_drive,
    // this->sw_interface_->drive_config.max_output_drive,
    // &this->sw_interface_->right_front_drive_motor_output));
    // hw_interface_->right_back_drive_motor->Set(TeamOKC::Clamp(-this->sw_interface_->drive_config.max_output_drive,
    // this->sw_interface_->drive_config.max_output_drive,
    // &this->sw_interface_->right_back_drive_motor_output));
    hw_interface_->left_front_drive_motor->Set(
        this->sw_interface_->left_front_drive_motor_output);
    hw_interface_->left_back_drive_motor->Set(
        this->sw_interface_->left_back_drive_motor_output);
    hw_interface_->right_front_drive_motor->Set(
        this->sw_interface_->right_front_drive_motor_output);
    hw_interface_->right_back_drive_motor->Set(
        this->sw_interface_->right_back_drive_motor_output);

    // set the steer outputs.
    // hw_interface_->left_front_steer_motor->Set(TeamOKC::Clamp(-this->sw_interface_->drive_config.max_output_steer,
    // this->sw_interface_->drive_config.max_output_steer,
    // &this->sw_interface_->left_front_steer_motor_output));
    // hw_interface_->left_back_steer_motor->Set(TeamOKC::Clamp(-this->sw_interface_->drive_config.max_output_steer,
    // this->sw_interface_->drive_config.max_output_steer,
    // &this->sw_interface_->left_back_steer_motor_output));
    // hw_interface_->right_front_steer_motor->Set(TeamOKC::Clamp(-this->sw_interface_->drive_config.max_output_steer,
    // this->sw_interface_->drive_config.max_output_steer,
    // &this->sw_interface_->right_front_steer_motor_output));
    // hw_interface_->right_back_steer_motor->Set(TeamOKC::Clamp(-this->sw_interface_->drive_config.max_output_steer,
    // this->sw_interface_->drive_config.max_output_steer,
    // &this->sw_interface_->right_back_steer_motor_output));
    hw_interface_->left_front_steer_motor->Set(
        this->sw_interface_->left_front_steer_motor_output);
    hw_interface_->left_back_steer_motor->Set(
        this->sw_interface_->left_back_steer_motor_output);
    hw_interface_->right_front_steer_motor->Set(
        this->sw_interface_->right_front_steer_motor_output);
    hw_interface_->right_back_steer_motor->Set(
        this->sw_interface_->right_back_steer_motor_output);

    return true;
}
