#pragma once

#include <memory>

#include "AHRS.h"
#include "frc/AnalogEncoder.h"
#include <rev/RelativeEncoder.h>

// == sensor ports ==
#define LEFT_FRONT_STEER_ENCODER 0
#define LEFT_BACK_STEER_ENCODER 1
#define RIGHT_FRONT_STEER_ENCODER 2
#define RIGHT_BACK_STEER_ENCODER 3

typedef struct sensors_t {
    // navX IMU
    std::unique_ptr<AHRS> ahrs;

    // swerve drive drive encoders
    std::unique_ptr<rev::SparkMaxRelativeEncoder> left_front_drive_encoder;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> left_back_drive_encoder;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> right_front_drive_encoder;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> right_back_drive_encoder;

    // swerve drive steer encoders
    std::unique_ptr<frc::AnalogEncoder> left_front_steer_encoder;
    std::unique_ptr<frc::AnalogEncoder> left_back_steer_encoder;
    std::unique_ptr<frc::AnalogEncoder> right_front_steer_encoder;
    std::unique_ptr<frc::AnalogEncoder> right_back_steer_encoder;

    // other steer encoders
    std::unique_ptr<rev::SparkMaxRelativeEncoder> left_front_steer_vel_encoder;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> left_back_steer_vel_encoder;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> right_front_steer_vel_encoder;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> right_back_steer_vel_encoder;

    // arm encoders
    std::unique_ptr<rev::SparkMaxRelativeEncoder> arm_lift_encoder;
    std::unique_ptr<frc::AnalogEncoder> arm_absolute_encoder;

} Sensors;
