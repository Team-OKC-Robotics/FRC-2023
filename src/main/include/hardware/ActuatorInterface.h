
#pragma once

#include <memory>
#include <rev/CANSparkMax.h>

// Brushless Motor
#define BRUSHLESS rev::CANSparkMax::MotorType::kBrushless

// Coast
#define COAST rev::CANSparkMax::IdleMode::kCoast

#define LEFT_FRONT_DRIVE_MOTOR 1
#define LEFT_BACK_DRIVE_MOTOR 3
#define RIGHT_FRONT_DRIVE_MOTOR 5
#define RIGHT_BACK_DRIVE_MOTOR 7
#define LEFT_FRONT_STEER_MOTOR 2
#define LEFT_BACK_STEER_MOTOR 4
#define RIGHT_FRONT_STEER_MOTOR 6
#define RIGHT_BACK_STEER_MOTOR 8


typedef struct actuator_interface_t {
    // swerve drive motors
    std::unique_ptr<rev::CANSparkMax> left_front_drive_motor;
    std::unique_ptr<rev::CANSparkMax> left_back_drive_motor;

    std::unique_ptr<rev::CANSparkMax> right_front_drive_motor;
    std::unique_ptr<rev::CANSparkMax> right_back_drive_motor;

    std::unique_ptr<rev::CANSparkMax> left_front_steer_motor;
    std::unique_ptr<rev::CANSparkMax> left_back_steer_motor;

    std::unique_ptr<rev::CANSparkMax> right_front_steer_motor;
    std::unique_ptr<rev::CANSparkMax> right_back_steer_motor;
} ActuatorInterface;