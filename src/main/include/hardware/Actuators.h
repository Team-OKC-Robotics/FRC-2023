#pragma once

#include <memory>
#include <rev/CANSparkMax.h>


// CTRE namespace
namespace ctre::phoenix::motorcontrol::can;

// Brushless Motor
#define BRUSHLESS rev::CANSparkMax::MotorType::kBrushless

// Coast
#define COAST rev::CANSparkMax::IdleMode::kCoast



typedef struct actuators_t {
    // Left drivetrain motors
    std::unique_ptr<rev::CANSparkMax> left_front_drive;
    std::unique_ptr<rev::CANSparkMax> left_back_drive;
    std::unique_ptr<rev::CANSparkMax> left_front_steer;
    std::unique_ptr<rev::CANSparkMax> left_back_steer;

    // Right drivetrain motors
    std::unique_ptr<rev::CANSparkMax> right_front_drive;
    std::unique_ptr<rev::CANSparkMax> right_back_drive;
    std::unique_ptr<rev::CANSparkMax> right_front_steer;
    std::unique_ptr<rev::CANSparkMax> right_back_steer;

    // arm motors
    std::unique_ptr<rev::CANSparkMax> arm_lift_motor;
    std::unique_ptr<rev::CANSparkMax> arm_up_motor;
    std::unique_ptr<rev::CANSparkMax> arm_extend_motor;
} Actuators;