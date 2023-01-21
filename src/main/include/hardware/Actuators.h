#pragma once

#include <memory>
#include <rev/CANSparkMax.h>


// CTRE namespace
namespace ctre_can = ctre::phoenix::motorcontrol::can;

// Brushless Motor
#define BRUSHLESS rev::CANSparkMax::MotorType::kBrushless

// Coast
#define COAST rev::CANSparkMax::IdleMode::kCoast

// CAN IDs
#define LEFT_MOTOR_1 1
#define LEFT_MOTOR_2 2
#define LEFT_MOTOR_3 3
#define RIGHT_MOTOR_1 4
#define RIGHT_MOTOR_2 5
#define RIGHT_MOTOR_3 6

#define INDEXER_MOTOR 7
#define INTAKE_POSITION_MOTOR 10
#define INTAKE_MOTOR 11

#define SHOOTER_MOTOR 8
#define TRIGGER_MOTOR 9

typedef struct actuator_interface_t {
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

    // Claw things
    std::unique_ptr<rev::CANSparkMax> claw_motor;
    std::unique_ptr<FRC::DigitalInput> claw_IR_sensor;
} ActuatorInterface;