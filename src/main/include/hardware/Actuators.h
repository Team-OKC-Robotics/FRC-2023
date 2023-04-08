#pragma once

#include <memory>
#include <rev/CANSparkMax.h>
#include "frc/DigitalInput.h"




// Brushless Motor
#define BRUSHLESS rev::CANSparkMax::MotorType::kBrushless

// Coast
#define COAST rev::CANSparkMax::IdleMode::kCoast
#define BRAKE rev::CANSparkMax::IdleMode::kBrake

#define LEFT_FRONT_DRIVE_MOTOR 1
#define LEFT_BACK_DRIVE_MOTOR 3
#define RIGHT_FRONT_DRIVE_MOTOR 5
#define RIGHT_BACK_DRIVE_MOTOR 7
#define LEFT_FRONT_STEER_MOTOR 2
#define LEFT_BACK_STEER_MOTOR 4
#define RIGHT_FRONT_STEER_MOTOR 6
#define RIGHT_BACK_STEER_MOTOR 8

#define ARM_LIFT_MOTOR 9
#define ARM_UP_MOTOR 10
#define ARM_EXTEND_MOTOR 11

#define CLAW_MOTOR 12

#define INTAKE_MOTOR 12


typedef struct actuators_t {
    // Left drivetrain motors
    std::unique_ptr<rev::CANSparkMax> left_front_drive_motor;
    std::unique_ptr<rev::CANSparkMax> left_back_drive_motor;
    std::unique_ptr<rev::CANSparkMax> right_front_drive_motor;
    std::unique_ptr<rev::CANSparkMax> right_back_drive_motor;

    // Right drivetrain motors
    std::unique_ptr<rev::CANSparkMax> left_front_steer_motor;
    std::unique_ptr<rev::CANSparkMax> left_back_steer_motor;
    std::unique_ptr<rev::CANSparkMax> right_front_steer_motor;
    std::unique_ptr<rev::CANSparkMax> right_back_steer_motor;

    // arm motors
    std::unique_ptr<rev::CANSparkMax> arm_lift_motor;
    std::unique_ptr<rev::CANSparkMax> arm_up_motor;
    std::unique_ptr<rev::CANSparkMax> arm_extend_motor;

    // Claw things
    std::unique_ptr<rev::CANSparkMax> claw_motor;

    //intake motors
    std::unique_ptr<rev::CANSparkMax> intake_motor;
} Actuators;