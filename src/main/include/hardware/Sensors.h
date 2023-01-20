#pragma once

#include <memory>

#include "AHRS.h"
#include <frc/DigitalInput.h>
#include <rev/RelativeEncoder.h>

// == sensor ports ==
#define DEPLOY_LIMIT_SWITCH 2
#define RETRACTED_LIMIT_SWITCH 3

#define BALL_DETECTOR 9

typedef struct Sensors {
    // navX IMU
    std::unique_ptr<AHRS> ahrs;

    // intake limit switches
    std::unique_ptr<frc::DigitalInput> deploy_limit_switch;
    std::unique_ptr<frc::DigitalInput> retracted_limit_switch;

    // Shooter ball detector
    std::unique_ptr<frc::DigitalInput> ball_detector;
} Sensors;