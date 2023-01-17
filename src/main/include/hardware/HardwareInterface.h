
#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>

#include "hardware/ActuatorInterface.h"
#include "hardware/SensorInterface.h"

// Subsystem I/O
#include "io/SwerveDriveIO.h"

typedef struct hardware_t {
    // Actuators
    std::unique_ptr<ActuatorInterface> actuators;

    // Sensors
    std::unique_ptr<SensorInterface> sensors;

} HardwareInterface;

// Subsystem hardware setup functions
/**
 * @brief Link the Swerve drive to the hardware interfaces.
 *
 * @param interface
 * @return true
 * @return false
 */
bool SetupSwerveDriveInterface(
    std::unique_ptr<HardwareInterface> &hardware,
    std::shared_ptr<SwerveDriveHardwareInterface> &interface);