#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>

#include "hardware/ActuatorInterface.h"
#include "hardware/SensorInterface.h"

// Subsystem I/O
#include "io/DrivetrainIO.h"
#include "io/IntakeIO.h"
#include "io/ShooterIO.h"

typedef struct hardware_t {
    // Actuators
    std::unique_ptr<ActuatorInterface> actuators;

    // Drivetrain specific hardware abstractions.
    std::unique_ptr<frc::DifferentialDrive> diff_drive;

    // Sensors
    std::unique_ptr<SensorInterface> sensors;

} HardwareInterface;

// Subsystem hardware setup functions

/**
 * @brief Link the Drivetrain to the hardware interfaces.
 *
 * @param interface
 * @return true
 * @return false
 */
bool SetupSwervedriveInterface(
    std::unique_ptr<HardwareInterface> &hardware,
    std::shared_ptr<DrivetrainHardwareInterface> *interface);

/**
 * @brief Link the Intake to the hardware interfaces.
 *
 * @param interface
 * @return true
 * @return false
 */
bool SetupArmInterface(std::unique_ptr<HardwareInterface> &hardware,
                          std::shared_ptr<IntakeHardwareInterface> *interface);

/**
 * @brief Link the Shooter to the hardware interfaces.
 *
 * @param interface
 * @return true
 * @return false
 */
bool SetupClawInterface(
    std::unique_ptr<HardwareInterface> &hardware,
    std::shared_ptr<ShooterHardwareInterface> *interface);

 bool SetupVisionTrackingInterface(
    std::unique_ptr<HardwareInterface> &hardware,
    std::shared_ptr<ShooterHardwareInterface> *interface);


