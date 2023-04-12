#pragma once

#include <frc/motorcontrol/MotorControllerGroup.h>

#include "hardware/Actuators.h"
#include "hardware/Sensors.h"
#include "Utils.h"
#include "io/SwerveDriveIO.h"
#include "io/ArmIO.h"
#include "io/ClawIO.h"
#include "io/IntakeIO.h"
#include "io/VisionIO.h"

// Subsystem I/O

typedef struct hardware_t {
    // Actuators
    std::unique_ptr<Actuators> actuators;

    // Sensors
    std::unique_ptr<Sensors> sensors;

} Hardware;

// Subsystem hardware setup functions

/**
 * @brief Link the Swerve drive to the hardware interfaces.
 *
 * @param interface
 * @return true
 * @return false
 */
bool SetupSwerveDriveInterface(
    std::unique_ptr<Hardware> &hardware,
    std::shared_ptr<SwerveDriveHardwareInterface> &interface);

/**
 * @brief Link the arm to the hardware interfaces.
 *
 * @param interface
 * @return true
 * @return false
 */
bool SetupArmInterface(std::unique_ptr<Hardware> &hardware,
                        std::shared_ptr<ArmHardwareInterface> &interface);

/**
 * @brief Link the claw to the hardware interfaces.
 *
 * @param interface
 * @return true
 * @return false
 */
bool SetupClawInterface(
   std::unique_ptr<Hardware> &hardware,
   std::shared_ptr<ClawHardwareInterface> &interface);

bool SetupIntakeInterface(
    std::unique_ptr<Hardware> &hardware,
    std::shared_ptr<IntakeHardwareInterface> &interface);


bool SetupVisionInterface(
   std::unique_ptr<HardwareInterface> &hardware,
   std::shared_ptr<VisionHardware> *interface);
