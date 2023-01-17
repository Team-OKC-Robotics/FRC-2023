
#include "hardware/HardwareInterface.h"

bool SetupSwerveDriveInterface(
    std::unique_ptr<HardwareInterface> &hardware,
    std::shared_ptr<SwerveDriveHardwareInterface> &interface) {
    OKC_CHECK(hardware->actuators != nullptr);
    OKC_CHECK(hardware->sensors != nullptr);

    // Get actuators interface for swerve drive.
    std::unique_ptr<ActuatorInterface> &actuators = hardware->actuators;
    std::unique_ptr<SensorInterface> &sensors = hardware->sensors;

    // make sure all the actuators actually exist
    OKC_CHECK(actuators->left_front_drive_motor != nullptr);
    OKC_CHECK(actuators->left_back_drive_motor != nullptr);
    OKC_CHECK(actuators->right_front_drive_motor != nullptr);
    OKC_CHECK(actuators->right_back_drive_motor != nullptr);
    OKC_CHECK(actuators->left_front_steer_motor != nullptr);
    OKC_CHECK(actuators->left_back_steer_motor != nullptr);
    OKC_CHECK(actuators->right_front_steer_motor != nullptr);
    OKC_CHECK(actuators->right_back_steer_motor != nullptr);

    OKC_CHECK(sensors->ahrs != nullptr);
    OKC_CHECK(sensors->left_front_steer_encoder != nullptr);
    OKC_CHECK(sensors->left_back_steer_encoder != nullptr);
    OKC_CHECK(sensors->right_front_steer_encoder != nullptr);
    OKC_CHECK(sensors->right_back_steer_encoder != nullptr);


    // set up swerve drive interface.
    SwerveDriveHardwareInterface swerve_drive_interface = {
        actuators->left_front_drive_motor.get(),
        actuators->left_back_drive_motor.get(),
        actuators->right_front_drive_motor.get(),
        actuators->right_back_drive_motor.get(),
        actuators->left_front_steer_motor.get(),
        actuators->left_back_steer_motor.get(),
        actuators->right_front_steer_motor.get(),
        actuators->right_back_steer_motor.get(),

        sensors->ahrs.get(),

        sensors->left_front_drive_encoder.get(),
        sensors->left_back_drive_encoder.get(),
        sensors->right_front_drive_encoder.get(),
        sensors->right_back_drive_encoder.get(),

        sensors->left_front_steer_encoder.get(),
        sensors->left_back_steer_encoder.get(),
        sensors->right_front_steer_encoder.get(),
        sensors->right_back_steer_encoder.get(),

        sensors->left_front_steer_vel_encoder.get(),
        sensors->left_back_steer_vel_encoder.get(),
        sensors->right_front_steer_vel_encoder.get(),
        sensors->right_back_steer_vel_encoder.get(),
    };

    // set the output interface
    interface = std::make_shared<SwerveDriveHardwareInterface>(swerve_drive_interface);

    return true;
}