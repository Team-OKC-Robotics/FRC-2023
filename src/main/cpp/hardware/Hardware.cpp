#include "hardware/Hardware.h"
#include "io/SwerveDriveIO.h"

bool SetupSwerveDriveInterface(
    std::unique_ptr<Hardware> &hardware,
    std::shared_ptr<SwerveDriveHardwareInterface> &interface) {
    OKC_CHECK(hardware->actuators != nullptr);
    OKC_CHECK(hardware->sensors != nullptr);

    // Get actuators interface for swerve drive.
    std::unique_ptr<Actuators> &actuators = hardware->actuators;
    std::unique_ptr<Sensors> &sensors = hardware->sensors;

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

bool SetupArmInterface(std::unique_ptr<Hardware> &hardware,
                            std::shared_ptr<ArmHardwareInterface> &interface){ 
 OKC_CHECK(hardware->actuators != nullptr);
                            

    //Get actuators interface for arm
    std::unique_ptr<Actuators> &actuators = hardware->actuators;
    std::unique_ptr<Sensors> &sensors = hardware->sensors;
    
   //make sure the actuators actually exist 
   OKC_CHECK(actuators->arm_lift_motor != nullptr);
   OKC_CHECK(actuators->arm_up_motor != nullptr);
   OKC_CHECK(actuators->arm_extend_motor != nullptr);

   ArmHardwareInterface arm_interface = {actuators->arm_lift_motor.get(),
                                         actuators->arm_up_motor.get(),
                                         actuators->arm_extend_motor.get(),
                                         sensors->arm_lift_encoder.get(),
                                         sensors->arm_duty_cycle_encoder.get(),
                                         sensors->arm_extend_encoder.get(),
                                         sensors->extend_limit_switch.get()
                                         };
    interface = std::make_shared<ArmHardwareInterface>(arm_interface);

    return true;
}

bool SetupClawInterface(std::unique_ptr<Hardware> &hardware, std::shared_ptr<ClawHardwareInterface> &interface) {
    //Get actuators interface for arm
    std::unique_ptr<Actuators> &actuators = hardware->actuators;
    std::unique_ptr<Sensors> &sensors = hardware->sensors;
    
   //make sure the actuators actually exist 
   OKC_CHECK(actuators->claw_motor != nullptr);
   OKC_CHECK(sensors->claw_encoder != nullptr);
   
   ClawHardwareInterface claw_interface = {
    actuators->claw_motor.get(),
    sensors->claw_encoder.get()
   };

    interface = std::make_shared<ClawHardwareInterface>(claw_interface);

    return true;
}

bool SetupIntakeInterface(std::unique_ptr<Hardware> &hardware, std::shared_ptr<IntakeHardwareInterface> &interface) {
    std::unique_ptr<Actuators> &actuators = hardware->actuators;
    std::unique_ptr<Sensors> &sensors = hardware->sensors;

    OKC_CHECK(actuators->intake_motor != nullptr);
    OKC_CHECK(sensors->intake_encoder != nullptr);

    IntakeHardwareInterface intake_interface = {
        actuators->intake_motor.get(),
        sensors->intake_encoder.get()
    };

    interface = std::make_shared<IntakeHardwareInterface>(intake_interface);

    return true;
}

bool SetupLEDsInterface(std::unique_ptr<Hardware> &hardware, std::shared_ptr<LEDsHardwareInterface> &interface) {
    std::unique_ptr<Actuators> &actuators = hardware->actuators;
    std::unique_ptr<Sensors> &sensors = hardware->sensors;

    OKC_CHECK(actuators->leds_controller_ != nullptr);

    LEDsHardwareInterface leds_interface = {
        actuators->leds_controller_.get()
    };

    interface = std::make_shared<LEDsHardwareInterface>(leds_interface);

    return true;
}