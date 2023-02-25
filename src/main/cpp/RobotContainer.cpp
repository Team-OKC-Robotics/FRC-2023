#include "RobotContainer.h"

RobotContainer::RobotContainer() {
    // Load robot parameters
    VOKC_CALL(RobotParams::LoadParameters(RobotParams::param_file));
    frc::DataLogManager::Start();

    // Initialize the hardware interface.
    hardware_ = std::make_unique<Hardware>();
    VOKC_CALL(this->InitHardware(hardware_));

    VOKC_CALL(this->InitSwerve());
    VOKC_CALL(this->InitArm());

    

    // Initialize the Gamepads
    VOKC_CALL(InitGamepads());

    // Initialize the commands
    VOKC_CALL(InitCommands());

    // Configure the button bindings
    VOKC_CALL(ConfigureButtonBindings());
}

bool RobotContainer::ConfigureButtonBindings() {
    VOKC_CHECK(driver_a_button_ != nullptr);
    VOKC_CHECK(driver_b_button_ != nullptr);
    VOKC_CHECK(driver_back_button_ != nullptr);
    VOKC_CHECK(driver_x_button_ != nullptr);

    //button bindings
    WPI_IGNORE_DEPRECATED
    // main driver controls
    driver_left_bumper_->WhileHeld(*manual_open_claw);
    driver_right_bumper_->WhileHeld(*manual_close_claw);
    
    // second driver controls
    manip_a_button_->WhileHeld(*lowerArmCommand);
    manip_y_button_->WhileHeld(*raiseArmCommand);
    manip_left_bumper_button_->WhileHeld(*retractArmCommand);
    manip_right_bumper_button_->WhileHeld(*extendArmCommand);

    // HACK XXX BUG TODO temporary first driver controls arm stuff for testing so only one person is needed to test the robot
    driver_a_button_->WhileHeld(*lowerArmCommand);
    driver_y_button_->WhileHeld(*raiseArmCommand);
    driver_left_bumper_->WhileHeld(*retractArmCommand);
    driver_right_bumper_->WhileHeld(*extendArmCommand);
    WPI_UNIGNORE_DEPRECATED
  
}

std::shared_ptr<frc2::Command> RobotContainer::GetAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autonomousCommand_;
}

std::shared_ptr<frc2::Command> RobotContainer::GetDriveCommand() {
    // VOKC_CHECK(swerve_teleop_command_ != nullptr);
    return swerve_teleop_command_;
}

bool RobotContainer::InitHardware(
    std::unique_ptr<Hardware> &hardware) {
    OKC_CHECK(hardware != nullptr);

    // Initialize sub-hardware interfaces.
    hardware->actuators = std::make_unique<Actuators>();
    hardware->sensors = std::make_unique<Sensors>();

    // Initialize the actuators.
    OKC_CALL(this->InitActuators(hardware->actuators.get()));

    // Set up sensors.
    OKC_CALL(this->InitSensors(*hardware->actuators, hardware->sensors.get()));

    return true;
}

bool RobotContainer::InitActuators(Actuators *actuators_interface) {
    OKC_CHECK(actuators_interface != nullptr);

    actuators_interface->left_front_drive_motor = std::make_unique<rev::CANSparkMax>(LEFT_FRONT_DRIVE_MOTOR, BRUSHLESS);
    actuators_interface->left_back_drive_motor = std::make_unique<rev::CANSparkMax>(LEFT_BACK_DRIVE_MOTOR, BRUSHLESS);
    actuators_interface->right_front_drive_motor = std::make_unique<rev::CANSparkMax>(RIGHT_FRONT_DRIVE_MOTOR, BRUSHLESS);
    actuators_interface->right_back_drive_motor = std::make_unique<rev::CANSparkMax>(RIGHT_BACK_DRIVE_MOTOR, BRUSHLESS);

    actuators_interface->left_front_steer_motor = std::make_unique<rev::CANSparkMax>(LEFT_FRONT_STEER_MOTOR, BRUSHLESS);
    actuators_interface->left_back_steer_motor = std::make_unique<rev::CANSparkMax>(LEFT_BACK_STEER_MOTOR, BRUSHLESS);
    actuators_interface->right_front_steer_motor = std::make_unique<rev::CANSparkMax>(RIGHT_FRONT_STEER_MOTOR, BRUSHLESS);
    actuators_interface->right_back_steer_motor = std::make_unique<rev::CANSparkMax>(RIGHT_BACK_STEER_MOTOR, BRUSHLESS);

    actuators_interface->arm_lift_motor = std::make_unique<rev::CANSparkMax>(ARM_LIFT_MOTOR, BRUSHLESS);
    actuators_interface->arm_up_motor = std::make_unique<rev::CANSparkMax>(ARM_UP_MOTOR, BRUSHLESS);
    actuators_interface->arm_extend_motor = std::make_unique<rev::CANSparkMax>(ARM_EXTEND_MOTOR, BRUSHLESS);
    return true;
}

bool RobotContainer::InitSensors(const Actuators &actuators,
                                 Sensors *sensor_interface) {
    OKC_CHECK(sensor_interface != nullptr);

    #ifdef __FRC_ROBORIO__
        // Initialize navX.
        try {
            sensor_interface->ahrs = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
        } catch (std::exception &ex) {
            std::string what_string = ex.what();
            std::string err_msg("Error instantiating navX MXP:  " + what_string);
            const char *p_err_msg = err_msg.c_str();

            // Print the error message.
            OKC_CHECK_MSG(false, p_err_msg);
        }
    #endif

    OKC_CHECK(actuators.left_front_drive_motor != nullptr);
    OKC_CHECK(actuators.left_back_drive_motor != nullptr);
    OKC_CHECK(actuators.right_front_drive_motor != nullptr);
    OKC_CHECK(actuators.right_back_drive_motor != nullptr);

    OKC_CHECK(actuators.left_front_steer_motor != nullptr);
    OKC_CHECK(actuators.left_back_steer_motor != nullptr);
    OKC_CHECK(actuators.right_front_steer_motor != nullptr);
    OKC_CHECK(actuators.right_back_steer_motor != nullptr);

    sensor_interface->left_front_drive_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.left_front_drive_motor->GetEncoder());
    sensor_interface->left_back_drive_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.left_back_drive_motor->GetEncoder());
    sensor_interface->right_front_drive_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.right_front_drive_motor->GetEncoder());
    sensor_interface->right_back_drive_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.right_back_drive_motor->GetEncoder());
    
    sensor_interface->left_front_steer_encoder = std::make_unique<frc::AnalogEncoder>(LEFT_FRONT_STEER_ENCODER);
    sensor_interface->left_back_steer_encoder = std::make_unique<frc::AnalogEncoder>(LEFT_BACK_STEER_ENCODER);
    sensor_interface->right_front_steer_encoder = std::make_unique<frc::AnalogEncoder>(RIGHT_FRONT_STEER_ENCODER);
    sensor_interface->right_back_steer_encoder = std::make_unique<frc::AnalogEncoder>(RIGHT_BACK_STEER_ENCODER);

    sensor_interface->left_front_steer_vel_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.left_front_steer_motor->GetEncoder());
    sensor_interface->left_back_steer_vel_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.left_back_steer_motor->GetEncoder());
    sensor_interface->right_front_steer_vel_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.right_front_steer_motor->GetEncoder());
    sensor_interface->right_back_steer_vel_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.right_back_steer_motor->GetEncoder());

    OKC_CHECK(sensor_interface->left_front_drive_encoder != nullptr);
    OKC_CHECK(sensor_interface->left_back_drive_encoder != nullptr);
    OKC_CHECK(sensor_interface->right_front_drive_encoder != nullptr);
    OKC_CHECK(sensor_interface->right_back_drive_encoder != nullptr);

    OKC_CHECK(sensor_interface->left_front_steer_encoder != nullptr);
    OKC_CHECK(sensor_interface->left_back_steer_encoder != nullptr);
    OKC_CHECK(sensor_interface->right_front_steer_encoder != nullptr);
    OKC_CHECK(sensor_interface->right_back_steer_encoder != nullptr);

    OKC_CHECK(actuators.arm_lift_motor != nullptr);
    OKC_CHECK(actuators.arm_extend_motor != nullptr);

    sensor_interface->arm_lift_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.arm_lift_motor->GetEncoder());
    sensor_interface->arm_duty_cycle_encoder = std::make_unique<frc::DutyCycleEncoder>(1);
    sensor_interface->arm_extend_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.arm_extend_motor->GetEncoder());
    sensor_interface->extend_limit_switch = std::make_unique<frc::DigitalInput>(0);

    OKC_CHECK(sensor_interface->arm_lift_encoder != nullptr);

    OKC_CHECK(actuators.claw_motor != nullptr);

    sensor_interface->claw_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.claw_motor->GetEncoder());

    return true;
}

bool RobotContainer::InitSwerve() {
    // == swerve drive ==
    OKC_CALL(SetupSwerveDriveInterface(hardware_, swerve_drive_hw_));

    // Initialize the software interface
    swerve_drive_sw_ = std::make_shared<SwerveDriveSoftwareInterface>();

    // Link SwerveDriveIO to hardware / software
    swerve_drive_io_ = std::make_shared<SwerveDriveIO>(swerve_drive_hw_.get(), swerve_drive_sw_.get());

    // Link swerve dirve software to the I/O
    swerve_drive_ = std::make_shared<SwerveDrive>(swerve_drive_sw_.get());
    
    OKC_CALL(swerve_drive_->Init());
    
    return true;
}

bool RobotContainer::InitArm() {
    OKC_CALL(SetupArmInterface(hardware_, arm_hw_));

    arm_sw_ = std::make_shared<ArmSoftwareInterface>();

    arm_io_ = std::make_shared<ArmIO>(arm_hw_.get(), arm_sw_.get());
    
    arm_ = std::make_shared<Arm>(arm_sw_.get());

    OKC_CALL(arm_->Init());

    return true;
}

bool RobotContainer::InitClaw() {
    OKC_CALL(SetupClawInterface(hardware_, claw_hw_));

    claw_sw_ = std::make_shared<ClawSoftwareInterface>();

    claw_io_ = std::make_shared<ClawIO>(claw_hw_.get(), claw_sw_.get());
    
    claw_ = std::make_shared<Claw>(claw_sw_.get());

    OKC_CALL(claw_->Init());

    return true;
}

bool RobotContainer::InitGamepads() {
    // Get joystick IDs from parameters.toml
    int gamepad1_id = RobotParams::GetParam("gamepad1_id", 0);
    int gamepad2_id = RobotParams::GetParam("gamepad2_id", 1);

    gamepad1_ = std::make_shared<frc::Joystick>(gamepad1_id);
    gamepad2_ = std::make_shared<frc::Joystick>(gamepad2_id);


    // Initialize the joystick buttons
    driver_a_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), A_BUTTON);
    driver_a_button_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), A_BUTTON);
    driver_b_button_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), B_BUTTON);
    driver_x_button_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), X_BUTTON);
    driver_y_button_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), Y_BUTTON);
    driver_start_bumper_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), START_BUTTON);
    driver_back_button_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), BACK_BUTTON);
    driver_left_bumper_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), LEFT_BUMP);
    driver_right_bumper_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), RIGHT_BUMP);

    // second driver
    manip_a_button_ = std::make_shared<frc2::JoystickButton>(gamepad2_.get(), A_BUTTON);
    manip_b_button_ = std::make_shared<frc2::JoystickButton>(gamepad2_.get(), B_BUTTON);
    manip_x_button_ = std::make_shared<frc2::JoystickButton>(gamepad2_.get(), X_BUTTON);
    manip_y_button_ = std::make_shared<frc2::JoystickButton>(gamepad2_.get(), Y_BUTTON);
    manip_back_button_ = std::make_shared<frc2::JoystickButton>(gamepad2_.get(), START_BUTTON);
    manip_start_button_ = std::make_shared<frc2::JoystickButton>(gamepad2_.get(), BACK_BUTTON);
    manip_left_bumper_button_ = std::make_shared<frc2::JoystickButton>(gamepad2_.get(), LEFT_BUMP);
    manip_right_bumper_button_ = std::make_shared<frc2::JoystickButton>(gamepad2_.get(), RIGHT_BUMP);
    manip_left_stick_button_ = std::make_shared<frc2::JoystickButton>(gamepad2_.get(), LEFT_STICK_BUTTON);
    manip_right_stick_button_ = std::make_shared<frc2::JoystickButton>(gamepad2_.get(), RIGHT_STICK_BUTTON);
    

    return true;
}

bool RobotContainer::InitCommands() {
    OKC_CHECK(swerve_drive_ != nullptr);

    // Placeholder autonomous command.
    m_autonomousCommand_ = std::make_shared<AutoSwerveCommand>(swerve_drive_, TeamOKC::Pose(10.0, 10.0, 90.0), true);

    // swerve commands
    swerve_teleop_command_ = std::make_shared<TeleOpSwerveCommand>(swerve_drive_, gamepad1_);

    // arm commands
    extendArmCommand = std::make_shared<IncrementArmExtendCommand>(arm_, 3); 
    retractArmCommand = std::make_shared<IncrementArmExtendCommand>(arm_, -3);

    raiseArmCommand = std::make_shared<IncrementArmPresetPositionCommand>(arm_, 3);
    lowerArmCommand = std::make_shared<IncrementArmPresetPositionCommand>(arm_, -3);

    // claw commands
    manual_open_claw = std::make_shared<ManualClawCommand>(claw_, -0.1);
    manual_close_claw = std::make_shared<ManualClawCommand>(claw_, 0.1);
     
    return true;
}
