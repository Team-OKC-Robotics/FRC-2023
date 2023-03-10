#include "RobotContainer.h"

RobotContainer::RobotContainer() {
    // Load robot parameters
    VOKC_CALL(RobotParams::LoadParameters(RobotParams::param_file));
    
    // reduce the size of the logs
    frc::DataLogManager::LogNetworkTables(false);

    // Initialize the hardware interface.
    hardware_ = std::make_unique<Hardware>();
    VOKC_CALL(this->InitHardware(hardware_));

    // initialize the subsystems
    VOKC_CALL(this->InitSwerve());
    VOKC_CALL(this->InitArm());
    VOKC_CALL(this->InitIntake());

    // Initialize the Gamepads
    VOKC_CALL(InitGamepads());

    // Initialize the commands
    VOKC_CALL(InitCommands());

    // Configure the button bindings
    VOKC_CALL(ConfigureButtonBindings());
}

bool RobotContainer::ConfigureButtonBindings() {
    OKC_CHECK(driver_a_button_ != nullptr);
    OKC_CHECK(driver_b_button_ != nullptr);
    OKC_CHECK(driver_back_button_ != nullptr);
    OKC_CHECK(driver_x_button_ != nullptr);

    //button bindings
    WPI_IGNORE_DEPRECATED
    // main driver controls
    // swerve
    driver_b_button_->WhenPressed(*fast_swerve_teleop_command_).WhenReleased(*swerve_teleop_command_);
    driver_a_button_->WhenPressed(*slow_swerve_teleop_command_).WhenReleased(*swerve_teleop_command_);

    // intake commands
    driver_left_bumper_->WhenPressed(*intake_command).WhenReleased(*stop_intake_command);
    driver_right_bumper_->WhenPressed(*other_intake_command).WhenReleased(*stop_intake_command);
    
    // HACK XXX BUG TODO temporary first driver controls arm stuff for testing so only one person is needed to test the robot
    driver_a_button_->WhileActiveContinous(*lowerArmCommand);
    driver_y_button_->WhileActiveContinous(*raiseArmCommand);
    
    driver_x_button_->WhileActiveContinous(*retractArmCommand);
    driver_b_button_->WhileActiveContinous(*extendArmCommand);
    
    // second driver controls
    manip_x_button_->WhenPressed(*arm_carry_command_);
    manip_a_button_->WhenPressed(*arm_pickup_command_);
    manip_b_button_->WhenPressed(*arm_score_mid_command_);
    manip_y_button_->WhenPressed(*arm_score_high_command_);
    manip_left_bumper_button_->WhenPressed(*arm_short_carry_command_);

    manip_start_button_->WhenPressed(*arm_dpad_set_state_command_);
  
    WPI_UNIGNORE_DEPRECATED
  
    return true;
}

std::shared_ptr<frc2::Command> RobotContainer::GetAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autonomousCommand_;
}

std::shared_ptr<frc2::Command> RobotContainer::GetDriveCommand() {
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
    actuators_interface->arm_extend_motor->SetInverted(true);
    actuators_interface->arm_extend_motor->SetIdleMode(BRAKE);

    actuators_interface->intake_motor = std::make_unique<rev::CANSparkMax>(INTAKE_MOTOR, BRUSHLESS);

    OKC_CHECK(actuators_interface->intake_motor != nullptr);

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
    sensor_interface->arm_duty_cycle_encoder = std::make_unique<frc::DutyCycleEncoder>(ARM_ABS_ENCODER);
    sensor_interface->arm_extend_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.arm_extend_motor->GetEncoder());
    sensor_interface->extend_limit_switch = std::make_unique<frc::DigitalInput>(EXTEND_LIMIT_SWITCH);

    OKC_CHECK(sensor_interface->arm_lift_encoder != nullptr);

    sensor_interface->intake_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.intake_motor->GetEncoder());

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

    OKC_CALL(arm_io_->Init());
    OKC_CALL(arm_->Init());

    return true;
}

bool RobotContainer::InitIntake() {
    OKC_CALL(SetupIntakeInterface(hardware_, intake_hw_));

    OKC_CHECK(hardware_ != nullptr);
    OKC_CHECK(intake_hw_ != nullptr);

    intake_sw_ = std::make_shared<IntakeSoftwareInterface>();

    OKC_CHECK(intake_sw_ != nullptr);

    intake_io_ = std::make_shared<IntakeIO>(intake_hw_.get(), intake_sw_.get());

    OKC_CHECK(intake_io_ != nullptr);

    intake_ = std::make_shared<Intake>(intake_sw_.get());

    OKC_CHECK(intake_ != nullptr);

    OKC_CALL(intake_->Init());

    return true;
}


bool RobotContainer::InitGamepads() {
    // Get joystick IDs from parameters.toml
    int gamepad1_id = RobotParams::GetParam("gamepad1_id", 0);
    int gamepad2_id = RobotParams::GetParam("gamepad2_id", 1);

    gamepad1_ = std::make_shared<frc::Joystick>(gamepad1_id);
    gamepad2_ = std::make_shared<frc::Joystick>(gamepad2_id);


    // Initialize the joystick buttons
    driver_a_button_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), A_BUTTON);
    driver_b_button_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), B_BUTTON);
    driver_x_button_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), X_BUTTON);
    driver_y_button_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), Y_BUTTON);
    driver_start_button_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), START_BUTTON);
    driver_back_button_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), BACK_BUTTON);
    driver_left_bumper_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), LEFT_BUMP);
    driver_right_bumper_ = std::make_shared<frc2::JoystickButton>(gamepad1_.get(), RIGHT_BUMP);
    // driver_left_trigger_ = std::make_shared<TriggerButton(gamepad1_.get(), LEFT_TRIGGER);
    // driver_right_trigger_ = std::make_shared<TriggerButton(gamepad1_.get(), RIGHT_TRIGGER);

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

    double pickup_rotation_ = RobotParams::GetParam("arm.pickup.arm_setpoint", 0.0);
    double pickup_extension_ = RobotParams::GetParam("arm.pickup.extend_setpoint", 0.0);

    double score_mid_rotation_ = RobotParams::GetParam("arm.score_medium.arm_setpoint", 0.0);
    double score_mid_extension_ = RobotParams::GetParam("arm.score_medium.extend_setpoint", 0.0);

    double score_high_rotation_ = RobotParams::GetParam("arm.score_high.arm_setpoint", 0.0);
    double score_high_extension_ = RobotParams::GetParam("arm.score_high.extend_setpoint", 0.0);

    // Placeholder autonomous command.
    m_autonomousCommand_ = std::make_shared<ScorePreloadedAuto>(swerve_drive_, arm_, intake_);
    // m_autonomousCommand_ = nullptr;

    // swerve commands
    swerve_teleop_command_ = std::make_shared<TeleOpSwerveCommand>(swerve_drive_, gamepad1_, 0.75, 0.75, false); // speed mod, open loop
    slow_swerve_teleop_command_ = std::make_shared<TeleOpSwerveCommand>(swerve_drive_, gamepad1_, 0.5, 1, true); // brake mode
    fast_swerve_teleop_command_ = std::make_shared<TeleOpSwerveCommand>(swerve_drive_, gamepad1_, 1.5, 0.1, false); // BOOOOOOOOOST
    OKC_CHECK(swerve_teleop_command_ != nullptr);

    // test arm commands
    extendArmCommand = std::make_shared<IncrementArmExtendCommand>(arm_, 0.5); 
    retractArmCommand = std::make_shared<IncrementArmExtendCommand>(arm_, -0.5);

    raiseArmCommand = std::make_shared<IncrementArmPresetPositionCommand>(arm_, 0.5);
    lowerArmCommand = std::make_shared<IncrementArmPresetPositionCommand>(arm_, -0.5);

    // arm commands
    arm_pickup_command_ = std::make_shared<ArmSetStateCommand>(arm_, TeamOKC::ArmState(pickup_extension_, pickup_rotation_));
    arm_score_mid_command_ = std::make_shared<ArmSetStateCommand>(arm_, TeamOKC::ArmState(score_mid_extension_, score_mid_rotation_));
    arm_score_high_command_ = std::make_shared<ArmSetStateCommand>(arm_, TeamOKC::ArmState(score_high_extension_, score_high_rotation_));
    arm_carry_command_ = std::make_shared<ArmSetStateCommand>(arm_, TeamOKC::ArmState(1, 0)); // hold the arm inside the robot when driving
    arm_short_carry_command_ = std::make_shared<ArmSetStateCommand>(arm_, TeamOKC::ArmState(2, pickup_rotation_)); // just bring teh arm a little in whenever we're moving in the community

    arm_dpad_set_state_command_ = std::make_shared<ArmSetStateDpadCommand>(arm_, gamepad2_);
    
    // intake commands
    intake_command = std::make_shared<IntakeCommand>(intake_, 0.3);
    other_intake_command = std::make_shared<IntakeCommand>(intake_, -0.3);
    stop_intake_command = std::make_shared<IntakeCommand>(intake_, -0.01);
   
    return true;
}

std::shared_ptr<Arm> RobotContainer::GetArm() {
    return arm_;
}