package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public final static class DriveConstants {
        //Driving Parameters... not the max speeds, just what's allowed
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI;

        public static final double kDirectionSlewRate = 1.2; //radians per second
        public static final double kMagnitudeSlewRate = 1.8; //percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; //^^

        //Chassis config
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        //distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot 
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

            // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kFrontLeftTurningCanId = 8;
    
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kRearLeftTurningCanId = 6;
    
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kFrontRightTurningCanId = 4;
   
    public static final int kRearRightDrivingCanId = 1;
    public static final int kRearRightTurningCanId = 2;

}

public static final class ModuleConstants {
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

}

public static final class NeoMotorConstants {
public static final double kFreeSpeedRpm = 5676;
}




