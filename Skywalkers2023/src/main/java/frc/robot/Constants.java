// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameter = Units.inchesToMeters(4); // done
        public static final double kTicksPerRotation = 2048 * 150 / 7; // done
        public static final double kDriveTicksPerRotation = 2048 * 6.75;

        public static final double kPTurning = 0.75; // CHANGE!!!
        public static final double kDTurning = 0.025; // necessary??
        public static final double kITurning = 0.0;

        public static final double kVDrive = 2.16;
        public static final double kSDrive = 0.634;

   }    

    public static final class OIConstants {
        public static final double kDeadband = 0.15;

        public static final int kDriverControllerPort = 0;
        public static final int kDriverControllerPort2 = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(24);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.00;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        
        public static final int kFrontLeftDriveMotorPort = 10;
        public static final int kBackLeftDriveMotorPort = 7;
        public static final int kFrontRightDriveMotorPort = 1;
        public static final int kBackRightDriveMotorPort = 4;

        public static final int kFrontLeftTurningMotorPort = 11;
        public static final int kBackLeftTurningMotorPort = 8;
        public static final int kFrontRightTurningMotorPort = 2;
        public static final int kBackRightTurningMotorPort = 5;

        // MIGHT NEED TO CHANGE
        public static final boolean kFrontLeftTurningEncoderReversed = false; //true
        public static final boolean kBackLeftTurningEncoderReversed = false; //true
        public static final boolean kFrontRightTurningEncoderReversed = false; //true
        public static final boolean kBackRightTurningEncoderReversed = false; //true

        // MIGHT NEED TO CHANGE
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 9;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kBackRightDriveAbsoluteEncoderPort = 6;

        // MIGHT NEED TO CHANGE
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        // public static final double kFLAbsEcoderOffsetDeg = 81.5;
        // public static final double kBLAbsEcoderOffsetDeg = 160.2;
        // public static final double kFRAbsEncoderOffsetDeg = 22.7;
        // public static final double kBRAbsEncoderOffsetDeg = 168.8;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.96;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 4.409;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 5.135;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 4.532;



        



        public static final int kIMUPort = 13;


        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4; // ????


        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3; // ????
        
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond*3/5; // ?????
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond/4; //????
    }

    public static final class IntakeConstants {
        public static final int kIntakePort = 22; // ??
        public static final double kMaxIntakeSpeed = 0.4; // ??
        public static final double kMaxOuttakeSpeed = -0.15; // ??
        public static final double kHoldSpeed = 0.05;
        public static final boolean kIntakeInverted = false; // ?
        public static final int kIntakeTicksPerRotation = 4096; // ??
        public static final double kExpectedFullVelocity = 50; // ?? in rotations per cycle
        public static final double kObjectHeldRatioThreshold = 3.0; // ?? modify as needed
        public static final double kCurrentThreshold = 15.00;

        public static double kSpeedUpFailTime = 0.75; //seconds it tries to speed up
        public static double kOutFailTime = 0.75; //seconds it tries to outtake
        public static double kObjectHeldThreshold = 300; //proportional to max intake speed, current values found at 0.3
        public static double kSpeedUpThreshold = 100;
        public static double kObjectOutThreshold = 166;
    }

    public static final class ElevatorConstants {
        public static final int kLeftElevatorPort = 31;
        public static final int kRightElevatorPort = 30; 

        public static final double kMaxElevatorSpeed = 0.4;
        public static final boolean kLeftElevatorInverted = false;
        public static final boolean kRightElevatorInverted = true;
        public static final int kelevatorTicksPerRotation = 2048;
        public static final double kElevatorGearRatio = 10.00 * 22.0 / 12.0;
        public static final double kSpoolDiameter = Units.inchesToMeters(1.751);
        public static final double kDistancePerRevolution = Math.PI * kSpoolDiameter;

        public static final double kPositionConversionFactor = kDistancePerRevolution/(kElevatorGearRatio * kelevatorTicksPerRotation) * 2.53715;
        public static final double kVelocityConversionFactor = kPositionConversionFactor * 10.0;

        public static final double kBottomLimit = 0.0;
        public static final double kTopLimit = 1.45;

        public static final double kMaxVel = 0.5;
        public static final double kMaxAcc = 0.5;

        public static final double kVUp = 6.17;
        public static final double kSUp = 0.999;

        public static final double kVDown = 6.41;
        public static final double kSDown = -0.13;
        
        

        public static final double kP = 20.00;
        public static final double kI = 0.00;
        public static final double kD = 0.00;

        public static final double kCurrentThreshold = 10.00;
        public static final double kHomingSpeed = -0.07;

        public static final double kMountAngleRadians = 0.9599;
    }


    public static final class ArmConstants {
        public static final int kArmPort = 23; // ???
        public static final int kArmAbsoluteEncoderPort = 16;
        public static final double kMaxArmSpeed = 0.5; // ??
        public static final boolean kArmInverted = true; // ??
        public static final boolean kArmAbsEncoderInverted = true;
        // public static final boolean kEncoderInverted = true;
        
        public static final int kArmTicksPerRotation = 2048; // ??
        public static final double kArmGearRatio = 180.00 * 2.0; // ???

        public static final double kAbsEncoderOffset = 3.27 * 360.0 / (2.0 * Math.PI);


        public static final double kPositionConversionFactor = 2 * Math.PI / (kArmTicksPerRotation * kArmGearRatio);
        public static final double kVelocityConversionFactor = kPositionConversionFactor * 10.0;

        public static final double kBottomLimit = -0.17; // -10 degrees
        public static final double kTopLimit = 2.44; // 150 degrees

        // feedback

        public static final double kPArm = 15.00;
        public static final double kIArm = 0.0;
        public static final double kDArm = 0.00;

        // feedforward

        // calculate necessary gains using recalc: https://www.reca.lc/linear (overkill?)

        // public static final double kSArm = 0.00; // to overcome static friction
        // public static final double kVArm = 2.69; //1.47 for Neo550
        // public static final double kGArm = 0.07; //0.32 for Neo550
        // public static final double kAArm = 0.00;

        public static final double kVUp = 6.14;
        public static final double kSUp = 0.671;

        public static final double kVDown = 6.4;
        public static final double kSDown = -0.439;

        public static final double kCurrentThreshold = 15.00;

        public static final double kHoningSpeed = -0.1;

    }

    public static final class SensorConstants {
        public static final int limitSwitchPort = 0;
        public static final int beamBreakerPort = 5;
    }

    public static final class LimelightConstants{
        public static double kPx = 5; //meters
        public static double kPy = 5; //meters
        public static double kPr = 0.05; //degrees
        public static double kDx = 0;
        public static double kDy = 0;
        public static double kDr = 0;
        public static double kIx = 0;
        public static double kIy = 0;
        public static double kIr = 0;
      
        public static double xclamp = 0.5; //slides sideways very slowly
        public static double yclamp = 0.5;
        public static double rclamp = 0.4;
      
        //public static double tagheight = 0.49; //19.3 inches to meters
      
        public static double xtolerance = 0.05; 
        public static double ytolerance = 0.03; 
        public static double rtolerance = 5; //degrees
      
        public static double mountingangle = 0; //for adjustable camera
        public static double cameraheight = Units.inchesToMeters(14); //14-ish inches, to meters
    }

}
