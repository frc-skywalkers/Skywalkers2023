// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            "frontLeft");

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            "frontRight");

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            "backleft");

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            "backRight");

    private final Pigeon2 imu = new Pigeon2(DriveConstants.kIMUPort);
    private final Limelight camera;
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), getModulePositions());

  private boolean fieldOriented = true;

  private final SwerveDrivePoseEstimator poseEstimator;

  private double resetX = 0.00;
  private double resetY = 0.00;

  private int resetCount = 0;
    
  

  public SwerveSubsystem() {
    camera = new Limelight();
    poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions(), new Pose2d(), VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    imu.configFactoryDefault();
    new Thread(() -> {
      try {
          Thread.sleep(1000);
          zeroHeading();
        } catch (Exception e) {
      }
    }).start();

    resetEncoders();
  }

  public void zeroHeading() {
    imu.setYaw(0);
  }

  public double getHeading() {
    double angle = imu.getYaw() % 360;
    if (angle > 180) {
      angle -= 360;
    } else if (angle <= -180) {
        angle += 360;
    }
    return angle;
  }



  public double getRoll() {
    return imu.getRoll();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    // resetX = pose.getX();
    // resetY = pose.getY();
    resetCount++;
  }

 
  @Override
  public void periodic() {

    // recalibrate();

    odometer.update(getRotation2d(), getModulePositions());

    Dashboard.Swerve.Driver.putString("Robot Location", getPose().getTranslation().toString());

    Dashboard.Swerve.Debugging.putNumber("Robot X Location", getPose().getTranslation().getX());
    Dashboard.Swerve.Debugging.putNumber("Robot Y Location", getPose().getTranslation().getY());
    Dashboard.Swerve.Debugging.putNumber("Robot Heading", getHeading());

    //Pose2d estimatedPose = camera.campose().toPose2d();

   /*  Dashboard.Swerve.Debugging.putNumber("Estimated X Location", estimatedPose.getX());
    Dashboard.Swerve.Debugging.putNumber("Estimated Y Location", estimatedPose.getY());
    Dashboard.Swerve.Debugging.putNumber("reset debugging", resetCount);

    poseEstimator.addVisionMeasurement(estimatedPose, Timer.getFPGATimestamp() - 0.3);*/

    Dashboard.Swerve.Driver.putBoolean("Field Oriented", fieldOriented);
    
    // Dashboard.Swerve.Debugging.putNumber("Robot X Reset", resetX);
    // Dashboard.Swerve.Debugging.putNumber("Robot Y Reset", resetY);

  } 

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    modulePositions[1] = frontLeft.getModulePosition();
    modulePositions[0] = frontRight.getModulePosition();
    modulePositions[3] = backLeft.getModulePosition();
    modulePositions[2] = backRight.getModulePosition();
    return modulePositions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    moduleStates[1] = frontLeft.getState();
    moduleStates[0] = frontRight.getState();
    moduleStates[3] = backLeft.getState();
    moduleStates[2] = backRight.getState();
    return moduleStates;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[1], false);
    frontRight.setDesiredState(desiredStates[0], false);
    backLeft.setDesiredState(desiredStates[3], false);
    backRight.setDesiredState(desiredStates[2], false);
  }

  public void setModuleStatesClosedLoop(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[1], true);
    frontRight.setDesiredState(desiredStates[0], true);
    backLeft.setDesiredState(desiredStates[3], true);
    backRight.setDesiredState(desiredStates[2], true);
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public void reset() {
    resetEncoders();
    zeroHeading();
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  public void reset(double heading) {
    resetEncoders();
    zeroHeading();
    resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.toRadians(heading))));
  }

  public void setHeading(double heading) {
    imu.setYaw(heading);
  }

  public void recalibrate() {
    frontLeft.recalibrateAbsoluteEncoderRad();
    frontRight.recalibrateAbsoluteEncoderRad();
    backLeft.recalibrateAbsoluteEncoderRad();
    backRight.recalibrateAbsoluteEncoderRad();
  }
 
  public void testMotors() {
    // for testing purposes test each swerve module individually at first
    // once one module has been completed, replace mentions of it with the next one you're testing it with
    SwerveModuleState testRun = new SwerveModuleState(1.0, Rotation2d.fromDegrees(90)); // change these values as necessary
    // first is speed in meteres per second, second is angle
    frontLeft.setDesiredState(testRun, false);
  }

  public void drive(double xSpeed, double ySpeed, double turningSpeed) {
    ChassisSpeeds chassisSpeeds;
    if (fieldOriented) {
      // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, turningSpeed, getRotation2d());
    } else {
      // Relative to robot
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

  public void toggleField() {
    fieldOriented = !fieldOriented;
  }

  public boolean getFieldOriented() {
    return fieldOriented;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }


}