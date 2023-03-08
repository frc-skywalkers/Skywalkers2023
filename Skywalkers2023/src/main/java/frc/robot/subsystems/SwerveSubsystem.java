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
import frc.robot.subsystems.Limelight;

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
    
  

  public SwerveSubsystem() {
    camera = new Limelight();
    poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions(), new Pose2d(), VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    new Thread(() -> {
      try {
          Thread.sleep(1000);
          zeroHeading();
        } catch (Exception e) {
      }
    }).start();
  }

  public void zeroHeading() {
    imu.setYaw(0);
  }

  public double getHeading() {
    double[] YPR = new double[3];
    // imu.getYawPitchRoll(YPR);
    imu.getYawPitchRoll(YPR);
    double ret = YPR[0];
    while(ret >= 360.01) ret -= 360.00;
    return ret;
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

  public Pose2d invertYPose(Pose2d pose) {
    return new Pose2d(pose.getX(), pose.getY(), pose.getRotation());
  }

  public Pose2d getTrajectoryPose() {
    Pose2d actualPose = getPose();
    Pose2d dumbPose = new Pose2d(actualPose.getX(), actualPose.getY(), actualPose.getRotation());
    return dumbPose;
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

 
  @Override
  public void periodic() {

    recalibrate();

    odometer.update(getRotation2d(), getModulePositions());

    Dashboard.Swerve.Driver.putString("Robot Location", getPose().getTranslation().toString());

    Dashboard.Swerve.Debugging.putNumber("Robot X Location", Units.metersToInches(getPose().getTranslation().getX()));
    Dashboard.Swerve.Debugging.putNumber("Robot Y Location", Units.metersToInches(getPose().getTranslation().getY()));
    Dashboard.Swerve.Debugging.putNumber("Robot Heading", getHeading());

    Pose2d estimatedPose = camera.campose().toPose2d();

    Dashboard.Swerve.Debugging.putNumber("Estimated X Location", estimatedPose.getX());
    Dashboard.Swerve.Debugging.putNumber("Estimated Y Location", estimatedPose.getY());

    poseEstimator.addVisionMeasurement(estimatedPose, Timer.getFPGATimestamp() - 0.3);

    // Swerve.Debugging.putNumber("Robot Heading", getHeading());
    Dashboard.Swerve.Driver.putBoolean("Field Oriented", fieldOriented);
    // Dashboard.Swerve.Debugging.putNumber("Pitch", getRoll());

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

}