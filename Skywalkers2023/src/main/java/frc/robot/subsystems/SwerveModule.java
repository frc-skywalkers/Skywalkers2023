// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */


  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turningMotor;

  private final PIDController turningPidController;
  // private final PIDController drivingPidController;

  private final CANCoder absoluteEncoder;

  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;
  
  private final String motorId;

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String motorId) {
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;

    this.motorId = motorId;

    driveMotor = new WPI_TalonFX(driveMotorId);
    turningMotor = new WPI_TalonFX(turningMotorId);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveMotor.setNeutralMode(NeutralMode.Brake);
    turningMotor.setNeutralMode(NeutralMode.Brake);

    absoluteEncoder = new CANCoder(absoluteEncoderId);

    // CANCoderConfiguration absoluteConfig = new CANewaCoderConfiguration();
    
    // absoluteConfig.sensorCoefficient = 2 * 


    // driveEncoder = new CANCoder(driveEncoderId, "rio");
    // turningEncoder = new CANCoder(turningEncoderId, "rio");
    
    // CANCoderConfiguration configDrive = new CANCoderConfiguration();

    // configDrive.sensorCoefficient = ModuleConstants.kWheelDiameter * Math.PI / ModuleConstants.kTicksPerRotation;
    // configDrive.unitString = "meter";
    // configDrive.sensorTimeBase = SensorTimeBase.PerSecond;
    
    // CANCoderConfiguration configTurn = new CANCoderConfiguration();

    // configTurn.sensorCoefficient = 2 * Math.PI / ModuleConstants.kTicksPerRotation;
    // configTurn.unitString = "rad";
    // configTurn.sensorTimeBase = SensorTimeBase.PerSecond;

    // driveEncoder.configAllSettings(configDrive);
    // turningEncoder.configAllSettings(configTurn);



    turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    // drivingPidController = new PIDController(ModuleConstants.kPDriving, ModuleConstants.kIDriving, ModuleConstants.kDDriving);
    // drivingPidController.enableContinuousInput(-0.8, 0.8);
    turningPidController.setTolerance(0.03);

    resetEncoders();
  }

  // get functions hella sus

  public double getDrivePosition() {
    return driveMotor.getSelectedSensorPosition() * Math.PI * ModuleConstants.kWheelDiameter / ModuleConstants.kDriveTicksPerRotation;
  }

  public double getTurningPosition() {
    return turningMotor.getSelectedSensorPosition() * Math.PI * 2.0 / ModuleConstants.kTicksPerRotation;
  } 

  public double getDriveVelocity() {
    return driveMotor.getSelectedSensorVelocity() * Math.PI * ModuleConstants.kWheelDiameter / ModuleConstants.kDriveTicksPerRotation * 10;
  }

  public double getTurningVelocity() {
    return turningMotor.getSelectedSensorVelocity() * Math.PI * 2.0 / ModuleConstants.kTicksPerRotation;
  }

  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getAbsolutePosition() * 2.0 * Math.PI / 360.000; // might need to correct??????
    angle -= absoluteEncoderOffsetRad;
    if (absoluteEncoderReversed) angle *= -1.0;
    return angle;
  }

  public double recalibrateAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getAbsolutePosition() * 2.0 * Math.PI / 360.000; // might need to correct??????
    if (absoluteEncoderReversed) angle *= -1.0;
    Dashboard.Swerve.Debugging.putNumber("test " + motorId, angle);
    return angle;
  }

  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
    turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad() / (2.000 * Math.PI) * ModuleConstants.kTicksPerRotation);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public void setDesiredState(SwerveModuleState state, boolean closedLoop) {
    if(Math.abs(state.speedMetersPerSecond) < 0.1) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    if(!closedLoop) {
      driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond); // goofy ahh way could do pid here as well but idk lmfao
    }
    else {
      double vol;
      double desiredSpeed = state.speedMetersPerSecond;
      if(desiredSpeed > 0.00) {
        vol = state.speedMetersPerSecond * ModuleConstants.kVDrive + ModuleConstants.kSDrive;
      }
      else {
        vol = state.speedMetersPerSecond * ModuleConstants.kVDrive - ModuleConstants.kSDrive;
      }

      driveMotor.setVoltage(vol);
    }
    // double sgn = Math.abs(state.speedMetersPerSecond) / state.speedMetersPerSecond;
    // driveMotor.setVoltage(5.0 * sgn);
    double turnSpeed = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
    turnSpeed = MathUtil.clamp(turnSpeed, -0.4, 0.4);
    turningMotor.set(turnSpeed);
    Dashboard.Swerve.Debugging.putNumber(motorId + " goal Angle", state.angle.getRadians());
    // Dashboard.Swerve.Debugging.putNumber(motorId + " actual Angle", getTurningPosition());
    double dif = state.angle.getRadians() - getTurningPosition();
    dif = Math.abs(dif);
    while(dif > Math.PI) dif -= 2.0 * Math.PI;
    dif = Math.abs(dif);
    Dashboard.Swerve.Debugging.putNumber(motorId + " dif Angle", dif);

  }

  public SwerveModulePosition getModulePosition() {
    SwerveModulePosition modulePosition = new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getTurningPosition()));
    return modulePosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Dashboard.Swerve.Debugging.putNumber(motorId + " drive Position", getDrivePosition());
    Dashboard.Swerve.Debugging.putNumber(motorId + " turning Position", getTurningPosition());
    Dashboard.Swerve.Debugging.putNumber(motorId + " absolute Position", getAbsoluteEncoderRad());
    Dashboard.Swerve.Debugging.putNumber(motorId + " drive Speed", getDriveVelocity());
  }
}