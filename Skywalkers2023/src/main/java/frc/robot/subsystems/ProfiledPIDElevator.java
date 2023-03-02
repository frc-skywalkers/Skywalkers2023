

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ElevatorConstants;

public class ProfiledPIDElevator extends ProfiledPIDSubsystem {

  public final WPI_TalonFX leftElevator = new WPI_TalonFX(ElevatorConstants.kLeftElevatorPort, "CANivore");
  public final WPI_TalonFX rightElevator = new WPI_TalonFX(ElevatorConstants.kRightElevatorPort, "CANivore");
  LinearFilter homingMovingAvg = LinearFilter.movingAverage(8);

  public boolean isZeroed = false;
  
  public ProfiledPIDElevator() {
    super(
        new ProfiledPIDController(
            ElevatorConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAcc)));
            
    leftElevator.configFactoryDefault();
    rightElevator.configFactoryDefault();
    leftElevator.setInverted(ElevatorConstants.kLeftElevatorInverted);
    rightElevator.setInverted(ElevatorConstants.kRightElevatorInverted);
    leftElevator.setNeutralMode(NeutralMode.Brake);
    rightElevator.setNeutralMode(NeutralMode.Brake);

    leftElevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightElevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftElevator.configForwardSoftLimitThreshold(ElevatorConstants.kTopLimit / ElevatorConstants.kPositionConversionFactor, 0);
    leftElevator.configReverseSoftLimitThreshold(ElevatorConstants.kBottomLimit / ElevatorConstants.kPositionConversionFactor, 0);

    rightElevator.configForwardSoftLimitThreshold(ElevatorConstants.kTopLimit / ElevatorConstants.kPositionConversionFactor, 0);
    rightElevator.configReverseSoftLimitThreshold(ElevatorConstants.kBottomLimit / ElevatorConstants.kPositionConversionFactor, 0);
    resetEncoders();
    disable();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = 0;
    if (setpoint.velocity > 0) {
      feedforward = setpoint.velocity * ElevatorConstants.kVUp + ElevatorConstants.kSUp;
    } else {
      feedforward = setpoint.velocity * ElevatorConstants.kVDown + ElevatorConstants.kSDown;
    }

    if (isZeroed) {
      setVoltage(feedforward + output);
    } else {
      System.out.println("ELEVATOR NOT ZEROED!");
    }
    
    SmartDashboard.putNumber("Set Voltage", (feedforward + output));
  }

  @Override
  public double getMeasurement() {
    return getPosition();
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Elevator Current", getCurrent());
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putNumber("Elevator Velocity", getVelocity());
    SmartDashboard.putNumber("Elevator Voltage", leftElevator.getMotorOutputVoltage());
  }

  public void setVoltage(double voltage) {
    if (isZeroed) {
      voltage = MathUtil.clamp(voltage, -6, 6);
      rightElevator.setVoltage(voltage);
      leftElevator.setVoltage(voltage);
    } else {
      System.out.println("ELEVATOR NOT ZEROED!");
      stop();
    }
  }

  public void setSpeed(double speed) {
    speed = MathUtil.clamp(speed, -ElevatorConstants.kMaxElevatorSpeed, ElevatorConstants.kMaxElevatorSpeed);
    rightElevator.set(speed);
    leftElevator.set(speed);
  }

  public double getPosition() {
    return rightElevator.getSelectedSensorPosition() * ElevatorConstants.kPositionConversionFactor;
  }

  public double getVelocity() {
    return rightElevator.getSelectedSensorVelocity() * ElevatorConstants.kVelocityConversionFactor;
  }

  public double getCurrent() {
    return homingMovingAvg.calculate((rightElevator.getStatorCurrent() + leftElevator.getStatorCurrent()) / 2.0);
  }

  public void stop() {
    leftElevator.stopMotor();
    rightElevator.stopMotor();
  }

  public void resetEncoders() {
    rightElevator.setSelectedSensorPosition(0);
  }

  public void disableSoftLimits() {
    leftElevator.configForwardSoftLimitEnable(false, 0);
    rightElevator.configReverseSoftLimitEnable(false, 0);
  }

  public void enableSoftLimits() {
    leftElevator.configForwardSoftLimitEnable(true, 0);
    rightElevator.configReverseSoftLimitEnable(true, 0);
    System.out.println("Enabled");
  }

}
