// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SensorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  public final WPI_TalonFX leftElevator = new WPI_TalonFX(ElevatorConstants.kLeftElevatorPort, "CANivore");
  public final WPI_TalonFX rightElevator = new WPI_TalonFX(ElevatorConstants.kRightElevatorPort, "CANivore");

  LinearFilter homingMovingAvg = LinearFilter.movingAverage(8);

  public final DigitalInput limitSwitch = new DigitalInput(SensorConstants.limitSwitchPort);

  public ElevatorSubsystem() {
    leftElevator.configFactoryDefault();
    rightElevator.configFactoryDefault();
    leftElevator.setInverted(ElevatorConstants.kLeftElevatorInverted);
    rightElevator.setInverted(ElevatorConstants.kRightElevatorInverted);
    leftElevator.setNeutralMode(NeutralMode.Brake);
    rightElevator.setNeutralMode(NeutralMode.Brake);

    leftElevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightElevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    resetEncoders();
  }

  public void setVoltage(double voltage) {
    rightElevator.setVoltage(voltage);
    leftElevator.setVoltage(voltage);
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

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    SmartDashboard.putNumber("Elevator Current", getCurrent());
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putNumber("Elevator Velocity", getVelocity());
  }
}
