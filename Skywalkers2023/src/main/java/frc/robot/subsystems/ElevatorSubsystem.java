

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
  double scaleFactor;
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
    // resetEncoders();
  }

  public void setVoltage(double voltage) {
    rightElevator.setVoltage(voltage * scaleFactor);
    leftElevator.setVoltage(voltage * scaleFactor);
  }

  public void setSpeed(double speed) {
    speed = MathUtil.clamp(speed, -ElevatorConstants.kMaxElevatorSpeed, ElevatorConstants.kMaxElevatorSpeed);
    rightElevator.set(speed * scaleFactor);
    leftElevator.set(speed * scaleFactor);
  }

  public void updateScaleFactor(double speed) {
    scaleFactor = 1;
    // if ((getPosition() >= 1.48 && speed > 0.1) || (getPosition() <= 0.05 && speed < -0.1)) {
      // scaleFactor = 0.3;
    // }
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

  public void moveUp() {
    setVoltage(5);
  }

  public void moveDown() {
    setVoltage(-0.5);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    SmartDashboard.putNumber("Elevator Current", getCurrent());
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putNumber("Elevator Velocity", getVelocity());
  }
}