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
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SensorConstants;

public class ProfiledPIDElevator extends ProfiledPIDSubsystem {
  /** Creates a new ProfiledPIDElevator. */

  public final WPI_TalonFX leftElevator = new WPI_TalonFX(ElevatorConstants.kLeftElevatorPort, "CANivore");
  public final WPI_TalonFX rightElevator = new WPI_TalonFX(ElevatorConstants.kRightElevatorPort, "CANivore");

  LinearFilter homingMovingAvg = LinearFilter.movingAverage(8);
  
  public ProfiledPIDElevator() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0.25, 0.25)));
            
    leftElevator.configFactoryDefault();
    rightElevator.configFactoryDefault();
    leftElevator.setInverted(ElevatorConstants.kLeftElevatorInverted);
    rightElevator.setInverted(ElevatorConstants.kRightElevatorInverted);
    leftElevator.setNeutralMode(NeutralMode.Brake);
    rightElevator.setNeutralMode(NeutralMode.Brake);

    leftElevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightElevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    disable();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = 0;
    if (setpoint.velocity > 0) {
      feedforward = setpoint.velocity * 6.17 + 0.999;
    } else {
      feedforward = setpoint.velocity * 6.08 + 0.47;
    }

    setVoltage(feedforward + output);
  }

  @Override
  public double getMeasurement() {
    return getPosition();
  }

  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -2, 2);
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

}
