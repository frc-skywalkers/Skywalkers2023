// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SensorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  public final WPI_TalonFX leftElevator = new WPI_TalonFX(ElevatorConstants.kLeftElevatorPort, "CANivore");
  public final WPI_TalonFX rightElevator = new WPI_TalonFX(ElevatorConstants.kRightElevatorPort, "CANivore");

  public final DigitalInput limitSwitch = new DigitalInput(SensorConstants.limitSwitchPort);

  public ElevatorSubsystem() {
    leftElevator.setInverted(ElevatorConstants.kLeftElevatorInverted);
    rightElevator.setInverted(ElevatorConstants.kRightElevatorInverted);
    // leftElevator.follow(rightElevator);
    leftElevator.setNeutralMode(NeutralMode.Brake);
    rightElevator.setNeutralMode(NeutralMode.Brake);
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
    return rightElevator.getSelectedSensorPosition() * ElevatorConstants.kConversionFactor;
  }

  public double getVelocity() {
    return rightElevator.getSelectedSensorVelocity() * ElevatorConstants.kConversionFactor;
  }

  public void stop() {
    setSpeed(0);
  }

  public void resetEncoders() {
    rightElevator.setSelectedSensorPosition(0);
  }

  public boolean isZeroed() {
    return rightElevator.getSupplyCurrent() > ElevatorConstants.kCurrentThreshold;
    //return this.getLimitSwitch();
    //return !this.getBeamBreaker();
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    SmartDashboard.putNumber("Left Elevator Current", leftElevator.getStatorCurrent());
    SmartDashboard.putNumber("Right Elevator Current", rightElevator.getStatorCurrent());
  }
}
