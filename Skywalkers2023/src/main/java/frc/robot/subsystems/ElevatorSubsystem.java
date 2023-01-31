// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  public final WPI_TalonFX leftElevator = new WPI_TalonFX(ElevatorConstants.kLeftElevatorPort);
  public final WPI_TalonFX rightElevator = new WPI_TalonFX(ElevatorConstants.kRightElevatorPort);

  public ElevatorSubsystem() {
    leftElevator.setInverted(ElevatorConstants.kLeftElevatorInverted);
    rightElevator.setInverted(ElevatorConstants.kRightElevatorInverted);
    leftElevator.follow(rightElevator);
  }

  public void setVoltage(double voltage) {
    setSpeed(voltage/(double)12);
  }

  public void setSpeed(double speed) {
    rightElevator.set(speed);
  }

  public double getPosition() {
    return rightElevator.getSelectedSensorPosition() * ElevatorConstants.kConversionFactor;
  }

  public double getVelocity() {
    return rightElevator.getSelectedSensorVelocity() * ElevatorConstants.kConversionFactor;
  }

  public void moveUp() {
    setSpeed(ElevatorConstants.kMaxElevatorSpeed);
  }

  public void moveDown() {
    setSpeed(-ElevatorConstants.kMaxElevatorSpeed);
  }



  @Override
  public void periodic() {}
}
