// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonFX intake = new WPI_TalonFX(IntakeConstants.kIntakePort);

  private double motorSpeed = 0.0;

  public IntakeSubsystem() {
    intake.configFactoryDefault();
  }

  // just speed should be fine, motor voltage unecessary

  public void setSpeed(double speed) {
    motorSpeed = speed;
    intake.set(motorSpeed);
  }

  public void moveIn() {
    setSpeed(IntakeConstants.kMaxIntakeSpeed);
  }

  public void moveOut() {
    setSpeed(-IntakeConstants.kMaxIntakeSpeed); 
  }

  public void stopIntake() {
    setSpeed(0.000);
  }

  public double getExpectedVelocity() {
    return motorSpeed * IntakeConstants.kExpectedFullVelocity;
  }

  public double getActualVelocity() {
    return intake.getSelectedSensorVelocity() / IntakeConstants.kIntakeTicksPerRotation;
  }

  public boolean objectHeld() {
    // double expected = getExpectedVelocity();
    // double actual = getActualVelocity();
    // double ratio = Math.abs(expected/actual);
    // return ratio > IntakeConstants.kObjectHeldRatioThreshold;
    return intake.getSupplyCurrent() > IntakeConstants.kCurrentThreshold;
  }



  @Override
  public void periodic() {
  }
}
