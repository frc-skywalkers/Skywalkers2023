// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubystem extends SubsystemBase {

  private final WPI_TalonFX leftIntake = new WPI_TalonFX(IntakeConstants.kLeftIntakePort);
  private final WPI_TalonFX rightIntake = new WPI_TalonFX(IntakeConstants.kRightIntakePort);

  private double motorSpeed = 0.0;

  public IntakeSubystem() {
    leftIntake.setInverted(IntakeConstants.kLeftIntakeInverted);
    rightIntake.setInverted(IntakeConstants.kRightIntakeInverted);
    leftIntake.follow(rightIntake);
  }

  // just speed should be fine, motor voltage unecessary

  public void setSpeed(double speed) {
    motorSpeed = speed;
  }

  public void moveUp() {
    setSpeed(IntakeConstants.kMaxIntakeSpeed);
  }

  public void moveDown() {
    setSpeed(-IntakeConstants.kMaxIntakeSpeed);
  }

  public void stopIntake() {
    setSpeed(0.000);
  }

  public double getExpectedVelocity() {
    return motorSpeed * IntakeConstants.kExpectedFullVelocity;
  }

  public double getActualVelocity() {
    return rightIntake.getSelectedSensorVelocity() / IntakeConstants.kIntakeGearRatio;
  }

  public boolean objectHeld() {
    double expected = getExpectedVelocity();
    double actual = getActualVelocity();
    double ratio = Math.abs(expected/actual);
    return ratio > IntakeConstants.kObjectHeldRatioThreshold;
  }



  @Override
  public void periodic() {
    rightIntake.set(motorSpeed);
  }
}
