// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonFX intake = new WPI_TalonFX(IntakeConstants.kIntakePort, "CANivore");
  private double intakeSpeed = 0;
  public boolean stop = false;

  public IntakeSubsystem() {
    intake.configFactoryDefault();
  }

  // just speed should be fine, motor voltage unecessary

  public void setSpeed(double speed) {
    intakeSpeed = speed;
    intake.set(intakeSpeed);
  }

  public void setVoltage(double voltage) {
    intake.setVoltage(voltage);
  }

  public void moveIn() {
    setSpeed(IntakeConstants.kMaxIntakeSpeed);
    intakeSpeed = IntakeConstants.kMaxIntakeSpeed;
  }

  public void moveOut() {
    setSpeed(-IntakeConstants.kMaxIntakeSpeed);
    intakeSpeed = -IntakeConstants.kMaxIntakeSpeed; 
  }

  public void stop() {
    stop = true;
    setSpeed(0.000);
    intakeSpeed = 0;
  }

  public double getActualVelocity() {
    return intake.getSelectedSensorVelocity();
  }

  public double getActualCurrent() {
    return intake.getStatorCurrent();
  }

  public boolean objectHeld() {
    // double expected = getExpectedVelocity();
    // double actual = getActualVelocity();
    // double ratio = Math.abs(expected/actual);
    // return ratio > IntakeConstants.kObjectHeldRatioThreshold;
    return getActualCurrent() > IntakeConstants.kObjectHeldThreshold * IntakeConstants.kMaxIntakeSpeed;
  }

  public boolean speedUp() {
    return getActualCurrent() > IntakeConstants.kSpeedUpThreshold * IntakeConstants.kMaxIntakeSpeed;
  }

  public boolean objectOut() {
    return getActualCurrent() < IntakeConstants.kObjectOutThreshold * IntakeConstants.kMaxIntakeSpeed;
  }

  @Override
  public void periodic() {
    /*SmartDashboard.putNumber("Velocity", getActualVelocity());
    SmartDashboard.putNumber("Speed", intakeSpeed);
    SmartDashboard.putBoolean("Object Held", objectHeld());
    SmartDashboard.putNumber("Current", intake.getStatorCurrent());
    System.out.println("Current: " + intake.getStatorCurrent());*/
  }
}
