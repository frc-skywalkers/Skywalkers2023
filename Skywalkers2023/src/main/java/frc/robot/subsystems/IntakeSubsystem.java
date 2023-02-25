// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonFX intake = new WPI_TalonFX(IntakeConstants.kIntakePort, "CANivore");

  private double motorSpeed = 0.0;

  public IntakeSubsystem() {
    // intake.configFactoryDefault();
    intake.setNeutralMode(NeutralMode.Brake);
    intake.setInverted(IntakeConstants.kIntakeInverted);
    // intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 75, 1));
  }

  // just speed should be fine, motor voltage unecessary

  public void setSpeed(double speed) {
    motorSpeed = speed;
    intake.set(motorSpeed);
  }

  public void setVoltage(double voltage) {
    intake.setVoltage(voltage);
  }

  public void moveIn() {
    setSpeed(IntakeConstants.kMaxIntakeSpeed);
    SmartDashboard.putString("Intake Status", "Moving");
  }

  public void moveOut() {
    // setSpeed(-IntakeConstants.kMaxOuttakeSpeed); 
    setVoltage(-IntakeConstants.kMaxOuttakeSpeed * 12.000);
    SmartDashboard.putString("Intake Status", "Moving");

  }

  public void stopIntake() {
    setSpeed(IntakeConstants.kHoldSpeed);
    // setVoltage(12.00 * IntakeConstants.k)
    SmartDashboard.putString("Intake Status", "Stopped");

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
    SmartDashboard.putNumber("intake velocity", getActualVelocity());
    SmartDashboard.putNumber("Intake Stator Current", intake.getStatorCurrent());
    SmartDashboard.putNumber("Intake Supply Current", intake.getSupplyCurrent());
  }
}
