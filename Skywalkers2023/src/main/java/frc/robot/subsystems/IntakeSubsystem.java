// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  public final static int conePiece = 1;
  public final static int cubePiece = -1;

  private int currentPiece = -1;

  private final boolean differentialIntake = IntakeConstants.differentialIntake;

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

  public void moveIn(int piece) {
    setSpeed(IntakeConstants.kMaxIntakeSpeed * piece);
    intakeSpeed = IntakeConstants.kMaxIntakeSpeed * piece;
  }

  public void moveOut(int piece) {
    setSpeed(IntakeConstants.kMaxOuttakeSpeed * piece);
    intakeSpeed = IntakeConstants.kMaxOuttakeSpeed * piece;
  }

  public void stop() {
    stop = true;
    setSpeed(0.000);
    intakeSpeed = 0;
  }

  public double getActualVelocity() {
    return Math.abs(intake.getSelectedSensorVelocity());
  }

  public double getActualCurrent() {
    return intake.getStatorCurrent();
  }

  public boolean intakeEmpty() {
    return getActualVelocity() > IntakeConstants.threshold(intakeSpeed) && (getActualVelocity() < 0 == intakeSpeed < 0);
  }

  public boolean pieceHeld() {
    return getActualVelocity() < IntakeConstants.pieceHeldThreshold && (getActualVelocity() < 0 == intakeSpeed < 0);
  }

  public void holdObject() {
    intake.setVoltage(IntakeConstants.kHoldSpeed * 12.0000);
  }



  public int getPiece() {
    return currentPiece;
  }

  public void setCone() {
    currentPiece = conePiece;
  }

  public void setCube() {
    currentPiece = cubePiece;
  }

  @Override
  public void periodic() {
    Dashboard.Intake.Debugging.putNumber("Intake Velocity", getActualVelocity());
    Dashboard.Intake.Debugging.putNumber("Intake Speed", intakeSpeed);
    Dashboard.Intake.Debugging.putBoolean("Intake Object Held", pieceHeld());
    Dashboard.Intake.Debugging.putNumber("Intake Current", intake.getStatorCurrent());
    Dashboard.Intake.Debugging.putNumber("Current Piece", currentPiece);
    Dashboard.Intake.Debugging.putNumber(getName(), intakeSpeed);
  }
}
