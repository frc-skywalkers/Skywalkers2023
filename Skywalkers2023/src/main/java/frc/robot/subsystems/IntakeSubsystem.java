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

  public Piece currentPiece = Piece.NONE;
  public Piece desiredPiece = Piece.CUBE;

  private final boolean differentialIntake = IntakeConstants.differentialIntake;

  private final WPI_TalonFX intake = new WPI_TalonFX(IntakeConstants.kIntakePort, "CANivore");
  private double intakeSpeed = 0;
  public boolean stop = false;

  public boolean outtake = false;

  public enum Piece {
    NONE(0),
    CONE(1),
    CUBE(-1);

    public int multiplier;

    Piece(int m) {
      multiplier = m;
    }
  }

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

  public void moveIn(Piece piece) {
    setSpeed(IntakeConstants.kMaxIntakeSpeed * piece.multiplier);
    intakeSpeed = IntakeConstants.kMaxIntakeSpeed * piece.multiplier;
  }

  public void moveOut(Piece piece) {
    setSpeed(IntakeConstants.kMaxOuttakeSpeed * piece.multiplier);
    intakeSpeed = IntakeConstants.kMaxOuttakeSpeed * piece.multiplier;
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
    return getActualVelocity() > IntakeConstants.threshold(intakeSpeed);
  }

  public boolean pieceHeld() {
    return getActualVelocity() < IntakeConstants.pieceHeldThreshold;
  }

  public void holdObject() {
    intake.setVoltage(IntakeConstants.kHoldSpeed * 12.0000 * currentPiece.multiplier);
  }

  public Piece getDesiredPiece() {
    return desiredPiece;
  }

  public void setDesiredPiece(Piece p) {
    desiredPiece = p;
  }

  public void setCurrentPiece(Piece p) {
    currentPiece = p;
  }

  public Piece getCurrentPiece() {
    return currentPiece;
  }

  @Override
  public void periodic() {
    Dashboard.Intake.Debugging.putNumber("Intake Velocity", getActualVelocity());
    Dashboard.Intake.Debugging.putNumber("Intake Speed", intakeSpeed);
    Dashboard.Intake.Debugging.putBoolean("Intake Object Held", pieceHeld());
    Dashboard.Intake.Debugging.putNumber("Intake Current", intake.getStatorCurrent());
    Dashboard.Intake.Driver.putString("Desired Piece", desiredPiece.toString());
    Dashboard.Intake.Driver.putString("Current Piece", currentPiece.toString());
    Dashboard.Intake.Debugging.putNumber(getName(), intakeSpeed);
  }
}
