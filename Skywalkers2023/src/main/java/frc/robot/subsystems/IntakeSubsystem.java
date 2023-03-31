// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.lightstripConstants;
import frc.robot.lightstrip.LedState;

public class IntakeSubsystem extends SubsystemBase {
  public final static int conePiece = 1;
  public final static int cubePiece = -1;

  public Mode mode = Mode.CUBE;

  private final boolean differentialIntake = IntakeConstants.differentialIntake;

  private final WPI_TalonFX intake = new WPI_TalonFX(IntakeConstants.kIntakePort, "CANivore");
  private double intakeSpeed = 0;
  public boolean stop = false;

  public boolean outtake = false;

  private final Lightstrip lightstrip;

  public enum Mode {
    CONE(-1),
    CUBE(1);

    public int multiplier;

    Mode(int m) {
      multiplier = m;
    }
  }

  public IntakeSubsystem(Lightstrip rLightstrip) {
    intake.configFactoryDefault();
    lightstrip = rLightstrip;
  }

  // just speed should be fine, motor voltage unecessary

  public void setSpeed(double speed) {
    intakeSpeed = speed;
    intake.set(intakeSpeed);
    Dashboard.Intake.Debugging.putNumber("Intake Set Speed", intakeSpeed);
  }

  public void setVoltage(double voltage) {
    intake.setVoltage(voltage);
    Dashboard.Intake.Debugging.putNumber("Intake Set Voltage", voltage);
  }

  public void moveIn() {
    setSpeed(IntakeConstants.kMaxIntakeSpeed * mode.multiplier);
    intakeSpeed = IntakeConstants.kMaxIntakeSpeed * mode.multiplier;
  }

  public void moveOut() {
    setSpeed(IntakeConstants.kMaxOuttakeSpeed * mode.multiplier);
    intakeSpeed = IntakeConstants.kMaxOuttakeSpeed * mode.multiplier;
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
    if(mode == Mode.CONE) {
      return getActualVelocity() < IntakeConstants.conePieceHeldThreshold;
    } else {
      return getActualVelocity() < IntakeConstants.cubePieceHeldThreshold;
    }
  }

  private double getThreshold() {
    if(mode == Mode.CONE) {
      return IntakeConstants.conePieceHeldThreshold;
    } else {
      return IntakeConstants.cubePieceHeldThreshold;
    }
  }

  public void holdObject() {
    intake.setVoltage(IntakeConstants.kHoldSpeed * 12.0000 * mode.multiplier);
  }

  public void setMode(Mode m) {
    this.mode = m;
  }

  public Mode getMode() {
    return mode;
  }

  public void toggleMode() {
    if (mode == Mode.CONE) {
      setMode(Mode.CUBE);
    } else if (mode == Mode.CUBE) {
      setMode(Mode.CONE);
    }
  }

  public double getSpeed() {
    return intake.get();
  }

  @Override
  public void periodic() {
    if(mode == Mode.CONE) {
      lightstrip.setColor(lightstripConstants.coneIntake);
    } else if(mode == Mode.CUBE) {
      lightstrip.setColor(lightstripConstants.cubeIntake);
    }


    Dashboard.Intake.Debugging.putNumber("Intake Velocity", getActualVelocity());
    Dashboard.Intake.Debugging.putNumber("Intake Speed", intakeSpeed);
    Dashboard.Intake.Debugging.putNumber("Intake Current", intake.getStatorCurrent());
    Dashboard.Intake.Driver.putString("Intake Mode", mode.toString());
    Dashboard.Intake.Debugging.putNumber(getName(), intakeSpeed);
  }
}
