

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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Dashboard;
import frc.robot.Constants.ElevatorConstants;

public class ProfiledPIDElevator extends ProfiledPIDSubsystem {

  public final WPI_TalonFX leftElevator = new WPI_TalonFX(ElevatorConstants.kLeftElevatorPort, "CANivore");
  public final WPI_TalonFX rightElevator = new WPI_TalonFX(ElevatorConstants.kRightElevatorPort, "CANivore");
  LinearFilter homingMovingAvg = LinearFilter.movingAverage(8);

  public boolean isZeroed = false;
  private boolean softLimitsEnabled = false;
  
  public ProfiledPIDElevator() {
    super(
        new ProfiledPIDController(
            ElevatorConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAcc)));

    this.getController().setTolerance(0.02);
            
    leftElevator.configFactoryDefault();
    rightElevator.configFactoryDefault();
    leftElevator.setInverted(ElevatorConstants.kLeftElevatorInverted);
    rightElevator.setInverted(ElevatorConstants.kRightElevatorInverted);
    leftElevator.setNeutralMode(NeutralMode.Brake);
    rightElevator.setNeutralMode(NeutralMode.Brake);

    leftElevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightElevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftElevator.configForwardSoftLimitThreshold(ElevatorConstants.kTopLimit / ElevatorConstants.kPositionConversionFactor, 0);
    leftElevator.configReverseSoftLimitThreshold(ElevatorConstants.kBottomLimit / ElevatorConstants.kPositionConversionFactor, 0);

    rightElevator.configForwardSoftLimitThreshold(ElevatorConstants.kTopLimit / ElevatorConstants.kPositionConversionFactor, 0);
    rightElevator.configReverseSoftLimitThreshold(ElevatorConstants.kBottomLimit / ElevatorConstants.kPositionConversionFactor, 0);
    resetEncoders();
    disable();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = 0;
    if (setpoint.velocity > 0) {
      feedforward = setpoint.velocity * ElevatorConstants.kVUp + ElevatorConstants.kSUp;
    } else {
      feedforward = setpoint.velocity * ElevatorConstants.kVDown + ElevatorConstants.kSDown;
    }

    if (isZeroed) {
      setVoltage(feedforward + output);
    } else {
      // System.out.println("ELEVATOR NOT ZEROED!");
    }
    
    //Dashboard.Elevator.Debugging.putNumber("Set Voltage", (feedforward + output));
  }

  @Override
  public double getMeasurement() {
    return getPosition();
  }

  public CommandBase goToPosition(double position) {
    return Commands.runOnce(() -> {
      this.setGoal(position);
      this.enable();
    }, this).andThen(Commands.waitUntil(this::atGoal));
  }

  @Override
  public void periodic() {
    super.periodic();
    if (softLimitsEnabled) {
      if (getPosition() > ElevatorConstants.kTopLimit && rightElevator.getMotorOutputVoltage() > 0) {
        stop();
      }
      if (getPosition() < ElevatorConstants.kBottomLimit && rightElevator.getMotorOutputVoltage() < 0) {
        stop();
      }
  }
    Dashboard.Elevator.Debugging.putNumber("Elevator Current", getCurrent());
    Dashboard.Elevator.Debugging.putNumber("Elevator Position", getPosition());
    Dashboard.Elevator.Debugging.putNumber("Elevator Velocity", getVelocity());
    Dashboard.Elevator.Debugging.putNumber("Elevator Voltage", leftElevator.getMotorOutputVoltage());
    Dashboard.Elevator.Driver.putBoolean("Elevator Goal Reached", this.getController().atGoal());
  }

  public void setVoltage(double voltage) {
    if (isZeroed) {
      voltage = MathUtil.clamp(voltage, -8, 8);
      rightElevator.setVoltage(voltage);
      leftElevator.setVoltage(voltage);
    } else {
      // System.out.println("ELEVATOR NOT ZEROED!");
      stop();
    }
  }

  public void setSpeed(double speed) {
    speed = MathUtil.clamp(speed, -ElevatorConstants.kMaxElevatorSpeed, ElevatorConstants.kMaxElevatorSpeed);
    rightElevator.set(speed);
    leftElevator.set(speed);
    if (softLimitsEnabled) {
      if (getPosition() > ElevatorConstants.kTopLimit && speed > 0) {
        stop();
      }
      if (getPosition() < ElevatorConstants.kBottomLimit && speed < 0) {
        stop();
      }
    }
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

  public void disableSoftLimits() {
    softLimitsEnabled = false;
  }

  public void enableSoftLimits() {
    softLimitsEnabled = true;
  }

  public boolean atGoal() {
    // Dashboard.Elevator.Driver.putBoolean("Elevator Goal Reached", this.getController().atGoal());
    return Math.abs(getPosition() - this.getController().getGoal().position) <= 0.07;

  }

}
