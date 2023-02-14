// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SensorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  public final WPI_TalonFX leftElevator = new WPI_TalonFX(ElevatorConstants.kLeftElevatorPort);
  public final WPI_TalonFX rightElevator = new WPI_TalonFX(ElevatorConstants.kRightElevatorPort);

  public final DigitalInput limitSwitch = new DigitalInput(SensorConstants.limitSwitchPort);
  public final DigitalInput beamBreaker = new DigitalInput(SensorConstants.beamBreakerPort);

  private double motorSpeed = 0;
  private double scaleFactor = 1.0;

  public ElevatorSubsystem() {
    leftElevator.setInverted(ElevatorConstants.kLeftElevatorInverted);
    rightElevator.setInverted(ElevatorConstants.kRightElevatorInverted);
    leftElevator.follow(rightElevator);
  }

  public void setVoltage(double voltage) {
    setSpeed(voltage/(double)12.000);
  }

  public void setSpeed(double speed) {
    MathUtil.clamp(speed, -ElevatorConstants.kMaxElevatorSpeed, ElevatorConstants.kMaxElevatorSpeed);

    motorSpeed = speed;
    updateScaleFactor();
    rightElevator.set(motorSpeed * scaleFactor);

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

  private void updateScaleFactor() {
    double pos = getPosition();
    if(pos <= ElevatorConstants.kBottomLimit || pos >= ElevatorConstants.kTopLimit) scaleFactor = 0.3;
    else scaleFactor = 1.0;
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

  public boolean getBeamBreaker() {
    return beamBreaker.get();
  }

  @Override
  public void periodic() {

  }
}
