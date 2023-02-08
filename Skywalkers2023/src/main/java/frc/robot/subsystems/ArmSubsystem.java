// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  WPI_TalonFX armMotor = new WPI_TalonFX(ArmConstants.kArmPort);

  double motorSpeed = 0.00;
  double scaleFactor = 1.00;

  public ArmSubsystem() {
    armMotor.setInverted(ArmConstants.kArmInverted);
  }

  public void setVoltage(double voltage) {
    setSpeed(voltage/(double)12.000);
  }

  public void setSpeed(double speed) {
    MathUtil.clamp(speed, -ArmConstants.kMaxArmSpeed, ArmConstants.kMaxArmSpeed);
    motorSpeed = speed;

    updateScaleFactor();

    armMotor.set(motorSpeed * scaleFactor);
  }

  public double getPosition() {
    return armMotor.getSelectedSensorPosition() * ArmConstants.kConversionFactor;
  }

  public double getVelocity() {
    return armMotor.getSelectedSensorVelocity() * ArmConstants.kConversionFactor;
  }

  public void moveArmUp() {
    setSpeed(ArmConstants.kMaxArmSpeed);
  }

  public void moveArmDown() {
    setSpeed(-ArmConstants.kMaxArmSpeed);
  }

  public void stop() {
    setSpeed(0.00);
  }

  private void updateScaleFactor() {
    double pos = getPosition();
    if(pos <= ArmConstants.kBottomLimit || pos >= ArmConstants.kTopLimit) {
      scaleFactor = 0.3;
    }
    else scaleFactor = 1.00;
  }

  public void resetEncoders() {
    armMotor.setSelectedSensorPosition(0);
  }

  public boolean isZeroed() {
    return armMotor.getSupplyCurrent() > ArmConstants.kCurrentThreshold;
  }

  @Override
  public void periodic() {


  }
}
